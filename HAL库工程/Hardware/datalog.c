#include "datalog.h"
#include "w25q64.h"
#include "at24c64_slot.h"
#include "delay.h"      /* GetTick()：书签写限流判据 */
#include <stdio.h>

/*
 * 阶段9：断网缓存实现（设计要点见 datalog.h 注释）
 *
 * 指针模型：
 *   写点 wp：下一条记录的写入位置（扇区号+扇区内偏移）
 *   消费点 rp：最旧一条未补传记录的位置，存 AT24C64 双槽书签（阶段10 起，
 *   由 RTC 备份寄存器切换而来——没装 CR1220 电池断电也不再丢补传进度）
 *   启动扫描：从 rp 起按环形找第一个"槽全FF"的位置即 wp——已消费但未擦的
 *   扇区记录仍然合法，扫描会正确跳过；全FF即"没写过"。
 * 懒擦除（lazy erase）：wp 推进到扇区头时才擦该扇区。擦除前若 rp 还在本扇区
 *   内（扇区里有未补传记录），先丢掉本扇区最旧记录（rp 跳出）再擦——环形满
 *   时"丢最旧"以扇区为单位，天然磨损均衡。
 *
 * 记录布局（显式字节打包，与编译器对齐无关——曾因结构体对齐+CRC把自身算进
 * 输入，导致写入的记录永远校验失败、补传全部静默丢弃）：
 *   [0..3]  magic "DGL1"   [4..7] ts(小端)  [8..9] t10  [10..11] h10
 *   [12..27] 0x00          [28..29] CRC16(前28字节,小端)  [30..31] 0xFF
 */
#define DL_BKP_MAGIC  0xD1CE5A00UL   /* 保留：RTC 书签魔数（切 AT24C64 后不再使用） */

/* 书签写限流：距上次真正写 EEPROM ≥30s 才落盘（补传风暴 4.3万条也封顶 ~2次/分，
 * AT24C64 100万次寿命 → 约68年；代价=掉电最多丢30s补传进度，服务端按 ts 去重） */
#define DL_BM_THROTTLE_MS   30000UL

static uint8_t  s_ready = 0;           /* Flash 在位且初始化成功 */
static uint32_t s_wpSec, s_wpOff;      /* 写点 */
static uint32_t s_rpSec, s_rpOff;      /* 消费点（书签） */
static uint32_t s_count;               /* 待补传条数 */
static uint32_t s_bmLastWrite = 0;     /* 上次真正写 EEPROM 书签的时刻（0=从未） */

/* CRC16（Modbus 0xA001，与 modbus.c 同算法，独立实现避免耦合） */
static uint16_t dl_crc16(const uint8_t *p, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    int k;
    while (len--)
    {
        crc ^= *p++;
        for (k = 0; k < 8; k++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

/* 打包一条记录到 32 字节缓冲（字节序仅内部约定，读写两侧一致即可） */
static void dl_pack(uint8_t *buf, uint32_t ts, int16_t t10, uint16_t h10)
{
    uint16_t crc;
    uint32_t i;

    // ========== 0~3字节：魔数Magic，固定"DGL1"，用来快速判断这条记录是不是合法日志 ==========
    buf[0] = 0x44;   // 'D'
    buf[1] = 0x4C;   // 'L'
    buf[2] = 0x47;   // 'G'
    buf[3] = 0x31;   // '1'

    // ========== 4~7字节：Unix时间戳 ts (uint32，小端存储)，采集这条数据的时间 ==========
    buf[4] = (uint8_t)ts;                  // ts 低字节
    buf[5] = (uint8_t)(ts >> 8);           // ts 第2字节
    buf[6] = (uint8_t)(ts >> 16);          // ts 第3字节
    buf[7] = (uint8_t)(ts >> 24);          // ts 高字节

    // ========== 8~9字节：温度 t10 (int16，放大10倍，小端) ==========
    // 例：25.5℃ 存255；支持负数，-10.2℃存 -102
    buf[8] = (uint8_t)t10;                          // t10低字节
    buf[9] = (uint8_t)((uint16_t)t10 >> 8);         // t10高字节

    // ========== 10~11字节：湿度 h10 (uint16，放大10倍，小端) ==========
    // 例：60.5%RH 存605；湿度无负数
    buf[10] = (uint8_t)h10;                         // h10低字节
    buf[11] = (uint8_t)(h10 >> 8);                  // h10高字节

    // ========== 12~27字节：预留填充区，共16字节，全部填0，方便以后扩展字段 ==========
    for (i = 12; i < 28; i++)
        buf[i] = 0x00;

    // ========== 计算CRC16校验：只对【前面0~27，共28个字节】做校验 ==========
    // ⚠重点：CRC字段本身不参与CRC计算！
    crc = dl_crc16(buf, 28);

    // ========== 28~29字节：存放CRC16结果（小端） ==========
    buf[28] = (uint8_t)crc;                 // crc低字节
    buf[29] = (uint8_t)(crc >> 8);          // crc高字节

    // ========== 30~31字节：末尾固定标记0xFF，上电扫描判断槽是否空白用 ==========
    buf[30] = 0xFF;
    buf[31] = 0xFF;
}

/* 校验+解包一条记录：返回1=合法 */
static uint8_t dl_unpack(const uint8_t *buf, uint32_t *ts, int16_t *t10, uint16_t *h10)
{
    uint16_t crc;

    if (buf[0] != 0x44 || buf[1] != 0x4C || buf[2] != 0x47 || buf[3] != 0x31)
        return 0;
    crc = (uint16_t)buf[28] | ((uint16_t)buf[29] << 8);
    if (dl_crc16(buf, 28) != crc)
        return 0;
    *ts  = (uint32_t)buf[4] | ((uint32_t)buf[5] << 8)
         | ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24);
    *t10 = (int16_t)((uint16_t)buf[8] | ((uint16_t)buf[9] << 8));
    *h10 = (uint16_t)((uint16_t)buf[10] | ((uint16_t)buf[11] << 8));
    return 1;
}

/* 供 Init 统计用的位置前进（不碰 s_rp） */
static void dl_rp_next_pos(uint32_t *pSec, uint32_t *pOff)
{
    *pOff += DL_REC_SIZE;
    if (*pOff >= W25Q64_SECTOR_SIZE)
    {
        *pOff = 0;
        *pSec = (*pSec + 1) % DL_SECTORS;
    }
}

/* 槽位是否全 0xFF（未写过/已擦除）。读失败按"有数据"处理（宁可多擦不可漏写） */
static uint8_t dl_slot_erased(uint32_t sec, uint32_t off)
{
    uint8_t buf[DL_REC_SIZE];
    uint32_t i;
    if (W25Q64_Read(DL_BASE_ADDR + sec * W25Q64_SECTOR_SIZE + off, buf, DL_REC_SIZE) != W25Q64_OK)
        return 0;
    for (i = 0; i < DL_REC_SIZE; i++)
        if (buf[i] != 0xFF)
            return 0;
    return 1;
}

/* 书签读写（AT24C64 双槽接口，tag=SLOT_TAG_BOOKMARK，载荷=rpSec(4)+rpOff(4)）
 * - 只在 DL_Pop（补传成功）/丢最旧扇区时更新，DL_Append 不写 EEPROM（控写寿命）
 * - AT24C64 掉电保持，断电重启后书签仍在；写坏一槽由双槽另一槽兜底
 * - 模块不在位时读写返回 NAK，bookmark 视为无效 → 从头补传（服务端按 ts 去重）
 * - 写限流：dl_bookmark_write() 距上次真写 <DL_BM_THROTTLE_MS 只更新内存、不落盘；
 *   DL_BookmarkCommit() 无条件强制落盘（补传结束调用，把限流窗口内的进度补齐） */

/* 真正写 EEPROM：打包当前 rp 写入双槽，成功则刷新限流时刻 */
static void dl_bm_flush(void)
{
    uint8_t pay[8];
    pay[0] = (uint8_t)s_rpSec;
    pay[1] = (uint8_t)(s_rpSec >> 8);
    pay[2] = (uint8_t)(s_rpSec >> 16);
    pay[3] = (uint8_t)(s_rpSec >> 24);
    pay[4] = (uint8_t)s_rpOff;
    pay[5] = (uint8_t)(s_rpOff >> 8);
    pay[6] = (uint8_t)(s_rpOff >> 16);
    pay[7] = (uint8_t)(s_rpOff >> 24);
    if (AT24C64_SlotWrite(SLOT_TAG_BOOKMARK, pay, 8) == AT24C64_OK)
        s_bmLastWrite = GetTick();
}

/* 常规更新：限流。首写（s_bmLastWrite==0）或距上次 ≥30s 才真写，否则只更新内存 */
static void dl_bookmark_write(void)
{
    uint32_t now = GetTick();
    if (s_bmLastWrite != 0 && (now - s_bmLastWrite) < DL_BM_THROTTLE_MS)
        return;                       /* 30s 内：内存书签已是最新，掉电最多丢这段进度 */
    dl_bm_flush();
}

/* 补传结束强制提交：绕过限流无条件落盘一次（app.c 存储任务 drain 结束后调用） */
void DL_BookmarkCommit(void)
{
    dl_bm_flush();
}

/* 有效：能读到有效槽且 rp 落在合法范围；读出时同步填充 s_rpSec/s_rpOff */
static uint8_t dl_bookmark_valid(void)
{
    uint8_t pay[8];
    if (AT24C64_SlotRead(SLOT_TAG_BOOKMARK, pay, 8) != AT24C64_OK)
        return 0;
    s_rpSec = (uint32_t)pay[0] | ((uint32_t)pay[1] << 8) |
              ((uint32_t)pay[2] << 16) | ((uint32_t)pay[3] << 24);
    s_rpOff = (uint32_t)pay[4] | ((uint32_t)pay[5] << 8) |
              ((uint32_t)pay[6] << 16) | ((uint32_t)pay[7] << 24);
    return (s_rpSec < DL_SECTORS && s_rpOff < W25Q64_SECTOR_SIZE);
}

/* rp 前进一条（含跨扇区），不写书签 */
static void dl_rp_next(void)
{
    s_rpOff += DL_REC_SIZE;
    if (s_rpOff >= W25Q64_SECTOR_SIZE)
    {
        s_rpOff = 0;
        s_rpSec = (s_rpSec + 1) % DL_SECTORS;
    }
}

uint8_t DL_Init(void)
{
    uint32_t k, sec, off, posSec, posOff;
    uint8_t  found = 0;
    uint8_t  bmValid = 0;

    if (W25Q64_Init() != W25Q64_OK)
        return W25Q64_ERR_ID;
    s_ready = 0;

    /* 消费点：AT24C64 书签优先（dl_bookmark_valid 内部已填充 s_rpSec/s_rpOff），
     * 无效（首次上电/EEPROM 不在位）从 0 开始 */
    bmValid = dl_bookmark_valid();
    if (!bmValid)
    {
        s_rpSec = 0;
        s_rpOff = 0;
        dl_bookmark_write();
    }

    /* 找写点：从消费点起按环形扫，第一个全FF槽即写点；
     * 整圈找不到FF = 所有扇区都写满过 → 写点压回消费点扇区头，
     * 首次追加时按"丢最旧扇区"规则处理
     * ⚠️ 第一圈必须从 s_rpOff 起步（不能从 0）：跨会话擦除史可能在 rp 扇区内、
     * rp 之前留下 FF 槽，忽略 rpOff 会让 wp 落到 rp 环序后方，
     * 计数循环从 rp 绕整圈 → 幽灵近满环 count（实测 227183、清 22 秒） */
    found = 0;
    for (k = 0; k < DL_SECTORS && !found; k++)
    {
        sec = (s_rpSec + k) % DL_SECTORS;
        for (off = (k == 0) ? s_rpOff : 0; off < W25Q64_SECTOR_SIZE; off += DL_REC_SIZE)
        {
            if (dl_slot_erased(sec, off))
            {
                s_wpSec = sec;
                s_wpOff = off;
                found = 1;
                break;
            }
        }
    }
    if (!found)
    {
        s_wpSec = s_rpSec;
        s_wpOff = 0;
    }

    /* 统计消费点→写点之间的全部槽位（含历史坏记录——补传时会自动跳过并清理。
     * ⚠️ 不能只数"合法"记录：坏记录挡在好记录前面时，合法计数会提前耗尽，
     * 导致补传半途而废、好记录被搁浅（实测 87 条旧格式挡住 87 条新记录）） */
    s_count = 0;
    if (found)
    {
        posSec = s_rpSec;
        posOff = s_rpOff;
        while (posSec != s_wpSec || posOff != s_wpOff)
        {
            s_count++;
            dl_rp_next_pos(&posSec, &posOff);
        }
    }
    else
    {
        /* 整圈无空槽 = 缓存区写满：待补传条数是全部槽位。
         * 不能置0——首次追加按规则丢最旧扇区时 count 会下溢成 4×10^9（实测踩中） */
        s_count = DL_SECTORS * DL_REC_PER_SECTOR;
    }

    /* 开机诊断：书签有效性 + rp/wp/found/count，定位"wp 是否落在 rp 环序后方"的幽灵计数 */
    printf("datalog: init bm=%u rp=%u/%u wp=%u/%u found=%u count=%u%s\r\n",
           (unsigned)bmValid, (unsigned)s_rpSec, (unsigned)s_rpOff,
           (unsigned)s_wpSec, (unsigned)s_wpOff, (unsigned)found, (unsigned)s_count,
           found ? "" : " (ring-full fallback)");

    s_ready = 1;
    return W25Q64_OK;
}

uint8_t DL_Append(uint32_t ts, int16_t t10, uint16_t h10)
{
    static uint8_t diagDone = 0;       /* 失败路径诊断：整个上电周期只打一轮 */
    uint8_t buf[DL_REC_SIZE];

    if (!s_ready)
        return W25Q64_ERR_SPI;

    /* 写点落到扇区头：仅当扇区里有旧数据（未擦除）才走"丢最旧+擦除"流程。
     * ⚠️ 空扇区（rp==wp==扇区头）绝不能触发丢扇区，否则 count 下溢（实测
     * 首条缓存打出 4294967169 = 2^32-127） */
    if (s_wpOff == 0 && !dl_slot_erased(s_wpSec, 0))
    {
        if (s_rpSec == s_wpSec)     /* 消费点还在本扇区内：先丢本扇区最旧记录 */
        {
            uint32_t n = (W25Q64_SECTOR_SIZE - s_rpOff) / DL_REC_SIZE;
            if (s_count >= n)
            {
                s_count -= n;       /* 这n条确实在计数里：正常丢最旧扇区 */
                s_rpSec = (s_rpSec + 1) % DL_SECTORS;
                s_rpOff = 0;
                dl_bookmark_write();
            }
            else
            {
                /* 计数与Flash读数矛盾（记账缓存是空的，扇区头却"有数据"）：
                 * 大概率是拔线EMI毛刺把FF空槽误读成非空。自修复：既然没有
                 * 待传记录，读写点一起跳到下一扇区头重新对齐，绝不卡死重试 */
                if (!diagDone)
                {
                    diagDone = 1;
                    printf("[DL] count=%u<n=%u, sec%u head not erased -> realign\r\n",
                           (unsigned)s_count, (unsigned)n, (unsigned)s_wpSec);
                }
                s_rpSec = (s_rpSec + 1) % DL_SECTORS;
                s_wpSec = s_rpSec;
                s_rpOff = 0;
                s_wpOff = 0;
                dl_bookmark_write();
            }
        }
        if (W25Q64_EraseSector(DL_BASE_ADDR + s_wpSec * W25Q64_SECTOR_SIZE) != W25Q64_OK)
        {
            if (!diagDone)
            {
                diagDone = 1;
                printf("[DL] erase FAIL wp=%u/%u rp=%u/%u cnt=%u\r\n",
                       (unsigned)s_wpSec, (unsigned)s_wpOff,
                       (unsigned)s_rpSec, (unsigned)s_rpOff, (unsigned)s_count);
            }
            return W25Q64_ERR_SPI;
        }
    }

    dl_pack(buf, ts, t10, h10);
    if (W25Q64_Write(DL_BASE_ADDR + s_wpSec * W25Q64_SECTOR_SIZE + s_wpOff, buf, DL_REC_SIZE)
        != W25Q64_OK)
    {
        if (!diagDone)
        {
            diagDone = 1;
            printf("[DL] write FAIL @%06X wp=%u/%u rp=%u/%u cnt=%u\r\n",
                   (unsigned)(DL_BASE_ADDR + s_wpSec * W25Q64_SECTOR_SIZE + s_wpOff),
                   (unsigned)s_wpSec, (unsigned)s_wpOff,
                   (unsigned)s_rpSec, (unsigned)s_rpOff, (unsigned)s_count);
        }
        return W25Q64_ERR_SPI;
    }

    /* 排查探针（阶段9 复发调查，2026-10-07）：写入后立即回读校验。
     * 失败=写时损坏（供电/总线/擦除状态）；成功则数据当时在芯片上是好的 */
    {
        static uint8_t probeCnt = 0;
        if (probeCnt < 2)
        {
            uint8_t  rb[DL_REC_SIZE];
            uint32_t vts;
            int16_t  vt;
            uint16_t vh;
            uint32_t i;

            if (W25Q64_Read(DL_BASE_ADDR + s_wpSec * W25Q64_SECTOR_SIZE + s_wpOff,
                            rb, DL_REC_SIZE) != W25Q64_OK || !dl_unpack(rb, &vts, &vt, &vh))
            {
                uint8_t sr1 = 0;
                (void)W25Q64_ReadSR1(&sr1);
                probeCnt++;
                printf("[DL] append verify FAIL @%06X wp=%u/%u sr1=%02X\r\n  w:",
                       (unsigned)(DL_BASE_ADDR + s_wpSec * W25Q64_SECTOR_SIZE + s_wpOff),
                       (unsigned)s_wpSec, (unsigned)s_wpOff, sr1);
                for (i = 0; i < DL_REC_SIZE; i++) { printf(" %02X", buf[i]); }
                printf("\r\n  r:");
                for (i = 0; i < DL_REC_SIZE; i++) { printf(" %02X", rb[i]); }
                printf("\r\n");
            }
        }
    }

    s_wpOff += DL_REC_SIZE;
    if (s_wpOff >= W25Q64_SECTOR_SIZE)
    {
        s_wpOff = 0;
        s_wpSec = (s_wpSec + 1) % DL_SECTORS;
    }
    s_count++;
    return W25Q64_OK;
}

uint32_t DL_Count(void)
{
    return s_ready ? s_count : 0;
}

uint8_t DL_Peek(uint32_t *ts, int16_t *t10, uint16_t *h10)
{
    uint8_t buf[DL_REC_SIZE];

    if (!s_ready || s_count == 0)
        return W25Q64_ERR_PARAM;
    if (W25Q64_Read(DL_BASE_ADDR + s_rpSec * W25Q64_SECTOR_SIZE + s_rpOff,
                    buf, DL_REC_SIZE) != W25Q64_OK)
        return W25Q64_ERR_SPI;
    if (!dl_unpack(buf, ts, t10, h10))
    {
        /* 排查探针：Peek 校验失败时转储原始字节（上电前2次），配合 append verify
         * 区分"写时损坏"与"读时损坏"（阶段9 复发调查） */
        static uint8_t peekPrn = 0;
        if (peekPrn < 2)
        {
            uint32_t i;
            peekPrn++;
            printf("[DL] peek FAIL @%06X rp=%u/%u cnt=%u\r\n  r:",
                   (unsigned)(DL_BASE_ADDR + s_rpSec * W25Q64_SECTOR_SIZE + s_rpOff),
                   (unsigned)s_rpSec, (unsigned)s_rpOff, (unsigned)s_count);
            for (i = 0; i < DL_REC_SIZE; i++) { printf(" %02X", buf[i]); }
            printf("\r\n");
        }
        return W25Q64_ERR_SPI;              /* 记录损坏：调用方可 Pop 跳过 */
    }
    return W25Q64_OK;
}

uint8_t DL_Pop(void)
{
    if (!s_ready || s_count == 0)
        return W25Q64_ERR_PARAM;
    dl_rp_next();
    s_count--;
    dl_bookmark_write();
    return W25Q64_OK;
}
