#include "ota.h"
#include "w25q64.h"
#include "AT24C64.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>

/* ===== 下载状态机 ===== */

typedef enum { ST_IDLE = 0, ST_DOWNLOADING } OtaState;

static OtaState s_state = ST_IDLE;
static uint32_t s_size;                 /* 目标固件总字节数 */
static uint32_t s_crcExpect;            /* ota_begin 声明的 CRC32 */
static uint32_t s_crc;                  /* 运行累计 CRC */
static uint32_t s_received;             /* 已收字节数 */
static uint32_t s_deadline;             /* 超时时刻 */

/* ---------------- CRC32（IEEE 802.3，反射多项式 0xEDB88320） ---------------- */
static void crc32_update(uint32_t *crc, const uint8_t *p, uint16_t len)
{
    while (len--)
    {
        uint32_t c = (*crc ^ *p++) & 0xFF;
        uint8_t  k;
        for (k = 0; k < 8; k++)
            c = (c & 1) ? (c >> 1) ^ 0xEDB88320UL : (c >> 1);
        *crc = c ^ (*crc >> 8);
    }
}

/* ---------------- JSON 极简解析（均带长度边界，不依赖结尾 '\0'） ---------------- */
static const uint8_t *js_find(const uint8_t *p, uint16_t len, const char *key)
{
    uint16_t kl = (uint16_t)strlen(key);
    uint16_t i;
    for (i = 0; i + kl + 1 < len; i++)
    {
        if (p[i] == '"' && strncmp((const char *)(p + i + 1), key, kl) == 0
            && p[i + kl + 1] == '"')
        {
            i += kl + 1;                          /* 越过右引号 */
            while (i < len && p[i] != ':') i++;
            if (i < len) i++;                     /* 越过冒号 */
            while (i < len && (p[i] == ' ' || p[i] == '\t')) i++;
            if (i < len && p[i] == '"') i++;      /* 字符串值跳过前导引号：值从内容开始 */
            return p + i;
        }
    }
    return 0;
}

static uint32_t js_num(const uint8_t *v, uint16_t rem)
{
    uint32_t x = 0;
    while (rem && v[0] >= '0' && v[0] <= '9') { x = x * 10 + (uint32_t)(v[0] - '0'); v++; rem--; }
    return x;
}

static uint32_t js_hex32(const uint8_t *v, uint16_t rem)
{
    uint32_t x = 0;
    uint8_t  n = 0;
    if (rem >= 2 && v[0] == '0' && (v[1] == 'x' || v[1] == 'X')) { v += 2; rem -= 2; }
    while (n < 8 && rem)
    {
        uint8_t c = *v++;
        uint32_t d;
        if (c >= '0' && c <= '9')      d = (uint32_t)(c - '0');
        else if (c >= 'a' && c <= 'f') d = (uint32_t)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') d = (uint32_t)(c - 'A' + 10);
        else break;
        x = (x << 4) | d;
        n++; rem--;
    }
    return x;
}

/* hex 串转字节，返回字节数（遇到非 hex 或顶格停止） */
static uint16_t hex_decode(const uint8_t *v, uint16_t rem, uint8_t *out, uint16_t maxOut)
{
    uint16_t n = 0;
    while (rem >= 2 && n < maxOut)
    {
        uint8_t hi, lo;
        uint8_t c1 = v[0], c2 = v[1];
        if (c1 >= '0' && c1 <= '9')      hi = (uint8_t)(c1 - '0');
        else if (c1 >= 'a' && c1 <= 'f') hi = (uint8_t)(c1 - 'a' + 10);
        else if (c1 >= 'A' && c1 <= 'F') hi = (uint8_t)(c1 - 'A' + 10);
        else break;
        if (c2 >= '0' && c2 <= '9')      lo = (uint8_t)(c2 - '0');
        else if (c2 >= 'a' && c2 <= 'f') lo = (uint8_t)(c2 - 'a' + 10);
        else if (c2 >= 'A' && c2 <= 'F') lo = (uint8_t)(c2 - 'A' + 10);
        else break;
        out[n++] = (uint8_t)((hi << 4) | lo);
        v += 2; rem -= 2;
    }
    return n;
}

/* ---------------- 对外接口 ---------------- */

uint8_t OTA_Begin(uint32_t size, uint32_t crc)
{
    uint32_t end = OTA_IMG_BASE + size;
    uint32_t a;
    uint8_t  meta[8];

    if (s_state != ST_IDLE)
    {
        printf("OTA: busy, ignore begin\r\n");
        return OTA_ERR_BUSY;
    }
    if (size < OTA_SIZE_MIN || size > OTA_SIZE_MAX || end > OTA_SLOT_END)
    {
        printf("OTA: bad size %lu\r\n", (unsigned long)size);
        return OTA_ERR_BAD;
    }

    /* 擦除 [0x0000, end) 覆盖的全部 4KB 扇区（含元数据区 0x00） */
    for (a = 0; a < end; a += 0x1000UL)
    {
        if (W25Q64_EraseSector(a) != W25Q64_OK)
        {
            printf("OTA: erase fail @0x%lX\r\n", (unsigned long)a);
            return OTA_ERR_FLASH;
        }
    }

    /* 元数据 8 字节大端 [镜像地址][长度]，与 bootloader 约定一致 */
    meta[0] = (uint8_t)(OTA_IMG_BASE >> 24); meta[1] = (uint8_t)(OTA_IMG_BASE >> 16);
    meta[2] = (uint8_t)(OTA_IMG_BASE >> 8);  meta[3] = (uint8_t)OTA_IMG_BASE;
    meta[4] = (uint8_t)(size >> 24);         meta[5] = (uint8_t)(size >> 16);
    meta[6] = (uint8_t)(size >> 8);          meta[7] = (uint8_t)size;
    if (W25Q64_Write(OTA_META_ADDR, meta, 8) != W25Q64_OK)
    {
        printf("OTA: meta write fail\r\n");
        return OTA_ERR_FLASH;
    }

    s_size = size;
    s_crcExpect = crc;
    s_crc = 0xFFFFFFFFUL;
    s_received = 0;
    s_state = ST_DOWNLOADING;
    s_deadline = GetTick() + OTA_TIMEOUT_MS;
    printf("OTA: begin size=%lu crc=0x%08lX chunks=%lu\r\n",
           (unsigned long)size, (unsigned long)crc,
           (unsigned long)((size + OTA_CHUNK_BYTES - 1) / OTA_CHUNK_BYTES));
    return OTA_OK;
}

uint8_t OTA_Chunk(uint8_t seq, const uint8_t *raw, uint16_t len)
{
    uint32_t off;

    if (s_state != ST_DOWNLOADING)
        return OTA_ERR_IDLE;
    if (len > OTA_CHUNK_BYTES)
        return OTA_ERR_LEN;

    off = (uint32_t)seq * OTA_CHUNK_BYTES;
    if (off != s_received)
    {
        /* 断线丢块导致顺序断裂：立即中止而非静默等90s超时，方便调用方重发 */
        printf("OTA: seq %u out of order (expect %lu), abort, re-publish\r\n",
               (unsigned)seq, (unsigned long)(s_received / OTA_CHUNK_BYTES));
        OTA_Abort();
        return OTA_ERR_SEQ;
    }
    if (s_received + len > s_size)
    {
        printf("OTA: chunk overruns size\r\n");
        return OTA_ERR_LEN;
    }

    if (W25Q64_Write(OTA_IMG_BASE + s_received, raw, len) != W25Q64_OK)
    {
        printf("OTA: flash write fail\r\n");
        OTA_Abort();
        return OTA_ERR_FLASH;
    }
    crc32_update(&s_crc, raw, len);
    s_received += len;
    s_deadline = GetTick() + OTA_TIMEOUT_MS;

    if (s_received >= s_size)
    {
        uint32_t final = ~s_crc;
        if (final != s_crcExpect)
        {
            printf("OTA: CRC fail 0x%08lX != expect 0x%08lX\r\n",
                   (unsigned long)final, (unsigned long)s_crcExpect);
            OTA_Abort();
            return OTA_ERR_CRC;
        }
        printf("OTA: verify OK (%lu bytes), write upgrade flag\r\n", (unsigned long)s_size);
        {   /* AT24C64 0x20 = [0x01=UPDATE][0x5A][0x6B]，bootloader 检测后烧写 */
            uint8_t f[3] = { 0x01, 0x5A, 0x6B };
            AT24C64_Write(0x20, f, 3);
        }
        s_state = ST_IDLE;
        printf("OTA: firmware saved, resetting now...\r\n");
        NVIC_SystemReset();           /* 复位后由 bootloader 烧写进应用区 */
    }
    return OTA_OK;
}

/**
 * @brief OTA轮询函数，在主循环里周期性调用
 * 作用：检测OTA下载是否超时，防止网络断连后卡死在下载状态
 */
void OTA_Poll(void)
{
    // 判断当前状态：只有正在下载固件的时候，才做超时检测
    if (s_state == ST_DOWNLOADING)
    {
        // 已到/超过 deadline 即超时：带符号差比较，抗 tick 回绕
        // （此前误用 (uint32)(now-deadline)>T：now-deadline 为负→无符号巨大→立刻误判超时）
        if ((int32_t)(GetTick() - s_deadline) >= 0)
        {
            printf("OTA: timeout, abort\r\n");
            OTA_Abort(); // 超时，终止本次OTA下载，状态切回空闲
        }
    }
}


uint8_t OTA_IsBusy(void)
{
    return (s_state != ST_IDLE);
}

void OTA_Abort(void)
{
    if (s_state != ST_IDLE)
        printf("OTA: aborted (%lu/%lu bytes)\r\n",
               (unsigned long)s_received, (unsigned long)s_size);
    s_state = ST_IDLE;
}

/* ---------------- 固件运行确认（10.4） ----------------
 * AT24C64 0x30：确认标记 [0x5A][0x6B][版本码4字节大端]。
 * 版本码 != 当前固件版本 → 判为"新固件首次运行"，需发一次确认；
 * 相同 → 已确认过，跳过（防每次上线重复发）。 */
#define CFM_ADDR    0x30

uint8_t OTA_NeedConfirm(void)
{
    uint8_t b[6];
    if (AT24C64_Read(CFM_ADDR, b, 6) != AT24C64_OK)
        return 1;                                   /* 读失败按需确认处理 */
    if (b[0] == 0x5A && b[1] == 0x6B &&
        b[2] == (uint8_t)(OTA_VER_CODE >> 24) &&
        b[3] == (uint8_t)(OTA_VER_CODE >> 16) &&
        b[4] == (uint8_t)(OTA_VER_CODE >> 8) &&
        b[5] == (uint8_t)OTA_VER_CODE)
        return 0;
    return 1;
}

void OTA_ConfirmMark(void)
{
    uint8_t b[6];
    b[0] = 0x5A; b[1] = 0x6B;
    b[2] = (uint8_t)(OTA_VER_CODE >> 24);
    b[3] = (uint8_t)(OTA_VER_CODE >> 16);
    b[4] = (uint8_t)(OTA_VER_CODE >> 8);
    b[5] = (uint8_t)OTA_VER_CODE;
    AT24C64_Write(CFM_ADDR, b, 6);
}

/* 清空确认标记：写 6 字节 0xFF（非法）→ OTA_NeedConfirm 下次返回 1 */
void OTA_ConfirmClear(void)
{
    uint8_t b[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    if (AT24C64_Write(CFM_ADDR, b, 6) == AT24C64_OK)
        printf("OTA: confirm marker cleared (will reconfirm on next online)\r\n");
    else
        printf("OTA: confirm clear write fail\r\n");
}

/* 直接把指定版本号写进确认标记（随意调整 AT24 里存的版本） */
void OTA_ConfirmSet(uint32_t ver)
{
    uint8_t b[6];
    b[0] = 0x5A; b[1] = 0x6B;
    b[2] = (uint8_t)(ver >> 24);
    b[3] = (uint8_t)(ver >> 16);
    b[4] = (uint8_t)(ver >> 8);
    b[5] = (uint8_t)ver;
    if (AT24C64_Write(CFM_ADDR, b, 6) == AT24C64_OK)
        printf("OTA: confirm marker set ver=0x%08lX\r\n", (unsigned long)ver);
    else
        printf("OTA: confirm set write fail\r\n");
}

/* 固件运行正常：清 bootloader "待确认"锁存（0x23=0x00），停止启动计数 */
void OTA_NotifyAlive(void)
{
    uint8_t pend = 0xFF;
    if (AT24C64_Read(0x23, &pend, 1) != AT24C64_OK)
        return;                       /* 读失败当无事（幂等，不影响） */
    if (pend != 0x01)
        return;                       /* 本没有待确认：不刷屏 */
    {
        uint8_t v = 0x00;
        if (AT24C64_Write(0x23, &v, 1) == AT24C64_OK)
            printf("OTA: alive, pending latch cleared\r\n");
    }
}

/* ---------------- MQTT 消息分发 ---------------- */
// MQTT回调入口：收到云端下发的MQTT消息自动执行
// topic：消息主题；topicLen主题长度；payload：消息正文(JSON)；len正文长度
void OTA_OnMqtt(const uint8_t *topic, uint16_t topicLen,
                const uint8_t *payload, uint16_t len)
{
    // 定义两个OTA的MQTT主题字符串常量
    static const uint8_t tkCmd[] = OTA_TOPIC_CMD;
    static const uint8_t tkFw[]  = OTA_TOPIC_FW;

    // 判断：收到的主题 == OTA命令主题（ota_begin指令）
    if (topicLen == sizeof(tkCmd) - 1 &&
        memcmp(topic, tkCmd, topicLen) == 0)
    {
        /* cmd：ota_begin 升级开始指令 */
        // 在JSON里面查找key="cmd"
        const uint8_t *c = js_find(payload, len, "cmd");
        // 判断找到了cmd，并且内容是"ota_begin"
        if (c && (size_t)(len - (uint16_t)(uintptr_t)(c - payload)) >= 9 &&
            strncmp((const char *)c, "ota_begin", 9) == 0)
        {
            // 取出JSON里面 size（完整固件总字节大小）、crc（固件整体校验值）
            const uint8_t *ss = js_find(payload, len, "size");
            const uint8_t *cc = js_find(payload, len, "crc");
            if (ss && cc)
            {
                // 把字符串转成数字：固件总大小、固件CRC32
                uint32_t size = js_num(ss, (uint16_t)(len - (uint16_t)(uintptr_t)(ss - payload)));
                uint32_t crc  = js_hex32(cc, (uint16_t)(len - (uint16_t)(uintptr_t)(cc - payload)));
                // 调用OTA_Begin，准备开始下载：擦写准备，记录固件总长度、最终校验码
                OTA_Begin(size, crc);
            }
        }
    }
    // 判断：收到的主题 == 固件分片主题（下发固件数据包）
    else if (topicLen == sizeof(tkFw) - 1 &&
             memcmp(topic, tkFw, topicLen) == 0)
    {
        /* ota：{seq,data-hex} 分块，下发固件分片 */
        // 在JSON查找key：seq分片编号、data十六进制固件数据
        const uint8_t *sq = js_find(payload, len, "seq");
        const uint8_t *dd = js_find(payload, len, "data");
        // 临时缓冲区：存放解码后的二进制固件
        static uint8_t raw[OTA_CHUNK_BYTES];
        uint16_t nb = 0;
        if (sq && dd)
        {
            // 解析分片序号seq
            uint32_t seq = js_num(sq, (uint16_t)(len - (uint16_t)(uintptr_t)(sq - payload)));
            uint16_t rem = (uint16_t)(len - (uint16_t)(uintptr_t)(dd - payload));
            // hex_decode：把JSON里的十六进制字符串，还原成二进制固件字节
            nb = hex_decode(dd, rem, raw, OTA_CHUNK_BYTES);
            // 把这一块二进制固件交给OTA_Chunk，写入W25Q64备份固件区
            OTA_Chunk((uint8_t)seq, raw, nb);
        }
    }
    /* 其它主题忽略，不做任何处理 */
}
