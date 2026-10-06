/* bootloader：升级状态机 + W25Q64→片内Flash 搬运（标准库版）
 * 移植自 F1 工程 07-bootloader程序（标准库）/Bootloader/App_bootloader.c
 * F4 主要差异：Flash 改扇区擦除 + 32位字编程（F1 是 1KB 页 + 半字编程） */
#include "App_bootloader.h"
#include "Int_bootloader.h"
#include "W25Q64.h"
#include "AT24C64.h"
#include "stm32f4xx_flash.h"
#include <stdio.h>

static uint8_t s_last_status = 0xFF;                     /* 防重复打印/重复动作 */
static uint8_t s_app_erased  = 0;                        /* 本次搬运是否已擦过应用区（决定失败后能否回退旧应用） */
uint8_t App_bootloader_update_status = BOOT_NO_UPDATE;   /* 默认不升级 */

static uint32_t s_app_run_addr = 0;                      /* 本次成功烧写的目标基址（0=没烧写） */
static uint8_t  s_copy_chunk[256];                       /* W25 一页一块的搬运缓冲 */

/* 把升级状态连同校验密钥写回 EEPROM（0x20: [状态][0x5A][0x6B]） */
static void App_bootloader_save_status(uint8_t status)
{
    uint8_t data[3];
    data[0] = status;
    data[1] = (uint8_t)(CHECK_KEY >> 8);
    data[2] = (uint8_t)(CHECK_KEY & 0xFF);
    AT24C64_Write(APP_UPDATE_ADDR, data, 3);
}

/* F407 应用区覆盖的扇区表：{扇区起始地址, 扇区号}。
 * ⚠️ 扇区0/1（0x08000000-0x08007FFF）是 bootloader 自己的，绝不能出现在擦除集合里 */
static const struct { uint32_t base; uint16_t sector; } s_app_sectors[] = {
    {0x08008000UL, FLASH_Sector_2 },
    {0x0800C000UL, FLASH_Sector_3 },
    {0x08010000UL, FLASH_Sector_4 },
    {0x08020000UL, FLASH_Sector_5 },
    {0x08040000UL, FLASH_Sector_6 },
    {0x08060000UL, FLASH_Sector_7 },
    {0x08080000UL, FLASH_Sector_8 },
    {0x080A0000UL, FLASH_Sector_9 },
    {0x080C0000UL, FLASH_Sector_10},
    {0x080E0000UL, FLASH_Sector_11},
};
#define APP_SECTOR_COUNT  (sizeof(s_app_sectors) / sizeof(s_app_sectors[0]))

/*==========================================================================
 * 擦除覆盖 [dst_base, dst_base+size) 的全部扇区（F4 粒度=扇区 16/64/128KB，
 * 比 F1 的 1KB 页粗——app 镜像之后的多余空间也会被一起擦掉，无害）。
 * 解锁/上锁收在内部。返回 1=全部成功，0=失败（已重新上锁）
 *==========================================================================*/
static uint8_t App_flash_erase_region(uint32_t dst_base, uint32_t size)
{
    uint32_t dst_end = dst_base + size;
    uint32_t i;

    FLASH_Unlock();
    for (i = 0; i < APP_SECTOR_COUNT; i++)
    {
        if (s_app_sectors[i].base >= dst_end)
            break;
        if (FLASH_EraseSector(s_app_sectors[i].sector, VoltageRange_3) != FLASH_COMPLETE)
        {
            FLASH_Lock();
            printf("erase fail sector %u\r\n", (unsigned)s_app_sectors[i].sector);
            return 0;
        }
    }
    FLASH_Lock();
    s_app_erased = 1;              /* 应用区已被擦除：此后失败不能回退旧应用 */
    return 1;
}

/*==========================================================================
 * 把 W25 的 size 字节搬到片内 Flash dst_base，同时累加源数据字节和到 *sum。
 * 每次读 256 字节（W25 一页）；F4 支持字编程（32位），尾部不足 4 字节补 0xFF。
 * 返回 1=整段写完，0=中途失败（已重新上锁，标志未清 → 复位后重做）
 *==========================================================================*/
static uint8_t App_flash_copy_from_w25(uint32_t w25_addr, uint32_t dst_base,
                                       uint32_t size, uint32_t *sum)
{
    uint32_t off = 0;

    FLASH_Unlock();
    while (off < size)
    {
        uint32_t remain = size - off;
        uint16_t n = (remain > sizeof(s_copy_chunk)) ? (uint16_t)sizeof(s_copy_chunk)
                                                     : (uint16_t)remain;
        uint16_t i;
        uint32_t addr = dst_base + off;

        if (W25Q64_Read(w25_addr + off, s_copy_chunk, n) != W25Q64_OK)
        {
            FLASH_Lock();
            printf("w25 read fail @%lu\r\n", (unsigned long)off);
            return 0;
        }
        for (i = 0; i < n; i++)
            *sum += s_copy_chunk[i];

        for (i = 0; i + 3 < n; i += 4)
        {
            uint32_t word = (uint32_t)s_copy_chunk[i]     |
                            ((uint32_t)s_copy_chunk[i + 1] <<  8) |
                            ((uint32_t)s_copy_chunk[i + 2] << 16) |
                            ((uint32_t)s_copy_chunk[i + 3] << 24);
            if (FLASH_ProgramWord(addr, word) != FLASH_COMPLETE)
            {
                FLASH_Lock();
                printf("program fail @0x%08lX\r\n", (unsigned long)addr);
                return 0;
            }
            addr += 4;
        }
        if (i < n)   /* 尾部不足 4 字节：实际字节保留、空位补 0xFF（擦除态） */
        {
            uint32_t k;
            uint32_t word = 0;
            for (k = 0; k < 4; k++)
            {
                uint8_t b = ((uint32_t)i + k < n) ? s_copy_chunk[i + k] : 0xFF;
                word |= (uint32_t)b << (8 * k);
            }
            if (FLASH_ProgramWord(addr, word) != FLASH_COMPLETE)
            {
                FLASH_Lock();
                printf("program fail @0x%08lX\r\n", (unsigned long)addr);
                return 0;
            }
        }
        off += n;
    }
    FLASH_Lock();
    return 1;
}

/*==========================================================================
 * A3 升级前备份：把片内当前应用备份到 W25 Slot B。
 * 旧应用大小：从应用区末尾倒扫找最后一个非 0xFF 字（未编程区是 0xFF）。
 * 返回 1=成功/无可备份；0=备份失败（调用方应放弃升级，旧版未被触碰）
 *==========================================================================*/
static uint8_t App_bootloader_backup_app(void)
{
    uint32_t size = 0;
    uint32_t addr;
    uint8_t  head[8];
    uint32_t off;

    for (addr = APP_END_ADDR; addr > APP_FLASH_BASE_ADDR; addr -= 4)
    {
        if (*(volatile uint32_t *)(addr - 4) != 0xFFFFFFFFUL)
        {
            size = addr - APP_FLASH_BASE_ADDR;
            break;
        }
    }
    if (size < APP_SIZE_MIN)          /* 无有效应用可保护 */
    {
        printf("backup skip (no app?)\r\n");
        return 1;
    }
    if (size > SLOT_B_MAX_SIZE)
    {
        printf("backup FAIL: size %lu > slot B\r\n", (unsigned long)size);
        return 0;
    }

    /* 擦 Slot B [头区, 正文区+size) 的扇区 */
    {
        uint32_t end = SLOT_B_IMG_ADDR + size;
        uint32_t a;
        for (a = SLOT_B_HEAD_ADDR; a < end; a += 0x1000UL)
        {
            if (W25Q64_EraseSector(a) != W25Q64_OK)
            {
                printf("backup erase fail @0x%lX\r\n", (unsigned long)a);
                return 0;
            }
        }
    }

    /* 头 8 字节：大端 [片内地址][大小] */
    head[0] = (uint8_t)(APP_FLASH_BASE_ADDR >> 24); head[1] = (uint8_t)(APP_FLASH_BASE_ADDR >> 16);
    head[2] = (uint8_t)(APP_FLASH_BASE_ADDR >> 8);  head[3] = (uint8_t)APP_FLASH_BASE_ADDR;
    head[4] = (uint8_t)(size >> 24); head[5] = (uint8_t)(size >> 16);
    head[6] = (uint8_t)(size >> 8);  head[7] = (uint8_t)size;
    if (W25Q64_Write(SLOT_B_HEAD_ADDR, head, 8) != W25Q64_OK)
    {
        printf("backup head write fail\r\n");
        return 0;
    }

    /* 搬运：片内读 → W25 写（256B/次） */
    for (off = 0; off < size; off += 256)
    {
        uint16_t n = (uint16_t)((size - off) > 256 ? 256 : (size - off));
        uint8_t  buf[256];
        uint16_t i;
        for (i = 0; i < n; i++)
            buf[i] = *(volatile uint8_t *)(APP_FLASH_BASE_ADDR + off + i);
        if (W25Q64_Write(SLOT_B_IMG_ADDR + off, buf, n) != W25Q64_OK)
        {
            printf("backup write fail @%lu\r\n", (unsigned long)off);
            return 0;
        }
    }
    printf("backup ok %lu bytes -> slot B\r\n", (unsigned long)size);
    return 1;
}

/*==========================================================================
 * A3 回滚：新固件多次未确认 → 从 Slot B 恢复旧应用回片内。
 * 返回 1=恢复并校验通过；0=失败（应用区可能已擦，调用方驻留 bootloader）
 *==========================================================================*/
static uint8_t App_bootloader_rollback_app(void)
{
    uint8_t  head[8];
    uint32_t size;
    uint32_t sum_src = 0, sum_dst = 0;
    uint32_t off;

    if (W25Q64_Read(SLOT_B_HEAD_ADDR, head, 8) != W25Q64_OK)
    {
        printf("rollback: head read fail\r\n");
        return 0;
    }
    if (head[0] != (uint8_t)(APP_FLASH_BASE_ADDR >> 24) ||
        head[1] != (uint8_t)(APP_FLASH_BASE_ADDR >> 16) ||
        head[2] != (uint8_t)(APP_FLASH_BASE_ADDR >> 8)  ||
        head[3] != (uint8_t)APP_FLASH_BASE_ADDR)
    {
        printf("rollback: head addr mismatch\r\n");
        return 0;
    }
    size = ((uint32_t)head[4] << 24) | ((uint32_t)head[5] << 16) |
           ((uint32_t)head[6] << 8)  | head[7];
    if (size < APP_SIZE_MIN || size > SLOT_B_MAX_SIZE)
    {
        printf("rollback: bad size %lu\r\n", (unsigned long)size);
        return 0;
    }

    /* 擦应用区 */
    s_app_erased = 1;
    if (App_flash_erase_region(APP_FLASH_BASE_ADDR, size) == 0)
        return 0;

    /* Slot B → 片内（字编程，尾部补 0xFF） */
    FLASH_Unlock();
    for (off = 0; off < size; off += 256)
    {
        uint16_t n = (uint16_t)((size - off) > 256 ? 256 : (size - off));
        uint8_t  buf[256];
        uint16_t i;
        if (W25Q64_Read(SLOT_B_IMG_ADDR + off, buf, n) != W25Q64_OK)
        {
            FLASH_Lock();
            printf("rollback: w25 read fail\r\n");
            return 0;
        }
        for (i = 0; i < n; i++)
            sum_src += buf[i];
        for (i = 0; i + 3 < n; i += 4)
        {
            uint32_t word = (uint32_t)buf[i] | ((uint32_t)buf[i+1] << 8) |
                            ((uint32_t)buf[i+2] << 16) | ((uint32_t)buf[i+3] << 24);
            if (FLASH_ProgramWord(APP_FLASH_BASE_ADDR + off + i, word) != FLASH_COMPLETE)
            {
                FLASH_Lock();
                printf("rollback: prog fail\r\n");
                return 0;
            }
        }
        if (i < n)
        {
            uint32_t word = 0, k;
            for (k = 0; k < 4; k++)
            {
                uint8_t b = (i + k < n) ? buf[i + k] : 0xFF;
                word |= (uint32_t)b << (8 * k);
            }
            if (FLASH_ProgramWord(APP_FLASH_BASE_ADDR + off + i, word) != FLASH_COMPLETE)
            {
                FLASH_Lock();
                printf("rollback: prog fail\r\n");
                return 0;
            }
        }
    }
    FLASH_Lock();

    /* 读回字节和比对 */
    for (off = 0; off < size; off++)
        sum_dst += *(volatile uint8_t *)(APP_FLASH_BASE_ADDR + off);
    if (sum_dst != sum_src)
    {
        printf("rollback: verify sum err\r\n");
        return 0;
    }

    printf("rollback ok -> 0x%08lX\r\n", (unsigned long)APP_FLASH_BASE_ADDR);
    return 1;
}

/*==========================================================================
 * 把 W25 里的固件镜像搬进片内 Flash 应用区
 * 流程：读元数据 → 大小/范围校验 → 读镜像向量表头校验 → 擦应用区 →
 *       搬运（字节和累加）→ 整段读回比对
 * 返回 1=写入并读回校验通过；0=任一步失败（标志未清，复位后重做）
 *==========================================================================*/
static uint8_t App_bootloader_write_app_flash(void)
{
    uint8_t  buff[8];
    uint32_t w25_addr = 0;
    uint32_t app_size = 0;
    uint32_t msp, reset, vec_addr;
    uint32_t sum_src, sum_dst, k;

    /* 1，读元数据（大端） */
    if (W25Q64_Read(META_APP_ADDR, buff, META_APP_SIZE) != W25Q64_OK)
    {
        printf("read meta error\r\n");
        return 0;
    }
    w25_addr = ((uint32_t)buff[0] << 24) | ((uint32_t)buff[1] << 16) |
               ((uint32_t)buff[2] <<  8) | (uint32_t)buff[3];
    app_size = ((uint32_t)buff[4] << 24) | ((uint32_t)buff[5] << 16) |
               ((uint32_t)buff[6] <<  8) | (uint32_t)buff[7];
    printf("meta: w25_addr=0x%08lX size=%lu\r\n",
           (unsigned long)w25_addr, (unsigned long)app_size);

    /* 2，范围校验 */
    if (app_size < APP_SIZE_MIN || app_size > APP_SIZE_MAX)
    {
        printf("app size out of range\r\n");
        return 0;
    }
    if (w25_addr < APP_ADDR_MIN || w25_addr + app_size > W25_FIRMWARE_END)
    {
        printf("w25 addr out of range\r\n");
        return 0;
    }

    /* 3，读镜像向量表头 8 字节（小端：前4=栈顶，后4=复位入口） */
    if (W25Q64_Read(w25_addr, buff, 8) != W25Q64_OK)
    {
        printf("read app vector fail\r\n");
        return 0;
    }
    msp   = (uint32_t)buff[0]         | ((uint32_t)buff[1] <<  8) |
            ((uint32_t)buff[2] << 16) | ((uint32_t)buff[3] << 24);
    reset = (uint32_t)buff[4]         | ((uint32_t)buff[5] <<  8) |
            ((uint32_t)buff[6] << 16) | ((uint32_t)buff[7] << 24);

    if ((msp & 0xFFFF0000UL) != 0x20000000UL)
    {
        printf("stack addr error\r\n");
        return 0;
    }

    /* 4，复位入口必须落在应用区内（防镜像/元数据错位烧错地方） */
    vec_addr = reset & 0xFFFFFFFEUL;
    if (vec_addr < APP_FLASH_BASE_ADDR || vec_addr >= APP_END_ADDR)
    {
        printf("reset address error 0x%08lX\r\n", (unsigned long)vec_addr);
        return 0;
    }
    if (app_size > (APP_END_ADDR - APP_FLASH_BASE_ADDR))
    {
        printf("app size > app region\r\n");
        return 0;
    }

    /* 4.5 升级前备份当前应用到 Slot B（A3 回滚用）；备份失败 = 放弃升级（旧版未被触碰） */
    if (!App_bootloader_backup_app())
        return 0;

    /* 5，擦应用区 */
    s_app_erased = 0;                                  /* 尚未擦，失败可安全回退旧应用 */
    if (App_flash_erase_region(APP_FLASH_BASE_ADDR, app_size) == 0)
        return 0;

    /* 6，搬运 + 源字节和 */
    sum_src = 0;
    if (App_flash_copy_from_w25(w25_addr, APP_FLASH_BASE_ADDR, app_size, &sum_src) == 0)
        return 0;

    /* 7，读回校验：目标区整段求和与源比对 */
    sum_dst = 0;
    for (k = 0; k < app_size; k++)
    {
        sum_dst += *(volatile uint8_t *)(APP_FLASH_BASE_ADDR + k);
    }
    if (sum_dst != sum_src)
    {
        printf("verify sum err %lu/%lu\r\n", (unsigned long)sum_dst, (unsigned long)sum_src);
        return 0;
    }

    s_app_run_addr = APP_FLASH_BASE_ADDR;
    printf("copy ok -> 0x%08lX\r\n", (unsigned long)APP_FLASH_BASE_ADDR);
    return 1;
}

/* 上电检查 EEPROM 升级状态：密钥对且状态合法才采纳，否则写回修复为"不升级" */
void App_bootloader_check_update(void)
{
    uint8_t  data[3];
    uint16_t check_key;

    printf("bootloader start\r\n");
    printf("check update\r\n");
    AT24C64_Read(APP_UPDATE_ADDR, data, 3);
    check_key = ((uint16_t)data[1] << 8) | data[2];
    printf("ee:%02X %02X %02X\r\n", data[0], data[1], data[2]);   /* 状态字节上电可见 */
    if (check_key != CHECK_KEY)
    {
        App_bootloader_save_status(BOOT_NO_UPDATE);
    }
    else if (data[0] == BOOT_UPDATE || data[0] == BOOT_NO_UPDATE || data[0] == BOOT_RESET)
    {
        App_bootloader_update_status = data[0];
    }
    else
    {
        /* 密钥对但状态字节未知：兜底按"不升级"写回，避免 process 空转不跳转 */
        App_bootloader_save_status(BOOT_NO_UPDATE);
    }
}

/* 待确认计数（A2/A3）：烧写新固件后，每次 NO_UPDATE 启动计数 +1。
 * 应用 MQTT 上线正常会把 0x23 清 0 → 停止计数。
 * 达阈值（≥BL_CONFIRM_BOOTS）→ 从 Slot B 回滚旧应用（A3）。
 * 返回：1=可正常跳转；0=回滚失败，驻留 bootloader */
static uint8_t App_bootloader_pending_check(void)
{
    uint8_t pend = 0, cnt = 0;
    if (AT24C64_Read(BL_PENDING_ADDR, &pend, 1) != AT24C64_OK)
        return 1;
    if (pend != 0x01)
        return 1;
    AT24C64_Read(BL_COUNT_ADDR, &cnt, 1);
    cnt++;
    AT24C64_Write(BL_COUNT_ADDR, &cnt, 1);
    if (cnt < BL_CONFIRM_BOOTS)
    {
        printf("boots pending: %u/%u (waiting firmware confirm)\r\n", cnt, BL_CONFIRM_BOOTS);
        return 1;
    }

    /* 达阈值：新固件未确认 → 自动回滚旧版（A3） */
    printf("boots pending: %u/%u, new firmware NOT confirmed -> rollback\r\n",
           cnt, BL_CONFIRM_BOOTS);
    if (App_bootloader_rollback_app())
    {
        uint8_t v = 0x00;
        AT24C64_Write(BL_PENDING_ADDR, &v, 1);   /* 清待确认锁存 */
        AT24C64_Write(BL_COUNT_ADDR, &v, 1);
        return 1;                                 /* 跳回旧版 */
    }
    printf("WARN: rollback FAILED, stay in bootloader, reflash manually\r\n");
    return 0;
}

/* 按状态分发（状态变化才动作一次） */
void App_bootloader_process(void)
{
    if (App_bootloader_update_status == s_last_status)
        return;
    s_last_status = App_bootloader_update_status;

    if (s_last_status == BOOT_UPDATE)
    {
        printf("update\r\n");
        if (App_bootloader_write_app_flash())
        {
            /* 只有"写完 + 读回校验通过"才清标志；失败保持 BOOT_UPDATE 复位后重做 */
            App_bootloader_save_status(BOOT_NO_UPDATE);
            App_bootloader_update_status = BOOT_NO_UPDATE;
            /* 置待确认：0x23=0x01、计数清 0；等新固件 MQTT 上线确认后清 0x23 */
            {
                uint8_t v = 0x01;
                AT24C64_Write(BL_PENDING_ADDR, &v, 1);
                v = 0;
                AT24C64_Write(BL_COUNT_ADDR, &v, 1);
                printf("new firmware pending confirm\r\n");
            }
            App_bootloader_jump_app();
        }
        else if (!s_app_erased)
        {
            /* 尚未擦应用区（校验/元数据/向量表阶段就失败，如"假固件"测试）：
             * 清标志回退到旧应用，避免卡在 bootloader */
            printf("update fail, fall back to old app\r\n");
            App_bootloader_save_status(BOOT_NO_UPDATE);
            App_bootloader_update_status = BOOT_NO_UPDATE;
            App_bootloader_jump_app();
        }
        else
        {
            printf("update fail (app partly erased), flag kept\r\n");
            /* 已发生过擦除：旧应用已不可信，保持 UPDATE 复位后重做 */
        }
    }
    else   /* NO_UPDATE / BOOT_RESET：都直接跳应用 */
    {
        printf("no update\r\n");
        if (App_bootloader_pending_check())       /* A2/A3：计数→达阈值回滚 */
            App_bootloader_jump_app();
        /* 否则回滚失败：驻留 bootloader 等待人工处理 */
    }
}

void App_bootloader_jump_app(void)
{
    uint32_t target = (s_app_run_addr != 0) ? s_app_run_addr : APP_FLASH_BASE_ADDR;

    printf("jump app 0x%08lX\r\n", (unsigned long)target);
    bootloader_jump_to_app(target);
}
