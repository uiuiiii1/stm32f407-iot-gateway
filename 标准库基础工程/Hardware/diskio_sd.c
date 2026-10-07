#include "diskio.h"		/* FatFS 底层接口声明（FatFs/diskio.h） */
#include "ff.h"
#include "sdcard.h"
#include "rtc.h"

/*============================================================================
  FatFS ↔ SD 卡桥接层（diskio）
  - 物理驱动器只有 1 个：pdrv=0 = SD 卡（sdcard.c，软件 SPI 专用总线）
  - 调用约束：全部工作在 STG 任务上下文（sdcard 超时依赖 GetTick，
    且单写者访问是 ffconf.h 关闭锁/重入的前提）
  - get_fattime：RTC 有效时给真实时间，无效（未装电池又未对时）给固定值，
    绝不返回 0（FAT 对 0 时间戳行为未定义）
  ============================================================================*/

static volatile DSTATUS s_Stat = STA_NOINIT;   /* 驱动器状态（ STA_NOINIT=未初始化） */

/*-----------------------------------------------------------------------
/ 查状态
/-----------------------------------------------------------------------*/
DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv) { return STA_NOINIT; }
    return s_Stat;
}

/*-----------------------------------------------------------------------
/ 初始化（f_mount 时自动调用；卡拔插后 FatFS 重挂载也会再进来）
/-----------------------------------------------------------------------*/
DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv) { return STA_NOINIT; }

    if (SD_Init() == SD_OK)
    {
        s_Stat &= (DSTATUS)~STA_NOINIT;
    }
    else
    {
        s_Stat = STA_NOINIT;
    }
    return s_Stat;
}

/*-----------------------------------------------------------------------
/ 读扇区（count × 512B）
/-----------------------------------------------------------------------*/
DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) { return RES_PARERR; }
    if (s_Stat & STA_NOINIT) { return RES_NOTRDY; }

    return (SD_ReadSectors(sector, buff, count) == SD_OK) ? RES_OK : RES_ERROR;
}

/*-----------------------------------------------------------------------
/ 写扇区（count × 512B）
/-----------------------------------------------------------------------*/
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) { return RES_PARERR; }
    if (s_Stat & STA_NOINIT) { return RES_NOTRDY; }

    return (SD_WriteSectors(sector, buff, count) == SD_OK) ? RES_OK : RES_ERROR;
}

/*-----------------------------------------------------------------------
/ 杂项控制
/-----------------------------------------------------------------------*/
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv) { return RES_PARERR; }
    if (s_Stat & STA_NOINIT) { return RES_NOTRDY; }

    switch (cmd)
    {
        case CTRL_SYNC:          /* 写后同步：sdcard 写返回时卡内部已写完 */
            return RES_OK;

        case GET_SECTOR_COUNT:   /* f_mkfs/f_getfree 用 */
            *(DWORD *)buff = SD_GetSectorCount();
            return RES_OK;

        case GET_SECTOR_SIZE:    /* 仅 _MAX_SS>_MIN_SS 时才被调用，防御性实现 */
            *(WORD *)buff = 512;
            return RES_OK;

        case GET_BLOCK_SIZE:     /* 擦除块大小（SD 无擦除概念，最小值 1） */
            *(DWORD *)buff = 1;
            return RES_OK;

        default:
            return RES_PARERR;
    }
}

/*-----------------------------------------------------------------------
/ 文件时间戳（RTC 00-99 年，2000 起；FAT 纪元 1980 起）
/-----------------------------------------------------------------------*/
DWORD get_fattime(void)
{
    uint8_t y, mo, d, h, mi, s;

    if (RTC_IsValid() && RTC_GetDateTime(&y, &mo, &d, &h, &mi, &s) == 0)
    {
        return ((DWORD)(y + 20) << 25) | ((DWORD)mo << 21) | ((DWORD)d << 16) |
               ((DWORD)h << 11) | ((DWORD)mi << 5) | ((DWORD)s >> 1);
    }

    /* RTC 无效（未装电池又没对上时）：固定 2026-01-01 00:00:00，绝不返回 0 */
    return ((DWORD)(2026 - 1980) << 25) | ((DWORD)1 << 21) | ((DWORD)1 << 16);
}
