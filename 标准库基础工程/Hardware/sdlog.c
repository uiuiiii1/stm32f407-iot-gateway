#include "sdlog.h"
#include "ff.h"
#include "rtc.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>

/*============================================================================
  SD 卡按天 CSV 归档实现（设计约束见 sdlog.h 顶部）
  - 日期来源：RTC_GetDateTime（本地时间，与 SNTP 同步后的 RTC 一致）；
    ts 值直接采信 COL 打的 Unix 时间戳，不重复换算
  - 写频率 0.5Hz、每行 ~28B → 日文件 ~1.2MB，SD 卡磨损可忽略
  ============================================================================*/

#define SDLOG_SYNC_EVERY   10
#define SDLOG_RETRY_MS     60000UL
#define SDLOG_NAME_MAX     20           /* "0:20261007.CSV" 足够 */

static FATFS    s_fs;
static FIL      s_file;
static uint8_t  s_ready = 0;          /* 0=禁用（未挂载/写失败） 1=就绪 */
static uint8_t  s_y = 0xFF, s_mo = 0xFF, s_d = 0xFF;   /* 当前文件日期（0xFF=未定/NODATE） */
static uint32_t s_count = 0;          /* 当前文件已写条数 */
static uint32_t s_sinceSync = 0;
static uint32_t s_lastTry = 0;        /* 上次重探测时刻（禁用态限流） */
static uint32_t s_lastStat = 0;       /* 上次统计打印时刻 */

/* 打开"日期对应"的当日文件（文件常开、指针在末尾追加）。
 * RTC 无效 → NODATE.CSV；RTC 恢复可信后日期变化会走换文件路径 */
static void SDLOG_OpenDayFile(void)
{
    char     name[SDLOG_NAME_MAX];
    uint8_t  y, mo, d, h, mi, s;

    if (RTC_IsValid() && RTC_GetDateTime(&y, &mo, &d, &h, &mi, &s) == 0)
    {
        s_y = y; s_mo = mo; s_d = d;
        sprintf(name, "0:20%02d%02d%02d.CSV", y, mo, d);
    }
    else
    {
        s_y = 0xFF; s_mo = 0xFF; s_d = 0xFF;
        strcpy(name, "0:NODATE.CSV");
    }

    if (f_open(&s_file, name, FA_WRITE | FA_OPEN_APPEND) != FR_OK)
    {
        printf("[SD] open %s fail\r\n", name);
        s_ready = 0;
        return;
    }
    s_count = 0;
    s_sinceSync = 0;
    printf("[SD] log ready: %s\r\n", name);
}

void SDLOG_Init(void)
{
    s_ready = 0;
    s_lastTry = GetTick();

    /* 介质预检已由上层确认过（见踩坑#41：R0.12c 会把非 FAT 卷误挂载），
     * 这里再拦一道：任何挂载失败都只降级，不阻塞 STG 主流程 */
    if (f_mount(&s_fs, "0:", 1) != FR_OK)
    {
        printf("[SD] log disabled: mount fail (no card?)\r\n");
        return;
    }
    SDLOG_OpenDayFile();
    s_ready = 1;
}

void SDLOG_Log(uint32_t ts, int16_t t10, uint16_t h10)
{
    FRESULT fr;
    UINT    bw;
    int     len;
    char    line[40];
    uint8_t y, mo, d, h, mi, s;

    if (!s_ready)
    {
        /* 禁用态限流重探测（拔插卡自恢复） */
        if ((GetTick() - s_lastTry) < SDLOG_RETRY_MS) { return; }
        s_lastTry = GetTick();
        SDLOG_Init();
        if (!s_ready) { return; }
    }

    /* 跨天检测：RTC 可信且日期变了 → 关旧文件开新文件（首次 RTC 从无效转
     * 有效也在此路径换出 NODATE.CSV） */
    if (RTC_IsValid() && RTC_GetDateTime(&y, &mo, &d, &h, &mi, &s) == 0 &&
        (y != s_y || mo != s_mo || d != s_d))
    {
        uint32_t prev = s_count;          /* OpenDayFile 会清 s_count，先存 */
        if (f_close(&s_file) != FR_OK)
        {
            printf("[SD] close on day-roll fail\r\n");
            s_ready = 0;
            return;
        }
        SDLOG_OpenDayFile();
        if (!s_ready) { return; }
        printf("[SD] day rollover, %u records in previous file\r\n", (unsigned)prev);
    }

    len = sprintf(line, "%lu,%d,%u\r\n", (unsigned long)ts, (int)t10, (unsigned)h10);
    fr = f_write(&s_file, line, (UINT)len, &bw);
    if (fr != FR_OK || bw != (UINT)len)
    {
        printf("[SD] write fail (fr=%d bw=%u) -> disabled\r\n", fr, (unsigned)bw);
        (void)f_close(&s_file);
        s_ready = 0;
        return;
    }
    s_count++;
    s_sinceSync++;

    /* 每 60s 一行归档统计（写入进度可见；日志克制原则——只在整分位打一行） */
    if ((GetTick() - s_lastStat) >= 60000UL)
    {
        s_lastStat = GetTick();
        printf("[SD] recs=%u total\r\n", (unsigned)s_count);
    }

    if (s_sinceSync >= SDLOG_SYNC_EVERY)
    {
        s_sinceSync = 0;
        if (f_sync(&s_file) != FR_OK)
        {
            printf("[SD] sync fail -> disabled\r\n");
            (void)f_close(&s_file);
            s_ready = 0;
        }
    }
}

uint32_t SDLOG_GetCount(void)
{
    return s_count;
}
