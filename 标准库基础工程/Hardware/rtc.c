#include "rtc.h"

/*
 * F407 内部 RTC 驱动（LSE 32.768kHz 板载晶振）
 * 未装CR1220电池前，断电后时间丢失（回到默认值），装电池后自动掉电走时
 */

void RTC_Init_Wrap(void)
{
    RTC_InitTypeDef rtc;
    uint32_t t = 200000;    /* LSE启动超时守卫：晶振坏/未焊时不死机，继续跑（时间可能不准） */

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);            /* 允许访问备份域（RTC/LSE） */

    RCC_LSEConfig(RCC_LSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
        if ((t--) == 0)
            break;
    }

    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForSynchro();

    rtc.RTC_HourFormat   = RTC_HourFormat_24;
    rtc.RTC_AsynchPrediv = 0x7F;            /* 32768 / (127+1) / (255+1) = 1Hz */
    rtc.RTC_SynchPrediv  = 0xFF;
    RTC_Init(&rtc);                         /* 只配分频器，不清零已走时的时间 */
}

uint8_t RTC_GetDateTime(uint8_t *y, uint8_t *mo, uint8_t *d,
                        uint8_t *h, uint8_t *mi, uint8_t *s)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef dt;

    /* 先读时间再读日期：读TR会锁存日历，读DR才解锁（F4硬性顺序） */
    RTC_GetTime(RTC_Format_BIN, &t);
    RTC_GetDate(RTC_Format_BIN, &dt);

    *y  = dt.RTC_Year;
    *mo = dt.RTC_Month;
    *d  = dt.RTC_Date;
    *h  = t.RTC_Hours;
    *mi = t.RTC_Minutes;
    *s  = t.RTC_Seconds;
    return 0;
}

void RTC_SetDateTime(uint8_t y, uint8_t mo, uint8_t d,
                     uint8_t h, uint8_t mi, uint8_t s)
{
    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef dt = {0};

    t.RTC_Hours   = h;
    t.RTC_Minutes = mi;
    t.RTC_Seconds = s;
    t.RTC_H12     = RTC_H12_AM;
    RTC_SetTime(RTC_Format_BIN, &t);

    dt.RTC_Year    = y;
    dt.RTC_Month   = mo;
    dt.RTC_Date    = d;
    dt.RTC_WeekDay = RTC_Weekday_Monday;
    RTC_SetDate(RTC_Format_BIN, &dt);
}

/* ---------- Unix 时间戳接口（阶段9 断网缓存用） ---------- */

/* 民用日→儒略日数（Howard Hinnant days_from_civil 算法，无 time.h 依赖） */
static uint32_t RTC_DaysFromCivil(uint32_t y, uint32_t m, uint32_t d)
{
    y -= m <= 2;
    uint32_t era = y / 400;
    uint32_t yoe = y - era * 400;                                /* [0, 399] */
    uint32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;/* [0, 365] */
    uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;        /* [0, 146096] */
    return era * 146097 + doe - 719468;                          /* 距 1970-01-01 的天数 */
}

/* 与 RTC_DaysFromCivil 互逆（civil_from_days） */
static void RTC_CivilFromDays(uint32_t z, uint32_t *y, uint32_t *m, uint32_t *d)
{
    z += 719468;
    uint32_t era = z / 146097;
    uint32_t doe = z - era * 146097;                              /* [0, 146096] */
    uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    uint32_t yy = yoe + era * 400;
    uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    uint32_t mp = (5 * doy + 2) / 153;
    *d = doy - (153 * mp + 2) / 5 + 1;
    *m = mp + (mp < 10 ? 3 : -9);
    *y = yy + (*m <= 2);
}

/* 当前 RTC 时间 → Unix 秒（UTC，不做时区换算；SNTP 返回的就是 UTC） */
uint32_t RTC_GetUnix(void)
{
    uint8_t y, mo, d, h, mi, s;
    RTC_GetDateTime(&y, &mo, &d, &h, &mi, &s);
    return RTC_DaysFromCivil(2000u + y, mo, d) * 86400u
           + (uint32_t)h * 3600u + (uint32_t)mi * 60u + s;
}

/* Unix 秒 → 设置 RTC（UTC；SNTP 对时用） */
void RTC_SetUnix(uint32_t unix)
{
    uint32_t days = unix / 86400u;
    uint32_t secs = unix % 86400u;
    uint32_t y, mo, d;
    RTC_CivilFromDays(days, &y, &mo, &d);
    RTC_SetDateTime((uint8_t)(y - 2000u), (uint8_t)mo, (uint8_t)d,
                    (uint8_t)(secs / 3600u), (uint8_t)((secs % 3600u) / 60u),
                    (uint8_t)(secs % 60u));
}

/* RTC 时间是否可信：没装电池断电后回到 2000-01-01，年份 <24 判为未对时。
 * 电池到货后本函数在断电重启时也会返回 1（走时保持） */
uint8_t RTC_IsValid(void)
{
    uint8_t y, mo, d, h, mi, s;
    RTC_GetDateTime(&y, &mo, &d, &h, &mi, &s);
    return (y >= 24) ? 1 : 0;
}

/* ---------- 备份寄存器（跨工程中立接口，datalog 书签用） ---------- */
/* SPL 封装；HAL 工程由 rtc_app.c 用 HAL_RTCEx_BKUPWrite/Read 实现同名函数 */
void RTC_BkpWrite(uint32_t idx, uint32_t val)
{
    RTC_WriteBackupRegister(RTC_BKP_DR0 + idx, val);
}

uint32_t RTC_BkpRead(uint32_t idx)
{
    return RTC_ReadBackupRegister(RTC_BKP_DR0 + idx);
}
