#include "rtc_app.h"
#include "rtc.h"        /* CubeMX 生成的 hrtc */

/*
 * F407 内部 RTC 驱动（LSE 32.768kHz 板载晶振）—— HAL 版
 * 未装CR1220电池前，断电后时间丢失（回到默认值），装电池后自动掉电走时
 */

/* 备份域访问、LSE 启动（HAL 内部带超时）、24小时制与 127/255 分频
 * 全在 MX_RTC_Init() 里。这里补开备份域写使能（DBP位）：
 * RTC_SetTime/SetDate 与备份寄存器写入都必须在 DBP=1 时才生效 */
void RTC_Init_Wrap(void)
{
    HAL_PWR_EnableBkUpAccess();
}

uint8_t RTC_GetDateTime(uint8_t *y, uint8_t *mo, uint8_t *d,
                        uint8_t *h, uint8_t *mi, uint8_t *s)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef dt;

    /* 先读时间再读日期：读TR会锁存日历，读DR才解锁（F4硬性顺序，HAL同样要求） */
    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &dt, RTC_FORMAT_BIN);

    *y  = dt.Year;
    *mo = dt.Month;
    *d  = dt.Date;
    *h  = t.Hours;
    *mi = t.Minutes;
    *s  = t.Seconds;
    return 0;
}

void RTC_SetDateTime(uint8_t y, uint8_t mo, uint8_t d,
                     uint8_t h, uint8_t mi, uint8_t s)
{
    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef dt = {0};

    t.Hours   = h;
    t.Minutes = mi;
    t.Seconds = s;
    t.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    t.StoreOperation = RTC_STOREOPERATION_RESET;
    HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN);

    dt.Year    = y;
    dt.Month   = mo;
    dt.Date    = d;
    dt.WeekDay = RTC_WEEKDAY_MONDAY;
    HAL_RTC_SetDate(&hrtc, &dt, RTC_FORMAT_BIN);
}

/* ---------- Unix 时间戳（阶段9 断网缓存用，与标准库版同名同语义） ---------- */

/* 民用日→天数（Howard Hinnant days_from_civil 算法，无 time.h 依赖） */
static uint32_t RTC_DaysFromCivil(uint32_t y, uint32_t m, uint32_t d)
{
    y -= m <= 2;
    uint32_t era = y / 400;
    uint32_t yoe = y - era * 400;
    uint32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

/* 天数→民用日（civil_from_days，与上互逆） */
static void RTC_CivilFromDays(uint32_t z, uint32_t *y, uint32_t *m, uint32_t *d)
{
    z += 719468;
    uint32_t era = z / 146097;
    uint32_t doe = z - era * 146097;
    uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    uint32_t yy = yoe + era * 400;
    uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    uint32_t mp = (5 * doy + 2) / 153;
    *d = doy - (153 * mp + 2) / 5 + 1;
    *m = mp + (mp < 10 ? 3 : -9);
    *y = yy + (*m <= 2);
}

/* 当前 RTC 时间 → Unix 秒（UTC，不做时区换算） */
uint32_t RTC_GetUnix(void)
{
    uint8_t y, mo, d, h, mi, s;
    RTC_GetDateTime(&y, &mo, &d, &h, &mi, &s);
    return RTC_DaysFromCivil(2000u + y, mo, d) * 86400u
           + (uint32_t)h * 3600u + (uint32_t)mi * 60u + s;
}

/* Unix 秒 → 设置 RTC（UTC；SNTP 网络对时用） */
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

/* RTC 时间是否可信：断电后 RTC 回到 2000-01-01，年份 <24 = 未对时。
 * 电池到货后断电走时保持，本函数断电重启也返回 1 */
uint8_t RTC_IsValid(void)
{
    uint8_t y, mo, d, h, mi, s;
    RTC_GetDateTime(&y, &mo, &d, &h, &mi, &s);
    return (y >= 24) ? 1 : 0;
}

/* ---------- 备份寄存器（跨工程中立接口，datalog 书签用） ---------- */
void RTC_BkpWrite(uint32_t idx, uint32_t val)
{
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0 + idx, val);
}

uint32_t RTC_BkpRead(uint32_t idx)
{
    return HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0 + idx);
}
