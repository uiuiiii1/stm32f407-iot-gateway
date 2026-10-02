#include "rtc_app.h"
#include "rtc.h"        /* CubeMX 生成的 hrtc */

/*
 * F407 内部 RTC 驱动（LSE 32.768kHz 板载晶振）—— HAL 版
 * 未装CR1220电池前，断电后时间丢失（回到默认值），装电池后自动掉电走时
 */

/* 备份域访问、LSE 启动（HAL 内部带超时）、24小时制与 127/255 分频
 * 全在 MX_RTC_Init() 里，这里不需要重复配置 */
void RTC_Init_Wrap(void)
{
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
