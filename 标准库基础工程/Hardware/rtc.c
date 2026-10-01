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
