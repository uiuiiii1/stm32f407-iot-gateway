#ifndef __RTC_H
#define __RTC_H

#include "stm32f4xx.h"

/* F407 内部RTC（LSE 32.768kHz 板载晶振）：替代故障DS1307的时间源
 * VBAT电池（CR1220）到位后可掉电走时；此前每次上电需串口发 's' 设置基准时间
 * （后续可接SNTP网络对时自动校准） */

/* 初始化：备份域访问+LSE启动（带超时守卫，晶振异常不死机）+24小时制
 * 可重复调用（不破坏已走时的时间） */
void RTC_Init_Wrap(void);

/* 读取当前日期时间（十进制出参）。返回0=成功 */
uint8_t RTC_GetDateTime(uint8_t *y, uint8_t *mo, uint8_t *d,
                        uint8_t *h, uint8_t *mi, uint8_t *s);

/* 设置日期时间（十进制入参，year传后两位如26=2026，hour为24小时制） */
void RTC_SetDateTime(uint8_t y, uint8_t mo, uint8_t d,
                     uint8_t h, uint8_t mi, uint8_t s);

#endif /* __RTC_H */
