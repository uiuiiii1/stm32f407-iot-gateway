#ifndef __RTC_APP_H
#define __RTC_APP_H

#include "main.h"

/* F407 内部RTC（LSE 32.768kHz 板载晶振）：替代故障DS1307的时间源
 * VBAT电池（CR1220）到位后可掉电走时；此前每次上电需串口发 's' 设置基准时间
 * （后续可接SNTP网络对时自动校准）
 *
 * HAL 版说明：文件名叫 rtc_app 是因为 CubeMX 已生成 Core/Inc/rtc.h，同名会冲突 */

/* 初始化：LSE、备份域、24小时制、127/255 分频都由 CubeMX 生成的
 * MX_RTC_Init() 完成了，这里保留空实现只为与标准库版本调用一致 */
void RTC_Init_Wrap(void);

/* 读取当前日期时间（十进制出参）。返回0=成功 */
uint8_t RTC_GetDateTime(uint8_t *y, uint8_t *mo, uint8_t *d,
                        uint8_t *h, uint8_t *mi, uint8_t *s);

/* 设置日期时间（十进制入参，year传后两位如26=2026，hour为24小时制） */
void RTC_SetDateTime(uint8_t y, uint8_t mo, uint8_t d,
                     uint8_t h, uint8_t mi, uint8_t s);

/* ---------- Unix 时间戳（阶段9，与标准库版同名同语义） ---------- */
uint32_t RTC_GetUnix(void);
void RTC_SetUnix(uint32_t unix);
uint8_t RTC_IsValid(void);

/* ---------- 备份寄存器（跨工程中立接口，idx=0..19） ---------- */
void RTC_BkpWrite(uint32_t idx, uint32_t val);
uint32_t RTC_BkpRead(uint32_t idx);

#endif /* __RTC_APP_H */
