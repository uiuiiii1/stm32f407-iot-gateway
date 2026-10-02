#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

/* RTOS 版：SysTick 由 FreeRTOS 接管（1ms tick）
 * Delay_Init 已废弃（保留空实现兼容旧调用点，可不再调用） */
void Delay_Init(void);
void Delay_ms(uint32_t nTime);      /* 毫秒延时：在线时=vTaskDelay，启动前=忙等 */
uint32_t GetTick(void);             /* 上电以来累计毫秒数（RTOS tick），超时判断用差值比较 */

#endif //__DELAY_H
