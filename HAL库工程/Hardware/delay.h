#ifndef __DELAY_H
#define __DELAY_H

#include "main.h"

/* HAL 版延时：SysTick 1ms 节拍由 CubeMX/HAL 接管（HAL_IncTick），
 * 这里只是把标准库工程的 delay 接口映射到 HAL，上层代码调用方式不变 */

void Delay_Init(void);              /* 空实现：HAL 已在 SystemClock_Config 后启动 SysTick */
void Delay_ms(uint32_t nTime);      /* 毫秒延时（阻塞） */
uint32_t GetTick(void);             /* 上电以来累计毫秒数，供驱动做非阻塞超时判断（约49.7天回绕，用差值比较） */

#endif //__DELAY_H
