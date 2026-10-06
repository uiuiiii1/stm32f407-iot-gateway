#ifndef __TICK_H
#define __TICK_H

#include <stdint.h>

/* bootloader 裸机毫秒时基：SysTick 1ms 中断（w25q64 的 WaitBusy 超时依赖 GetTick） */
void Tick_Init(void);
uint32_t GetTick(void);

#endif /* __TICK_H */
