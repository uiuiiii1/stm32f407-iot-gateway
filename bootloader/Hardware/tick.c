/* bootloader 裸机毫秒时基：SysTick 1ms 中断
 * 注意：FreeRTOS 主工程里 SysTick 由 port 接管，这里是 bootloader 独立工程，
 * 两者不会同时存在（跳转应用前 Int_bootloader 会把 SysTick 停掉） */
#include "tick.h"
#include "stm32f4xx.h"

static volatile uint32_t s_tick;

void Tick_Init(void)
{
    SysTick_Config(SystemCoreClock / 1000U);   /* 1ms，中断入口 = SysTick_Handler */
}

uint32_t GetTick(void)
{
    return s_tick;
}

/* 覆盖启动文件的 [WEAK] 默认实现 */
void SysTick_Handler(void)
{
    s_tick++;
}
