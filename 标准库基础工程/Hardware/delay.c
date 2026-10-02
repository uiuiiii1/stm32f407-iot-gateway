#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * RTOS 版延时/时基垫片（阶段8）
 * - SysTick 已由 FreeRTOS 接管（1ms tick，见 FreeRTOSConfig.h）
 * - GetTick() = xTaskGetTickCount()（1ms，约49.7天回绕，超时判断用差值比较）
 * - Delay_ms() = vTaskDelay()；调度器启动前（初始化路径）回退为粗略忙等
 *   （仅 w5500 复位时序等初始化场景使用，精度要求不高）
 */

uint32_t GetTick(void)
{
    return (uint32_t)xTaskGetTickCount();
}

void Delay_ms(uint32_t nTime)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        /* 调度器启动前：粗略忙等（168MHz，约10周期/次） */
        volatile uint32_t i = nTime * 14000UL;
        while (i--)
            __NOP();
    }
    else
    {
        vTaskDelay(nTime);
    }
}
