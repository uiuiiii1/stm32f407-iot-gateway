#include "stm32f4xx_it.h"

/* 核心异常服务程序
 * 注意：SVC/PendSV/SysTick 三个异常已由 FreeRTOS 接管
 * （FreeRTOSConfig.h 里把 vPortSVCHandler/xPortPendSVHandler/xPortSysTickHandler
 *   映射到 SVC_Handler/PendSV_Handler/SysTick_Handler，实现位于 port.c），
 * 本文件不得再定义它们，否则与内核重复定义、链接报错或调度失效。
 * 外设中断的服务函数后续按需在此添加 */

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1);
}

void MemManage_Handler(void)
{
    while (1);
}

void BusFault_Handler(void)
{
    while (1);
}

void UsageFault_Handler(void)
{
    while (1);
}

void DebugMon_Handler(void)
{
}
