#include "stm32f4xx_it.h"
#include "delay.h"

/* 核心异常服务程序。外设中断的服务函数后续按需在此添加 */

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

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    TimingDelay_Decrement();    /* delay.c 的1ms时基计数 */
}
