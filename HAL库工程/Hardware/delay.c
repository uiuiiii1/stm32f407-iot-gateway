/***
	***************************************************************************
	*	@file  	delay.c
	*	@brief   delay接口相关函数（HAL 版）
   ***************************************************************************
	*  标准库工程用 SysTick_Config + Own 中断计数实现；HAL 工程里 SysTick 已由
	*  CubeMX 配置为 1ms 并在 stm32f4xx_it.c 的 SysTick_Handler 中调用 HAL_IncTick，
	*  因此直接复用 HAL_GetTick / HAL_Delay，接口签名与标准库版保持一致。
	***************************************************************************
***/

#include "delay.h"

//	函数：延时初始化
//	说明：HAL 的 SysTick 已由 MX 配置启动，此处无需操作
//
void Delay_Init(void)
{
}

//	函数：毫秒延时
// 参数：nTime - 延时时间，单位ms
//
void Delay_ms(uint32_t nTime)
{
	HAL_Delay(nTime);
}

//	函数：获取上电以来的毫秒数
//	说明：供驱动做非阻塞超时判断；注意约49.7天回绕，超时判断用差值比较
//
uint32_t GetTick(void)
{
	return HAL_GetTick();
}
