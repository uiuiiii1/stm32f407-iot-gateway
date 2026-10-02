/***
	***************************************************************************
	*	@file  	delay.c
	*	@brief   delay接口相关函数（HAL + FreeRTOS 版）
   ***************************************************************************
	*  RTOS 垫片：调度器启动后 Delay_ms=vTaskDelay、GetTick=FreeRTOS tick 计数；
	*  启动前 SysTick 仍归 HAL（HAL_IncTick），Delay_ms 忙等 HAL_GetTick。
	*  接口签名与裸机版一致，驱动层调用方式不变。
	***************************************************************************
***/

#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

//	函数：延时初始化
//	说明：SysTick 已由 HAL_Init 启动（1ms，HAL_IncTick），调度器启动后由
//        FreeRTOS port 重新配置同频率 tick，此处无需操作
//
void Delay_Init(void)
{
}

//	函数：毫秒延时
// 参数：nTime - 延时时间，单位ms
// 说明：调度器运行中 = vTaskDelay（让出CPU）；启动前 = 忙等 HAL 时基
//       （供 main 初始化阶段与驱动在两种阶段统一调用）
//
void Delay_ms(uint32_t nTime)
{
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		vTaskDelay(nTime);
	}
	else
	{
		uint32_t start = HAL_GetTick();
		while ((HAL_GetTick() - start) < nTime) { }
	}
}

//	函数：获取上电以来的毫秒数
//	说明：调度器启动后 = FreeRTOS tick 计数（供任务/驱动做非阻塞超时判断，
//        约49.7天回绕，超时判断用差值比较）。仅调度器启动后使用。
//
uint32_t GetTick(void)
{
	return (uint32_t)xTaskGetTickCount();
}
