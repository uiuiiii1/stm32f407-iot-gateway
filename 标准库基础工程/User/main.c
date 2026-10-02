#include "stm32f4xx.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "lcd_spi_154.h"
#include "w5500_bsp.h"
#include "modbus.h"
#include "mqtt.h"
#include "app.h"

/*
 * 标准库基础工程 —— 阶段8：FreeRTOS 多任务（里程碑B第一站）
 *
 * 本文件只负责：外设初始化 -> APP_Init() 创建任务 -> 启动调度器。
 * 任务、共享状态、心跳机制见 User/app.c。
 *
 * 关键变化（相对阶段7裸机版）：
 *   - SysTick 归 FreeRTOS，delay.c 已改为 RTOS 垫片（GetTick=tick计数、Delay_ms=vTaskDelay）
 *   - ioLibrary 已注册临界区回调（w5500_bsp.c）
 */

int main(void)
{
    /* SPL 中断优先级分组：全4位（FreeRTOS 要求，NVIC 优先级全为抢占优先级） */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    USART1_Init(115200);
    printf("System Start (FreeRTOS V11.3.0)\r\n");

    SPI_LCD_Init();
    LCD_SetAsciiFont(&ASCII_Font16);
    LCD_SetBackColor(LCD_WHITE);
    LCD_SetColor(LCD_BLACK);
    LCD_Clear();
    LCD_ShowNumMode(Fill_Space);

    /* 仪表盘静态框架 */
    LCD_DisplayString(16, 16,  "Industrial Gateway");
    LCD_DisplayString(16, 40,  "Link:");
    LCD_DisplayString(16, 64,  "MQTT:");
    LCD_DisplayString(16, 92,  "Temp:");
    LCD_DisplayString(16, 116, "Hum :");
    LCD_DisplayString(112, 92,  "C");
    LCD_DisplayString(112, 116, "%");
    LCD_DisplayString(16, 144, "IP: 192.168.0.250");
    LCD_DisplayString(16, 168, "PUB:");

    MB_USART_Init();
    W5500_BSP_Init();
    W5500_NetworkInit();
    MQTT_Init();
    printf("Gateway ready: IP 192.168.0.250, broker.emqx.io:1883, MQTT pub 5s\r\n");

    /* 创建共享状态互斥锁 + 4个任务（详见 app.c） */
    APP_Init();

    printf("Starting scheduler...\r\n");
    vTaskStartScheduler();

    /* 正常不会走到这里：走到=堆不足等致命错误 */
    printf("FATAL: scheduler start failed\r\n");
    for (;;);
}

/* ---------- FreeRTOS 钩子 ---------- */

void vApplicationMallocFailedHook(void)
{
    printf("FATAL: malloc failed\r\n");
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    printf("FATAL: stack overflow in %s\r\n", pcTaskName ? pcTaskName : "?");
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vAssertCalled(const char *pcFile, int pcLine)
{
    printf("ASSERT: %s:%d\r\n", pcFile, pcLine);
    taskDISABLE_INTERRUPTS();
    for (;;);
}
