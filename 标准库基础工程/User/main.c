#include "stm32f4xx.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "lcd_spi_154.h"
#include "w5500_bsp.h"
#include "modbus.h"
#include "mqtt.h"
#include "rtc.h"
#include "AT24C64.h"
#include "ota.h"
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
 *
 * 阶段10 追加：AT24C64_Init() —— OTA/书签的掉电安全状态介质（软件I2C PB10/PB11），
 * datalog 补传书签后端已切到 AT24C64 双槽接口（见 datalog.c）。模块不在位时
 * 该调用返回 NAK 但不阻塞，书签退化为不持久（断电重头补传，服务端去重兜底）。
 */

int main(void)
{
    /* 阶段10：应用从 0x08000000 搬到 0x08008000（bootloader 之后），
     * 向量表必须重定位到新基址，否则中断全部走 0x08000000 的 bootloader 向量区 */
    SCB->VTOR = 0x08008000;

    /* SPL 中断优先级分组：全4位（FreeRTOS 要求，NVIC 优先级全为抢占优先级） */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* SD 卡 CS(PC6) 预置拉高：阶段11 SD 已改软件 SPI 专用总线（PC6~PC9），
     * 但 PC6 若浮空仍可能让卡被误选中而驱动其 MISO（PC8）——拉高无害且防御
     * 误接线，保留（bootloader 同款处理） */
    {
        GPIO_InitTypeDef gpio = {0};
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
        gpio.GPIO_Pin   = GPIO_Pin_6;
        gpio.GPIO_Mode  = GPIO_Mode_OUT;
        gpio.GPIO_OType = GPIO_OType_PP;
        gpio.GPIO_Speed = GPIO_Speed_100MHz;
        gpio.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_Init(GPIOC, &gpio);
        GPIO_SetBits(GPIOC, GPIO_Pin_6);
    }

    USART1_Init(115200);
    printf("System Start (FreeRTOS V11.3.0)\r\n");
    printf("FW v%s (OTA), build %s %s\r\n", OTA_VER_STR, __DATE__, __TIME__);

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
    RTC_Init_Wrap();     /* 内部RTC（LSE带超时守卫）：阶段9 时间戳/书签依赖它 */
    AT24C64_Init();      /* 阶段10：掉电安全状态介质（软件I2C），datalog 书签后端 */
    W5500_BSP_Init();
    W5500_NetworkInit();
    MQTT_Init();
#if MQTT_LOCAL_BROKER
    printf("Gateway ready: IP 192.168.0.250, broker=LOCAL(192.168.0.105:1883), MQTT pub 5s\r\n");
#else
    printf("Gateway ready: IP 192.168.0.250, broker.emqx.io:1883, MQTT pub 5s\r\n");
#endif

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
