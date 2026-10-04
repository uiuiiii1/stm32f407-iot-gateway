/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f4xx_it.h"   /* Fault_Blink：Error_Handler / 异常时的 LED 故障指示 */
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "usart_app.h"      /* USART1 调试串口 + printf 重定向 */
#include "lcd_spi_154.h"
#include "w5500_bsp.h"
#include "modbus.h"
#include "mqtt.h"
#include "rtc_app.h"
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 阶段8：FreeRTOS 多任务。任务/共享状态/心跳全部在 app.c，
 * 本文件只负责外设初始化 -> APP_Init() -> 启动调度器。 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * HAL 工程 —— 阶段8：FreeRTOS 多任务（与标准库版功能一致）
 * W5500 接线：SCLK=PA5 / MISO=PA6 / MOSI=PA7（SPI1），nSS=PC4，nRST=PC5
 * 网络：静态IP 192.168.0.250（见 w5500_bsp.h 配置区）；Modbus 变送器接 USART2（DE=PA4）
 *
 * SysTick 归属：HAL_Init 先按 1ms 配好（HAL_IncTick 时基），vTaskStartScheduler
 * 时由 port 重新配置为同频率 FreeRTOS tick；it.c 的 SysTick_Handler 链式调用
 * 两者（HAL_IncTick + xPortSysTickHandler），HAL_GetTick/HAL_Delay 全程可用，
 * 无需像 CubeMX 那样把 HAL 时基迁到 TIM6/7。
 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  RTC_Init_Wrap();   /* 开备份域写使能（DBP）：必须在 MX_RTC_Init 之前，阶段9 对时/书签依赖 */
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* MX_GPIO_Init 按 .ioc 里的默认状态把 PD12 拉低了，这里立刻重新点亮背光 */
  LCD_Backlight_ON;

  /* 时钟/GPIO/SPI/USART/RTC 已由上面的 MX_xxx_Init 配好，这里只做应用层初始化 */
  Delay_Init();
  USART1_Init(115200);
  printf("System Start (FreeRTOS V11.3.0)\r\n");

  SPI_LCD_Init();

  LCD_SetAsciiFont(&ASCII_Font16);  LCD_SetBackColor(LCD_WHITE);
  LCD_SetColor(LCD_BLACK);
  LCD_Clear();
  LCD_ShowNumMode(Fill_Space);  /* 数字不足位补空格 */

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

  /* 创建互斥锁 + 4 任务（详见 app.c），随后交出控制权给 FreeRTOS */
  APP_Init();
  printf("Starting scheduler...\r\n");
  vTaskStartScheduler();

  /* 正常不会走到这里：走到 = 堆不足等致命错误 */
  printf("FATAL: scheduler start failed\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 调度器已启动且不会返回，这里只是兜底 */
  }
  /* USER CODE END 3 */
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  /* 手工修改（CubeMX 重新生成会覆盖，需重打）：
   * LSE(32.768K) 起振失败时不让整板卡死在 Error_Handler。
   * 失败则降级：关 LSE、开 LSI 做 RTC 时钟（RTC 仍可走时，只是断电不保持）。 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 初始化失败：PC0 LED 快闪报错（寄存器直写，不依赖外设初始化结果），
   * 不静默死循环——避免"程序没跑起来"和"初始化失败"分不清。 */
  Fault_Blink();
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
