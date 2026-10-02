/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
extern void xPortSysTickHandler(void);   /* V11 未在头文件公开声明（port.c 内部），链式调用需显式声明 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 故障指示：PC0 用户 LED（低电平点亮）快闪 4 下 + 停顿，无限循环。
 * 寄存器直写 + 关中断，不依赖任何外设初始化是否完成——初始化最早期失败也能给出指示。
 * 与主循环"2秒一闪"的心跳灯节奏明显不同：看到快闪 = 进了故障处理。
 * HardFault 不在此定义处理函数：AC5 下启动文件的 [WEAK] 导出与 C 的 __weak 定义
 * 会报 L6200E 重复定义（实测），任何强定义又会与 CubeMX 重生成的默认版冲突，
 * 故 HardFault 使用启动文件弱默认（死循环）；其余异常用 Fault_Blink 给出指示。 */
void Fault_Blink(void)
{
  volatile uint32_t d;
  uint32_t k;

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  GPIOC->MODER = (GPIOC->MODER & ~(3U << (0 * 2))) | (1U << (0 * 2));
  __disable_irq();
  while (1)
  {
    for (k = 0; k < 4; k++)
    {
      GPIOC->BSRR = (uint32_t)GPIO_PIN_0 << 16;   /* 拉低 = 点亮 */
      for (d = 0; d < 300000; d++) { }
      GPIOC->BSRR = GPIO_PIN_0;                   /* 拉高 = 熄灭 */
      for (d = 0; d < 300000; d++) { }
    }
    for (d = 0; d < 1200000; d++) { }
  }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  Fault_Blink();
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/* HardFault_Handler 未在此定义：缺省使用启动文件的弱默认实现（死循环），
 * 避免 CubeMX 重新生成默认版本时重复定义 */

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  Fault_Blink();
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  Fault_Blink();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  Fault_Blink();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/* SVC_Handler / PendSV_Handler 未在此定义：FreeRTOSConfig.h 已把 port 的
 * vPortSVCHandler/xPortPendSVHandler 映射为这两个名字（调度器启动/任务切换用）。
 * ⚠️ 若用 CubeMX 重新生成 it.c 会恢复空实现 → 与 port 重复定义（L6200E），
 *    届时删除生成版这两个函数即可。 */

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/* PendSV_Handler 同 SVC_Handler：由 port 实现，勿在此定义 */

/**
  * @brief This function handles System tick timer.
  * @note  链式设计：HAL 时基（HAL_IncTick，modbus字节超时/HAL_Delay 依赖）
  *        与 FreeRTOS tick 共用 SysTick。调度器启动前 port 未接管，
  *        只走 HAL_IncTick；启动后两边都走。
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  HAL_IncTick();
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    xPortSysTickHandler();
  }
  /* USER CODE END SysTick_IRQn 0 */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
