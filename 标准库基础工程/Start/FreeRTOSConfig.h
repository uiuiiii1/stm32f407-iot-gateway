#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*
 * FreeRTOS V11.3.0 配置 —— STM32F407VGT6（168MHz，CM4F，Keil AC5）
 * SysTick 由内核接管（1ms tick）；SVC/PendSV/SysTick 异常名映射见文件底部
 */

#include <stdint.h>

/* ---------- 调度基础 ---------- */
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      ( 168000000UL )
/* ⚠️ 不要定义 configSYSTICK_CLOCK_HZ！port.c 的语义：定义了它就切 SysTick 到
 * 外部时钟 HCLK/8=21MHz（ST 官方配套 reload=21000 的约定），而 reload 按
 * configSYSTICK_CLOCK_HZ/configTICK_RATE_HZ 计算——若=CPU时钟则 tick 慢 8 倍（125Hz），
 * 全部超时/keepalive 拉长 8 倍（实测 MQTT 被服务器踢线）。删掉后 port 默认
 * 用处理器时钟源 + reload=168000，正确 1kHz。 */
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    8
#define configMINIMAL_STACK_SIZE                ( ( uint16_t ) 128 )
#define configMAX_TASK_NAME_LEN                 12
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configNUMBER_OF_CORES                   1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0
#define configSTACK_DEPTH_TYPE                  uint16_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 48 * 1024 ) )
#define configAPPLICATION_ALLOCATED_HEAP        0

/* ---------- 钩子 ---------- */
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2

/* ---------- 可选功能 ---------- */
#define configUSE_TIMERS                        0
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         2
#define configUSE_TRACE_FACILITY                0
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_POSIX_ERRNO                   0

/* ---------- 可选API ---------- */
#define INCLUDE_vTaskPrioritySet               0
#define INCLUDE_uxTaskPriorityGet              0
#define INCLUDE_vTaskDelete                    1
#define INCLUDE_vTaskSuspend                   1
#define INCLUDE_vTaskDelayUntil                1
#define INCLUDE_vTaskDelay                     1
#define INCLUDE_xTaskGetSchedulerState         1
#define INCLUDE_xTaskGetCurrentTaskHandle      1
#define INCLUDE_uxTaskGetStackHighWaterMark    1
#define INCLUDE_uxTaskGetStackHighWaterMark2   0
#define INCLUDE_xTaskGetIdleTaskHandle         0
#define INCLUDE_eTaskGetState                  0
#define INCLUDE_xTaskAbortDelay                0
#define INCLUDE_xTaskGetHandle                 0
#define INCLUDE_xTaskResumeFromISR             0
#define INCLUDE_xTimerPendFunctionCall         0

/* ---------- F4 中断优先级（NVIC 4位，工程使用全4位分组） ---------- */
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS 4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5
#define configKERNEL_INTERRUPT_PRIORITY                 ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY            ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* ---------- 异常名映射：内核实现接管这三个异常（stm32f4xx_it.c 已移除同名处理函数） ---------- */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* ---------- 断言 ---------- */
extern void vAssertCalled( const char * pcFile, int pcLine );
#define configASSERT( x ) if( ( x ) == 0 ) vAssertCalled( __FILE__, ( int ) __LINE__ )

#endif /* FREERTOS_CONFIG_H */
