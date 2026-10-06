#include "Int_bootloader.h"
#include "App_bootloader.h"
#include "stm32f4xx.h"
#include <stdio.h>

/* 跳转到应用：校验栈顶/复位向量 → 注销 bootloader 运行环境 → 设 MSP/VTOR → 跳复位。
 * 移植自 F1 版（07-bootloader程序（标准库）），F4 差异仅头文件与分区常量 */
uint8_t bootloader_jump_to_app(uint32_t app_base_addr)
{
    typedef void (*pFunc)(void);

    /* 1，校验：栈顶必须落在 SRAM；复位向量必须落在本应用分区内 */
    uint32_t app_start_ptr    = *(volatile uint32_t *)app_base_addr;        /* 栈顶 */
    uint32_t app_reset_vector = *(volatile uint32_t *)(app_base_addr + 4);  /* 复位向量 */

    if ((app_start_ptr & 0xFFFF0000) != 0x20000000UL)
    {
        printf("Stack pointer error\r\n");
        return 1;
    }
    if (app_base_addr != APP_FLASH_BASE_ADDR)
    {
        printf("App base address error\r\n");
        return 1;
    }
    {
        uint32_t vector_addr = app_reset_vector & 0xFFFFFFFEU;   /* 去 Thumb 位 */
        if (vector_addr < app_base_addr || vector_addr >= APP_END_ADDR)
        {
            printf("Reset address error\r\n");
            return 1;
        }
    }

    /* 2，注销 bootloader 运行环境：关中断、清 NVIC、停 SysTick
     * （W25_WaitBusy 依赖 GetTick，故擦写期间不能全局关中断，这里跳转前才关） */
    __disable_irq();
    {
        uint32_t i;
        for (i = 0; i < 8; i++)
        {
            NVIC->ICER[i] = 0xFFFFFFFFU;   /* disable all */
            NVIC->ICPR[i] = 0xFFFFFFFFU;   /* clear pending */
        }
    }
    SysTick->CTRL = 0;
    SysTick->VAL  = 0;
    SysTick->LOAD = 0;

    /* 3，设主栈指针 + 重定向向量表 + 跳复位 */
    __set_MSP(app_start_ptr);
    SCB->VTOR = app_base_addr;

    {
        pFunc jump_to_app = (pFunc)app_reset_vector;
        jump_to_app();
    }
    return 0;
}
