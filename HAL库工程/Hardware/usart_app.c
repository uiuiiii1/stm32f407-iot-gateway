/*
 * usart_app.c —— 调试串口应用层（HAL 版）
 * 标准库工程里 USART1_Init 负责 GPIO+外设初始化；HAL 工程里这些由
 * MX_USART1_UART_Init（Core/Src/usart.c）完成，此处只保留波特率设置与 printf 重定向
 */
#include <stdio.h>
#include "usart_app.h"
#include "usart.h"      /* CubeMX 生成：extern UART_HandleTypeDef huart1/huart2 */

void USART1_Init(uint32_t baudrate)
{
    if (huart1.Init.BaudRate != baudrate)
    {
        huart1.Init.BaudRate = baudrate;
        if (HAL_UART_Init(&huart1) != HAL_OK)
            Error_Handler();
    }
}

/* printf重定向到USART1（MicroLIB） */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}
