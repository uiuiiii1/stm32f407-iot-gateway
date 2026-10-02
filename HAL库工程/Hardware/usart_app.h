#ifndef __USART_APP_H
#define __USART_APP_H

#include "main.h"

/* 调试串口（CH340）：PA9=TX / PA10=RX
 * 接线：CH340 TXD->PA10、RXD->PA9、GND共地；CH340的3.3V/5V不要接（只留三根线）
 * 注意：本系统为单电源同源供电，CH340插拔电脑不应影响板子运行
 *
 * 文件名叫 usart_app 是为了避开 CubeMX 生成的 Core/Inc/usart.h（MX_USARTx_UART_Init 在那边） */

/* 初始化USART1调试串口
 * baudrate : 波特率（本项目固定传115200，8-N-1）
 * 引脚/外设初始化由 MX_USART1_UART_Init 完成，这里只按需改波特率；
 * 初始化后 printf 已重定向到本串口（MicroLIB + fputc，见 usart_app.c） */
void USART1_Init(uint32_t baudrate);

#endif /* __USART_APP_H */
