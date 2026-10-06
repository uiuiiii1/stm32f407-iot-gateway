#ifndef __SPI2_H
#define __SPI2_H

#include "stm32f4xx.h"

/* 初始化SPI2主机：PB13=SCK / PB14=MISO / PB15=MOSI（AF5），模式0，5.25MHz（42MHz/8）
 * 片选不在本层——由各器件驱动自行控制（W25Q64=PB12、SD卡=PC6），便于多设备共享总线 */
void SPI2_Init(void);

/* 全双工收发一个字节：写入tx并返回从机移出的字节
 * "只写"时忽略返回值，"只读"时传 0xFF 取返回 */
uint8_t SPI2_RW(uint8_t tx);

#endif /* __SPI2_H */
