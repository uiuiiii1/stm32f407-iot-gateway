#ifndef __AT24C64_H
#define __AT24C64_H

#include "main.h"   /* HAL：CubeMX 生成，含 stm32f4xx_hal.h */

/* AT24C64 软件 I2C：PB10=SCL，PB11=SDA（开漏输出 + 上拉）
 * 移植自标准库工程 Hardware/AT24C64.c/h；HAL 差异：GPIO 用 HAL API、时钟用 __HAL_RCC */
#define AT24C64_SCL_PORT      GPIOB
#define AT24C64_SCL_PIN       GPIO_PIN_10
#define AT24C64_SDA_PORT      GPIOB
#define AT24C64_SDA_PIN       GPIO_PIN_11

#define AT24C64_DEV_ADDR      0xA0
#define AT24C64_SIZE          8192
#define AT24C64_PAGE_SIZE     32
#define AT24C64_TWC_MS        5
#define AT24C64_I2C_FREQ_KHZ  100

#define AT24C64_OK            0
#define AT24C64_ERR_NAK       1
#define AT24C64_ERR_PARAM     2

uint8_t AT24C64_Init(void);
uint8_t AT24C64_WriteByte(uint16_t Addr, uint8_t Data);
uint8_t AT24C64_ReadByte(uint16_t Addr, uint8_t *Data);
uint8_t AT24C64_Write(uint16_t Addr, const uint8_t *Buf, uint16_t Len);
uint8_t AT24C64_Read(uint16_t Addr, uint8_t *Buf, uint16_t Len);
uint8_t AT24C64_WaitReady(void);

#endif /* __AT24C64_H */
