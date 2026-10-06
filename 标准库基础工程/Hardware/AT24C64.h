#ifndef __AT24C64_H
#define __AT24C64_H

#include "stm32f4xx.h"

/* AT24C64 软件 I2C：PB10=SCL，PB11=SDA（开漏输出 + 上拉）
 * 移植自 F:\STM32-OTA·远程升级\07-bootloader程序（标准库）\Hardware\AT24C64.*
 * 改动：stm32f10x.h -> stm32f4xx.h；GPIO 时钟 RCC_APB2 -> RCC_AHB1 */
#define AT24C64_SCL_PORT      GPIOB          // I2C时钟引脚所在GPIO端口：PB
#define AT24C64_SCL_PIN       GPIO_Pin_10    // I2C时钟SCL引脚：PB10
#define AT24C64_SDA_PORT      GPIOB          // I2C数据引脚所在GPIO端口：PB
#define AT24C64_SDA_PIN       GPIO_Pin_11    // I2C数据SDA引脚：PB11
#define AT24C64_DEV_ADDR      0xA0           // AT24C64器件I2C从机地址（A0/A1/A2全部拉低）
#define AT24C64_SIZE          8192           // AT24C64总容量：8192字节 = 64Kbit
#define AT24C64_PAGE_SIZE     32             // 页写大小：一页32字节，页写不能跨页
#define AT24C64_TWC_MS        5              // EEPROM内部写周期最大等待时间5ms
#define AT24C64_I2C_FREQ_KHZ  100            // I2C通信速率 100KHz
#define AT24C64_OK            0              // 返回值：操作成功
#define AT24C64_ERR_NAK       1              // 返回值：收到NAK，I2C应答失败
#define AT24C64_ERR_PARAM     2              // 返回值：参数错误（地址越界、空指针等）

uint8_t AT24C64_Init(void);                     // AT24C64初始化：配置GPIO开漏、开启DWT计数器
uint8_t AT24C64_WriteByte(uint16_t Addr, uint8_t Data); // 单字节写入，Addr:EEPROM地址，Data待写数据
uint8_t AT24C64_ReadByte(uint16_t Addr, uint8_t *Data);// 单字节读取，Addr:读取地址，Data输出读到的数据
uint8_t AT24C64_Write(uint16_t Addr, const uint8_t *Buf, uint16_t Len); // 多字节页写，自动分页处理
uint8_t AT24C64_Read(uint16_t Addr, uint8_t *Buf, uint16_t Len);        // 多字节连续读取
uint8_t AT24C64_WaitReady(void);                 // 等待EEPROM写完，轮询应答，等待内部写操作完成

#endif /* __AT24C64_H */
