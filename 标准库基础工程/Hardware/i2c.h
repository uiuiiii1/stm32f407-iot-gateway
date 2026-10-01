#ifndef __I2C_H
#define __I2C_H

#include "stm32f4xx.h"

/* 错误码定义：0=成功，非0=错误 */
#define I2C_OK              0
#define I2C_ERR_TIMEOUT     1   /* 等待事件/标志位超时 */
#define I2C_ERR_NACK        2   /* 从机无应答 */

/* 初始化I2C1：PB6=SCL、PB7=SDA，复用开漏+上拉，100kHz标准模式 */
void I2C1_Init(void);

/* 向从机寄存器连续写数据（START -> 地址W -> 寄存器地址 -> 数据 -> STOP）
 * slaveAddr : 8位从机地址（7位地址左移1位，如DS1307写0xD0）
 * regAddr   : 目标寄存器起始地址
 * pData/len : 待写数据及长度
 * 返回      : I2C_OK=成功；I2C_ERR_TIMEOUT=超时；I2C_ERR_NACK=从机无应答 */
uint8_t I2C1_WriteBytes(uint8_t slaveAddr, uint8_t regAddr, const uint8_t *pData, uint16_t len);

/* 从从机寄存器连续读数据（写寄存器地址后重复START转接收）
 * slaveAddr/regAddr : 同上
 * pData/len         : 接收缓冲区及读取长度
 * 返回              : 同上 */
uint8_t I2C1_ReadBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t len);

/* 从从机指定16位寄存器（字）地址连续读数据（AT24C32等双字节地址EEPROM专用）
 * 写字地址高、低两字节后重复START转接收，其余同I2C1_ReadBytes */
uint8_t I2C1_ReadAddr16(uint8_t slaveAddr, uint16_t regAddr, uint8_t *pData, uint16_t len);

/* 总线探针：只发从机地址（8位形式）测应答，不做寄存器操作，用于硬件诊断
 * addr8 : 8位从机地址，如0xD0
 * 返回  : 1=有应答；0=NACK/超时（失败原因已打印到串口） */
uint8_t I2C1_ProbeAddr(uint8_t addr8);

#endif /* __I2C_H */
