#ifndef __MODBUS_H
#define __MODBUS_H

#include "main.h"

/* 返回值：0=成功，非0=错误 */
#define MB_OK           0
#define MB_ERR_TIMEOUT  1   /* 等待回复超时 */
#define MB_ERR_CRC      2   /* CRC校验失败 */
#define MB_ERR_FRAME    3   /* 响应帧格式不对（地址/功能码/长度） */

/* ===== 配置区：按变送器寄存器表调整（当前值来自商家资料） ===== */
#define MB_SLAVE_ADDR   0x01      /* 从站地址 */
#define MB_BAUDRATE     9600      /* 波特率 */
#define MB_REG_START    0x0000    /* 起始寄存器：0x0000=湿度，0x0001=温度 */
#define MB_REG_COUNT    2         /* 一次读湿度+温度共2个寄存器 */
/* ==================================================== */

/* 初始化：USART2(PA2=TX/PA3=RX) 与 DE 脚 PA4 已由 CubeMX 生成代码配置好，
 * 本函数只把 DE 置于接收态，保留是为了与标准库版本 main 调用一致 */
void    MB_USART_Init(void);

/* 读保持寄存器（功能码03），失败自动重试2次（共3次尝试）
 * regStart : 起始寄存器地址（如0x0000）
 * count    : 寄存器个数（1~4）
 * out      : 输出缓冲区，存寄存器原始值（高字节在前已拼好）
 * 返回     : MB_OK=成功；MB_ERR_TIMEOUT=超时；MB_ERR_CRC=CRC错；MB_ERR_FRAME=帧格式错 */
uint8_t MODBUS_ReadRegs(uint16_t regStart, uint16_t count, uint16_t *out);

/* 读温湿度（内部调MODBUS_ReadRegs，一次读0x0000起的2个寄存器）
 * temp10 : 出参，温度×10，有符号补码——-101 表示 -10.1℃
 * hum10  : 出参，湿度×10，无符号——292 表示 29.2%RH
 * 返回   : MB_OK=成功；其余同MODBUS_ReadRegs */
uint8_t MODBUS_ReadTempHum(int16_t *temp10, uint16_t *hum10);

#endif /* __MODBUS_H */
