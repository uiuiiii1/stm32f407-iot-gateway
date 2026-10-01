#ifndef __DS1307_H
#define __DS1307_H

#include "stm32f4xx.h"

/* DS1307 返回值：0=成功，非0=错误（透传I2C错误码） */
#define DS1307_OK            0
#define DS1307_ERR_NO_ACK    2   /* 模块无应答（未接/接线错误） */

/* DS1307_Init 返回的时钟状态 */
#define DS1307_CLK_VALID     0   /* 时钟有效（CH=0） */
#define DS1307_CLK_INVALID   1   /* 时钟停振（CH=1），需设置默认时间 */

/* 7位地址0x68，I2C_Send7bitAddress使用8位形式 */
#define DS1307_ADDR_WRITE    0xD0
#define DS1307_ADDR_READ     0xD1
/* 统一用写地址+寄存器寻址方式，这里定义7位左移前的原始值备用 */
#define DS1307_ADDR7         0x68

/* 寄存器0x00~0x06：秒/分/时/星期/日/月/年，BCD码 */
#define DS1307_REG_SEC       0x00
#define DS1307_REG_HOUR      0x02
#define DS1307_REG_WDAY      0x03
#define DS1307_REG_DATE      0x04
#define DS1307_REG_MONTH     0x05
#define DS1307_REG_YEAR      0x06

/* 初始化：检测0x68应答；秒寄存器CH=1（时钟停振）时清零启动时钟
 * 返回 : DS1307_ERR_NO_ACK=模块无应答；DS1307_CLK_VALID=时钟有效；
 *        DS1307_CLK_INVALID=曾停振已清CH（调用方需设置默认时间） */
uint8_t DS1307_Init(void);

/* 设置时间，BCD转换在内部完成
 * 六个入参均为十进制：year传后两位（26=2026年），hour为24小时制
 * 返回 : DS1307_OK=成功；I2C_ERR_TIMEOUT/I2C_ERR_NACK=通信失败 */
uint8_t DS1307_SetTime(uint8_t year, uint8_t month, uint8_t date,
                       uint8_t hour, uint8_t min, uint8_t sec);

/* 读取时间，十进制输出（BCD转换在内部完成）
 * 六个出参均为十进制指针；hour为24小时制
 * 返回 : DS1307_OK=成功；I2C_ERR_TIMEOUT/I2C_ERR_NACK=通信失败 */
uint8_t DS1307_GetTime(uint8_t *hour, uint8_t *min, uint8_t *sec,
                       uint8_t *year, uint8_t *month, uint8_t *date);

/* 读星期（1~7，DS1307不校验具体含义，仅作占位）
 * 返回 : 1~7=星期值；0=通信失败 */
uint8_t DS1307_GetWeekday(void);

#endif /* __DS1307_H */
