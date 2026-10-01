#include "ds1307.h"
#include "i2c.h"
#include <stdio.h>

/*
 * DS1307 实时时钟驱动（I2C1）
 * 寄存器0x00~0x06全部为BCD码；秒寄存器bit7是CH位（1=停振，必须清零时钟才走）；
 * 小时寄存器保持24小时制（bit6=0）
 */

/* BCD转十进制：0x59 -> 59 */
static uint8_t BCD_ToDec(uint8_t bcd)
{
    return (uint8_t)((bcd >> 4) * 10 + (bcd & 0x0F));
}

/* 十进制转BCD：59 -> 0x59 */
static uint8_t Dec_ToBCD(uint8_t dec)
{
    return (uint8_t)(((dec / 10) << 4) | (dec % 10));
}

/*
 * 初始化：检测0x68应答；若CH=1则清零CH并返回"时钟无效"
 * 返回值：DS1307_ERR_NO_ACK=模块无应答；DS1307_CLK_VALID=时钟有效；DS1307_CLK_INVALID=时钟停振已清CH
 */
uint8_t DS1307_Init(void)
{
    uint8_t sec;
    uint8_t err;

    /* 读秒寄存器，兼作应答检测：无应答/超时说明模块不在总线 */
    err = I2C1_ReadBytes(DS1307_ADDR_WRITE, DS1307_REG_SEC, &sec, 1);
    if (err)
    {
        printf("DS1307_Init: I2C err=%d (1=timeout, 2=NACK)\r\n", err);
        return DS1307_ERR_NO_ACK;
    }

    if (sec & 0x80)                       /* CH=1：时钟停振 */
    {
        sec &= 0x7F;                      /* 清CH位，启动时钟 */
        err = I2C1_WriteBytes(DS1307_ADDR_WRITE, DS1307_REG_SEC, &sec, 1);
        if (err)
            return DS1307_ERR_NO_ACK;
        return DS1307_CLK_INVALID;
    }

    return DS1307_CLK_VALID;
}

/* 设置时间（十进制入参，年传后两位如26表示2026），连续写0x00~0x06 */
uint8_t DS1307_SetTime(uint8_t year, uint8_t month, uint8_t date,
                       uint8_t hour, uint8_t min, uint8_t sec)
{
    uint8_t buf[7];
    /* 星期固定写1（DS1307不校验星期值，仅作占位） */
    buf[0] = Dec_ToBCD(sec) & 0x7F;       /* 秒：清CH位保证时钟走 */
    buf[1] = Dec_ToBCD(min);
    buf[2] = Dec_ToBCD(hour) & 0x3F;      /* 24小时制：bit6=0 */
    buf[3] = 0x01;                        /* 星期 */
    buf[4] = Dec_ToBCD(date);
    buf[5] = Dec_ToBCD(month);
    buf[6] = Dec_ToBCD(year);

    return I2C1_WriteBytes(DS1307_ADDR_WRITE, DS1307_REG_SEC, buf, 7);
}

/* 读取时间：从0x00连续读7字节，BCD转十进制输出 */
uint8_t DS1307_GetTime(uint8_t *hour, uint8_t *min, uint8_t *sec,
                       uint8_t *year, uint8_t *month, uint8_t *date)
{
    uint8_t buf[7];
    uint8_t err;

    err = I2C1_ReadBytes(DS1307_ADDR_WRITE, DS1307_REG_SEC, buf, 7);
    if (err)
        return err;

    *sec   = BCD_ToDec(buf[0] & 0x7F);    /* 屏蔽CH位 */
    *min   = BCD_ToDec(buf[1] & 0x7F);
    *hour  = BCD_ToDec(buf[2] & 0x3F);    /* 24小时制 */
    *year  = BCD_ToDec(buf[6]);
    *month = BCD_ToDec(buf[5] & 0x1F);
    *date  = BCD_ToDec(buf[4] & 0x3F);

    return DS1307_OK;
}

/* 读星期（1~7），用于显示"Wn" */
uint8_t DS1307_GetWeekday(void)
{
    uint8_t wday;
    uint8_t err;

    err = I2C1_ReadBytes(DS1307_ADDR_WRITE, DS1307_REG_WDAY, &wday, 1);
    if (err)
        return 0;

    return BCD_ToDec(wday & 0x07);
}
