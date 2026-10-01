#include "i2c.h"
#include <stdio.h>

/*
 * I2C1 驱动（PB6=SCL, PB7=SDA），SPL 标准库实现
 * 所有等待事件/标志位的循环都带超时计数，
 * 从机（如DS1307）未接时返回错误码而不是卡死
 */

/* 超时计数上限：100MHz主频下循环约2~3个周期，该值对应几十毫秒量级 */
#define I2C_TIMEOUT_CNT   200000

/* 等待指定事件（EVx），带超时；NACK快速失败，返回0=事件到来 */
static uint8_t I2C_WaitEvent(uint32_t event)
{
    uint32_t timeout = I2C_TIMEOUT_CNT;
    while (I2C_CheckEvent(I2C1, event) == ERROR)
    {
        if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF) != RESET)   /* 收到NACK：立即失败并区分原因 */
        {
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            return I2C_ERR_NACK;
        }
        if ((timeout--) == 0)
            return I2C_ERR_TIMEOUT;
    }
    return I2C_OK;
}

/* 等待指定标志位置位，带超时 */
static uint8_t I2C_WaitFlag(uint32_t flag, FlagStatus status)
{
    uint32_t timeout = I2C_TIMEOUT_CNT;
    while (I2C_GetFlagStatus(I2C1, flag) != status)
    {
        if ((timeout--) == 0)
            return I2C_ERR_TIMEOUT;
    }
    return I2C_OK;
}

void I2C1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    I2C_DeInit(I2C1);                 /* 先复位外设，清除上次运行遗留的挂起状态 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* PB6=SCL, PB7=SDA：复用开漏 + 上拉（I2C总线必须开漏） */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitStructure.I2C_ClockSpeed          = 100000;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
}

/* 向从机寄存器连续写数据：START -> 地址(W) -> 寄存器地址 -> 数据... -> STOP */
uint8_t I2C1_WriteBytes(uint8_t slaveAddr, uint8_t regAddr, const uint8_t *pData, uint16_t len)
{
    uint16_t i;
    uint8_t err;

    /* 等待总线空闲 */
    err = I2C_WaitFlag(I2C_FLAG_BUSY, RESET);
    if (err) return err;

    I2C_GenerateSTART(I2C1, ENABLE);
    err = I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);          /* EV5 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);
    err = I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED); /* EV6 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    I2C_SendData(I2C1, regAddr);                                /* 目标寄存器地址 */
    err = I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);     /* EV8_2 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    for (i = 0; i < len; i++)
    {
        I2C_SendData(I2C1, pData[i]);
        err = I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED); /* EV8_2 */
        if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return I2C_OK;
}

/* 从从机寄存器连续读数据：START -> 地址(W) -> 寄存器地址 -> 重复START -> 地址(R) -> 数据... -> STOP */
uint8_t I2C1_ReadBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t len)
{
    uint16_t i;
    uint8_t err;

    err = I2C_WaitFlag(I2C_FLAG_BUSY, RESET);
    if (err) return err;

    I2C_GenerateSTART(I2C1, ENABLE);
    err = I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);          /* EV5 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);
    err = I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED); /* EV6 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    I2C_SendData(I2C1, regAddr);
    err = I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);     /* EV8_2 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    /* 重复START，转为接收方向 */
    I2C_GenerateSTART(I2C1, ENABLE);
    err = I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);          /* EV5 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Receiver);
    err = I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED); /* EV6 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    for (i = 0; i < len; i++)
    {
        if (i == len - 1)
        {
            /* 最后一个字节前关应答，读完后发STOP */
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
        }
        err = I2C_WaitFlag(I2C_FLAG_RXNE, SET);                 /* 等待收到字节 */
        if (err)
        {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
            I2C_AcknowledgeConfig(I2C1, ENABLE);                /* 恢复应答，供下次传输 */
            return err;
        }
        pData[i] = I2C_ReceiveData(I2C1);
    }

    I2C_AcknowledgeConfig(I2C1, ENABLE);                        /* 恢复应答 */
    return I2C_OK;
}

/* 从从机指定16位寄存器（字）地址连续读数据（AT24C32等双字节地址EEPROM专用）
 * 写字地址高、低两字节后重复START转接收，时序同I2C1_ReadBytes */
uint8_t I2C1_ReadAddr16(uint8_t slaveAddr, uint16_t regAddr, uint8_t *pData, uint16_t len)
{
    uint16_t i;
    uint8_t err;

    err = I2C_WaitFlag(I2C_FLAG_BUSY, RESET);
    if (err) return err;

    I2C_GenerateSTART(I2C1, ENABLE);
    err = I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);          /* EV5 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);
    err = I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED); /* EV6 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    I2C_SendData(I2C1, (uint8_t)(regAddr >> 8));                /* 字地址高字节 */
    err = I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);     /* EV8_2 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    I2C_SendData(I2C1, (uint8_t)regAddr);                       /* 字地址低字节 */
    err = I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);     /* EV8_2 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    /* 重复START，转为接收方向 */
    I2C_GenerateSTART(I2C1, ENABLE);
    err = I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);          /* EV5 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Receiver);
    err = I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED); /* EV6 */
    if (err) { I2C_GenerateSTOP(I2C1, ENABLE); return err; }

    for (i = 0; i < len; i++)
    {
        if (i == len - 1)
        {
            /* 最后一个字节前关应答，读完后发STOP */
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
        }
        err = I2C_WaitFlag(I2C_FLAG_RXNE, SET);                 /* 等待收到字节 */
        if (err)
        {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
            I2C_AcknowledgeConfig(I2C1, ENABLE);                /* 恢复应答，供下次传输 */
            return err;
        }
        pData[i] = I2C_ReceiveData(I2C1);
    }

    I2C_AcknowledgeConfig(I2C1, ENABLE);                        /* 恢复应答 */
    return I2C_OK;
}

/* 总线探针：只发地址不做寄存器操作，用于硬件诊断
   返回1=从机应答(ADDR置位)，0=无应答；失败原因打印到串口 */
uint8_t I2C1_ProbeAddr(uint8_t addr8)
{
    uint32_t timeout = I2C_TIMEOUT_CNT;
    uint8_t  ack;

    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET)
    {
        if ((timeout--) == 0)
        {
            printf("I2C probe: BUSY stuck\r\n");
            return 0;
        }
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_TIMEOUT_CNT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
    {
        if ((timeout--) == 0)
        {
            printf("I2C probe: START timeout\r\n");
            I2C_GenerateSTOP(I2C1, ENABLE);
            return 0;
        }
    }

    I2C_Send7bitAddress(I2C1, addr8, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT_CNT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) == RESET &&
           I2C_GetFlagStatus(I2C1, I2C_FLAG_AF) == RESET)
    {
        if ((timeout--) == 0)
        {
            printf("I2C probe: addr phase timeout\r\n");
            I2C_GenerateSTOP(I2C1, ENABLE);
            return 0;
        }
    }

    ack = (I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) != RESET) ? 1 : 0;
    I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    I2C_GenerateSTOP(I2C1, ENABLE);
    return ack;
}
