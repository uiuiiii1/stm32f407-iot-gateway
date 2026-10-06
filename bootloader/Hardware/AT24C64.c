#include "AT24C64.h"

/*============================================================
  AT24C64 软件 I2C 驱动（PB10=SCL，PB11=SDA，开漏输出）
  - 开漏：写 1 = 释放总线（上拉拉高），写 0 = 拉低；读 SDA 前必须先释放。
  - DWT 周期计数器做半位延时，稳定跑 100kHz。
  - 写完后器件进入内部写周期（<=5ms），用应答轮询等待。
  - 移植说明：F4 的 core_cm4.h 已定义 DWT_CTRL_CYCCNTENA_Msk，
    为避免宏重定义告警，这里使用本文件私有前缀。
  ============================================================*/

/* 本文件私有定义，避免与 core_cm4.h 的 DWT_* 位宏重定义冲突 */
#define AT24C64_DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004U)
#define AT24C64_DWT_CTRL     (*(volatile uint32_t *)0xE0001000U)
#define AT24C64_DWT_ENA      (1UL << 0)

#define IIC_HALF_BIT_US   (1000U / AT24C64_I2C_FREQ_KHZ / 2U)

static void IIC_Delay(void)
{
    uint32_t start = AT24C64_DWT_CYCCNT;
    uint32_t ticks = IIC_HALF_BIT_US * (SystemCoreClock / 1000000U);
    while ((AT24C64_DWT_CYCCNT - start) < ticks) { ; }
}

static void IIC_SCL(uint8_t v)
{
    if (v) GPIO_SetBits(AT24C64_SCL_PORT, AT24C64_SCL_PIN);
    else   GPIO_ResetBits(AT24C64_SCL_PORT, AT24C64_SCL_PIN);
}

static void IIC_W_SDA(uint8_t v)
{
    if (v) GPIO_SetBits(AT24C64_SDA_PORT, AT24C64_SDA_PIN);
    else   GPIO_ResetBits(AT24C64_SDA_PORT, AT24C64_SDA_PIN);
}

static uint8_t IIC_R_SDA(void)
{
    return (GPIO_ReadInputDataBit(AT24C64_SDA_PORT, AT24C64_SDA_PIN) ? 1 : 0);
}

static void IIC_Start(void)
{
    IIC_W_SDA(1);
    IIC_SCL(1);
    IIC_Delay();
    IIC_W_SDA(0);
    IIC_Delay();
    IIC_SCL(0);
    IIC_Delay();
}

static void IIC_Stop(void)
{
    IIC_W_SDA(0);
    IIC_Delay();
    IIC_SCL(1);
    IIC_Delay();
    IIC_W_SDA(1);
    IIC_Delay();
}

/* 发送一个字节，返回应答位：0=ACK，1=NAK */
static uint8_t IIC_SendByte(uint8_t byte)
{
    uint8_t i, ack;
    for (i = 0; i < 8; i++)
    {
        IIC_SCL(0);
        IIC_W_SDA((byte & 0x80) ? 1 : 0);
        IIC_Delay();
        IIC_SCL(1);
        IIC_Delay();
        byte <<= 1;
    }
    /* 第 9 个时钟：释放 SDA，读从机应答 */
    IIC_SCL(0);
    IIC_W_SDA(1);
    IIC_Delay();
    IIC_SCL(1);
    IIC_Delay();
    ack = IIC_R_SDA();
    IIC_SCL(0);
    return ack;
}

/* 接收一个字节，ack=1 主机应答继续读，ack=0 非应答结束 */
static uint8_t IIC_ReadByte(uint8_t ack)
{
    uint8_t i, byte = 0;
    IIC_W_SDA(1);   /* 释放 SDA */
    for (i = 0; i < 8; i++)
    {
        IIC_SCL(0);
        IIC_Delay();
        IIC_SCL(1);
        IIC_Delay();
        byte <<= 1;
        if (IIC_R_SDA())
        {
            byte |= 0x01;
        }
    }
    IIC_SCL(0);
    IIC_W_SDA(ack ? 0 : 1);
    IIC_Delay();
    IIC_SCL(1);
    IIC_Delay();
    IIC_SCL(0);
    IIC_W_SDA(1);
    return byte;
}

/* 哑写设定地址，返回 AT24C64_OK / ERR_NAK */
static uint8_t AT24C64_SetAddr(uint16_t addr)
{
    IIC_Start();
    if (IIC_SendByte(AT24C64_DEV_ADDR | 0x00)) { IIC_Stop(); return AT24C64_ERR_NAK; }
    if (IIC_SendByte((uint8_t)(addr >> 8)))    { IIC_Stop(); return AT24C64_ERR_NAK; }
    if (IIC_SendByte((uint8_t)addr))           { IIC_Stop(); return AT24C64_ERR_NAK; }
    return AT24C64_OK;
}

/*============================================================
  对外接口
  ============================================================*/

uint8_t AT24C64_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 使能 DWT 周期计数器（Cortex-M4 调试组件），供 IIC_Delay 使用 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    AT24C64_DWT_CYCCNT = 0;
    AT24C64_DWT_CTRL  |= AT24C64_DWT_ENA;

    /* 配置 SCL/SDA 引脚：开漏输出（F4：GPIO 时钟在 AHB1） */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin   = AT24C64_SCL_PIN | AT24C64_SDA_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;   /* F4：输出模式 + OType 选开漏 */
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    IIC_SCL(1);
    IIC_W_SDA(1);

    return AT24C64_WaitReady();
}

uint8_t AT24C64_WaitReady(void)
{
    uint32_t retry;
    for (retry = 0; retry < 1000; retry++)
    {
        IIC_Start();
        if (IIC_SendByte(AT24C64_DEV_ADDR) == 0) { IIC_Stop(); return AT24C64_OK; }
        IIC_Stop();
        IIC_Delay();
    }
    return AT24C64_ERR_NAK;
}

uint8_t AT24C64_WriteByte(uint16_t Addr, uint8_t Data)
{
    if (Addr >= AT24C64_SIZE) return AT24C64_ERR_PARAM;
    if (AT24C64_SetAddr(Addr) != AT24C64_OK) return AT24C64_ERR_NAK;
    if (IIC_SendByte(Data)) { IIC_Stop(); return AT24C64_ERR_NAK; }
    IIC_Stop();
    return AT24C64_WaitReady();
}

uint8_t AT24C64_ReadByte(uint16_t Addr, uint8_t *Data)
{
    if (Addr >= AT24C64_SIZE || Data == 0) return AT24C64_ERR_PARAM;
    if (AT24C64_SetAddr(Addr) != AT24C64_OK) return AT24C64_ERR_NAK;

    IIC_Start();
    if (IIC_SendByte(AT24C64_DEV_ADDR | 0x01)) { IIC_Stop(); return AT24C64_ERR_NAK; }
    *Data = IIC_ReadByte(0);
    IIC_Stop();
    return AT24C64_OK;
}

uint8_t AT24C64_Write(uint16_t Addr, const uint8_t *Buf, uint16_t Len)
{
    uint16_t remain, pageRemain, chunk, i;

    if (Addr >= AT24C64_SIZE || Len > AT24C64_SIZE - Addr) return AT24C64_ERR_PARAM;

    remain = Len;
    while (remain > 0)
    {
        pageRemain = AT24C64_PAGE_SIZE - (uint16_t)(Addr % AT24C64_PAGE_SIZE);
        chunk = (remain < pageRemain) ? remain : pageRemain;

        if (AT24C64_SetAddr(Addr) != AT24C64_OK) return AT24C64_ERR_NAK;
        for (i = 0; i < chunk; i++)
        {
            if (IIC_SendByte(*Buf++)) { IIC_Stop(); return AT24C64_ERR_NAK; }
        }
        IIC_Stop();

        Addr   += chunk;
        remain -= chunk;

        if (AT24C64_WaitReady() != AT24C64_OK) return AT24C64_ERR_NAK;
    }
    return AT24C64_OK;
}

uint8_t AT24C64_Read(uint16_t Addr, uint8_t *Buf, uint16_t Len)
{
    uint16_t i;
    if (Addr >= AT24C64_SIZE || Len > AT24C64_SIZE - Addr) return AT24C64_ERR_PARAM;
    if (Len == 0) return AT24C64_OK;

    if (AT24C64_SetAddr(Addr) != AT24C64_OK) return AT24C64_ERR_NAK;

    IIC_Start();
    if (IIC_SendByte(AT24C64_DEV_ADDR | 0x01)) { IIC_Stop(); return AT24C64_ERR_NAK; }

    for (i = 0; i < Len; i++)
    {
        Buf[i] = IIC_ReadByte((i < Len - 1) ? 1 : 0);
    }
    IIC_Stop();
    return AT24C64_OK;
}
