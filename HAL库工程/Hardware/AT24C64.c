#include "AT24C64.h"

/*============================================================
  AT24C64 软件 I2C 驱动（PB10=SCL，PB11=SDA，开漏输出）
  - 开漏：写 1 = 释放总线（上拉拉高），写 0 = 拉低；读 SDA 前必须先释放。
  - DWT 周期计数器做半位延时，稳定跑 100kHz。
  - 写完后器件进入内部写周期（<=5ms），用应答轮询等待。
  - HAL 适配：GPIO 用 HAL_GPIO_WritePin/ReadPin，时钟用 __HAL_RCC_GPIOB_CLK_ENABLE。
  ============================================================*/

#define IIC_HALF_BIT_US   (1000U / AT24C64_I2C_FREQ_KHZ / 2U)

static void IIC_Delay(void)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = IIC_HALF_BIT_US * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { ; }
}

static void IIC_SCL(uint8_t v)
{
    HAL_GPIO_WritePin(AT24C64_SCL_PORT, AT24C64_SCL_PIN,
                      v ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void IIC_W_SDA(uint8_t v)
{
    HAL_GPIO_WritePin(AT24C64_SDA_PORT, AT24C64_SDA_PIN,
                      v ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t IIC_R_SDA(void)
{
    return (HAL_GPIO_ReadPin(AT24C64_SDA_PORT, AT24C64_SDA_PIN) == GPIO_PIN_SET) ? 1 : 0;
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
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* 配置 SCL/SDA 引脚：开漏 + 上拉（HAL：GPIOB 时钟 AHB1） */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin   = AT24C64_SCL_PIN | AT24C64_SDA_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

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
