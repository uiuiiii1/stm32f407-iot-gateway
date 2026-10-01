#include "spi2.h"

/*
 * SPI2 主机驱动（F407，APB1 42MHz）
 * 引脚：PB13=SCK / PB14=MISO / PB15=MOSI（AF5），片选由各器件驱动自行控制
 * 模式0（CPOL=0 / CPHA=1Edge），波特率 42MHz/8 = 5.25MHz——面包板+杜邦线的保守值，稳定后可提到 /4
 * 本总线由 W25Q64 与（未来的）SD卡共享，靠各自片选分时复用
 */

void SPI2_Init(void)
{
    GPIO_InitTypeDef  gpio = {0};
    SPI_InitTypeDef   spi  = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* PB13(SCK) / PB15(MOSI)：复用推挽 */
    gpio.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_15;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);

    /* PB14(MISO)：复用输入，上拉——从机未驱动/未接线时读1，便于与"被拉死"区分开 */
    gpio.GPIO_Pin   = GPIO_Pin_14;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);

    /* 关键：F4必须显式指定复用功能编号，把PB13/14/15路由到SPI2（AF5）
     * 否则AFR保持复位值AF0，引脚根本没接到SPI外设——SCK不翻转、MISO不响应 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

    spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode              = SPI_Mode_Master;
    spi.SPI_DataSize          = SPI_DataSize_8b;
    spi.SPI_CPOL              = SPI_CPOL_Low;             /* 模式0 */
    spi.SPI_CPHA              = SPI_CPHA_1Edge;
    spi.SPI_NSS               = SPI_NSS_Soft;             /* 片选软件控制 */
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  /* 42MHz/8=5.25MHz */
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI2, &spi);
    SPI_Cmd(SPI2, ENABLE);

    /* 上电后可能残留一个 RXNE，读一次清掉，避免首次收发取错数据 */
    (void)SPI_I2S_ReceiveData(SPI2);
}

uint8_t SPI2_RW(uint8_t tx)
{
    /* 等发送区空，再写 */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) { ; }
    SPI_I2S_SendData(SPI2, tx);
    /* 等接收区有数据 */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) { ; }
    return (uint8_t)SPI_I2S_ReceiveData(SPI2);
}
