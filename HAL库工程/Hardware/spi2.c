#include "spi2.h"

/*
 * SPI2 主机驱动（F407，APB1 42MHz）—— HAL 版，与标准库版语义一致
 * 引脚：PB13=SCK / PB14=MISO / PB15=MOSI（AF5），片选由各器件驱动自行控制
 * 模式0（CPOL=0 / CPHA=1Edge），波特率 42MHz/8 = 5.25MHz——面包板+杜邦线的保守值，稳定后可提到 /4
 * 本总线由 W25Q64 与（未来的）SD卡共享，靠各自片选分时复用
 * 寄存器级收发（与 LCD 驱动同风格）；所有等待带超时——SPI 异常时返回 0xFF 不卡死
 */

#define SPI2_GUARD  100000UL    /* 等待标志位的自旋上限（约几十ms量级，远大于1字节时间） */

void SPI2_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB13(SCK) / PB15(MOSI)：复用推挽；PB14(MISO)：复用输入上拉
     * 上拉——从机未驱动/未接线时读1，便于与"被拉死"区分开 */
    gpio.Pin   = GPIO_PIN_13 | GPIO_PIN_15;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin  = GPIO_PIN_14;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 关键：F4必须显式指定复用功能编号（Alternate），把PB13/14/15路由到SPI2（AF5）
     * 否则AFR保持复位值AF0，引脚根本没接到SPI外设——SCK不翻转、MISO不响应 */
    {
        SPI_HandleTypeDef h = {0};
        h.Instance = SPI2;
        h.Init.Mode               = SPI_MODE_MASTER;
        h.Init.Direction          = SPI_DIRECTION_2LINES;
        h.Init.DataSize           = SPI_DATASIZE_8BIT;
        h.Init.CLKPolarity        = SPI_POLARITY_LOW;            /* 模式0 */
        h.Init.CLKPhase           = SPI_PHASE_1EDGE;
        h.Init.NSS                = SPI_NSS_SOFT;                /* 片选软件控制 */
        h.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_8;     /* 42MHz/8=5.25MHz */
        h.Init.FirstBit           = SPI_FIRSTBIT_MSB;
        h.Init.TIMode             = SPI_TIMODE_DISABLE;
        h.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
        if (HAL_SPI_Init(&h) != HAL_OK)
            return;
    SET_BIT(SPI2->CR1, SPI_CR1_SPE);   /* HAL_SPI_Init 只配置不使能，必须显式开SPE */
    }

    /* 上电后可能残留一个 RXNE，读一次清掉，避免首次收发取错数据 */
    (void)SPI2->DR;
}

uint8_t SPI2_RW(uint8_t tx)
{
    uint32_t guard = SPI2_GUARD;

    /* 等发送区空，再写（直接写8位DR避免读-改写字节序问题） */
    while ((SPI2->SR & SPI_SR_TXE) == 0U && --guard) { }
    if (guard == 0U)
        return 0xFF;                        /* SPI 异常：返回全1，不卡死调用方 */
    *(volatile uint8_t *)&SPI2->DR = tx;

    guard = SPI2_GUARD;
    while ((SPI2->SR & SPI_SR_RXNE) == 0U && --guard) { }
    if (guard == 0U)
        return 0xFF;
    return (uint8_t)SPI2->DR;
}
