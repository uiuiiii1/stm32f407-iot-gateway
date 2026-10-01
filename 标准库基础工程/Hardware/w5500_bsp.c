#include "w5500_bsp.h"
#include "wizchip_conf.h"
#include "delay.h"

/*
 * W5500(USR-ES1) 板级支持包
 * 要点：
 * 1. F4 引脚必须 GPIO_Init 与 GPIO_PinAFConfig 成对出现（AF5=SPI1）
 * 2. nRST 复位时序：拉低>500us -> 释放 -> 等>=50ms（内部PLL稳定）再访问SPI
 * 3. PWDN 引脚已在硬件上接GND（正常工作态），软件不管
 */

#define W5500_CS_PORT   GPIOC
#define W5500_CS_PIN    GPIO_Pin_4
#define W5500_RST_PORT  GPIOC
#define W5500_RST_PIN   GPIO_Pin_5

static volatile uint32_t s_spi1Err = 0;  /* SPI1等待超时次数，>0=硬件层出现过异常 */

/* SPI1收发等待超时计数：>0说明SPI1出现过"标志位永不置位"的硬件异常 */
uint32_t W5500_BSP_SPI1Err(void) { return s_spi1Err; }

/* SPI1 全双工一字节（ioLibrary 回调用） */
static uint8_t SPI1_RW(uint8_t tx)
{
    uint32_t t = 100000;

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
        if ((t--) == 0) { s_spi1Err++; return 0xFF; }   /* 超时：硬件层面异常，带病返回 */
    }
    SPI_I2S_SendData(SPI1, tx);
    t = 100000;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if ((t--) == 0) { s_spi1Err++; return 0xFF; }
    }
    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

static uint8_t bsp_spi_read(void)
{
    return SPI1_RW(0xFF);
}

static void bsp_spi_write(uint8_t val)
{
    (void)SPI1_RW(val);
}

static void bsp_cs_select(void)   { GPIO_ResetBits(W5500_CS_PORT, W5500_CS_PIN); }
static void bsp_cs_deselect(void) { GPIO_SetBits(W5500_CS_PORT, W5500_CS_PIN);   }

void W5500_BSP_Init(void)
{
    GPIO_InitTypeDef  gpio = {0};
    SPI_InitTypeDef   spi  = {0};
    uint8_t txSize[8] = {2, 2, 2, 2, 2, 2, 2, 2};   /* 8个Socket收发缓冲各2KB（共16KB） */
    uint8_t rxSize[8] = {2, 2, 2, 2, 2, 2, 2, 2};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

    /* PA5(SCK)/PA7(MOSI)：复用推挽；PA6(MISO)：复用输入 */
    gpio.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_6;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    /* ★ F4必须显式路由复用功能（AF5=SPI1），漏了引脚就不通 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    /* PC4=nSS（默认不选中），PC5=nRST（默认释放） */
    gpio.GPIO_Pin   = W5500_CS_PIN | W5500_RST_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(W5500_CS_PORT, &gpio);
    bsp_cs_deselect();
    GPIO_SetBits(W5500_RST_PORT, W5500_RST_PIN);

    spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode              = SPI_Mode_Master;
    spi.SPI_DataSize          = SPI_DataSize_8b;
    spi.SPI_CPOL              = SPI_CPOL_Low;              /* 模式0（W5500支持0~3） */
    spi.SPI_CPHA              = SPI_CPHA_1Edge;
    spi.SPI_NSS               = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  /* 84MHz/64=1.3MHz，调试期低速；稳定后改 /16 */
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
    (void)SPI_I2S_ReceiveData(SPI1);                       /* 清残留RXNE */

    /* 硬件复位时序：拉低>500us -> 释放 -> 等>=50ms（PLL稳定） */
    GPIO_ResetBits(W5500_RST_PORT, W5500_RST_PIN);
    Delay_ms(2);
    GPIO_SetBits(W5500_RST_PORT, W5500_RST_PIN);
    Delay_ms(50);

    /* 注册ioLibrary回调并分配Socket缓冲 */
    reg_wizchip_cs_cbfunc(bsp_cs_select, bsp_cs_deselect);
    reg_wizchip_spi_cbfunc(bsp_spi_read, bsp_spi_write);
    wizchip_init(txSize, rxSize);
}

void W5500_NetworkInit(void)
{
    wiz_NetInfo ni = {
        .mac  = W5500_MAC,
        .ip   = W5500_IP,
        .sn   = W5500_SN,
        .gw   = W5500_GW,
        .dns  = {0, 0, 0, 0},
        .dhcp = NETINFO_STATIC
    };
    wizchip_setnetinfo(&ni);
}

uint8_t W5500_LinkStatus(void)
{
    return (wizphy_getphylink() == PHY_LINK_ON) ? 1 : 0;
}

/* 配置自愈校验（只读不改）：读回芯片的源IP寄存器（SIPR=Common块0x000F）与配置区比对。
 * 芯片被意外复位（干扰/电源抖动）会丢失全部网络配置（PHY不受影响仍显示Link）。
 * 返回1=配置完好；0=不一致（是否恢复由调用方防抖决定，避免单次坏读触发重配） */
uint8_t W5500_CheckConfig(void)
{
    uint8_t cur[4];
    const uint8_t want[4] = W5500_IP;

    WIZCHIP_READ_BUF(SIPR, cur, 4);
    if (cur[0] != want[0] || cur[1] != want[1] ||
        cur[2] != want[2] || cur[3] != want[3])
    {
        return 0;
    }
    return 1;
}

/* SPI1环回自检：拔掉模块的MISO/MOSI线、用杜邦线短接核心板PA6-PA7后调用。
 * 发送的每个字节经PA7->短接线->PA6原样收回，全部一致=SPI1外设/引脚/AF配置全好
 * tx/tx : 发送与接收缓冲（同长度）；返回1=通过，0=有回环不一致（核心板侧问题） */
uint8_t W5500_BSP_SPILoopback(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    uint8_t i, pass = 1;

    for (i = 0; i < len; i++)
    {
        rx[i] = SPI1_RW(tx[i]);
        if (rx[i] != tx[i])
            pass = 0;
    }
    return pass;
}

/* 读版本寄存器VERSIONR：Common块偏移0x0039，固定值0x04
 * ⚠️ 注意地址是0x0039不是0x0000——0x0000是模式寄存器MR（复位值0x00），
 *    曾因读错地址把"芯片正常应答MR=0x00"误判为"模块离线"
 * SPI帧格式：[地址H=0x00][地址L=0x39][控制字节(Common块<<3 | 读 | VDM=00)] */
uint8_t W5500_ReadVersion(void)
{
    uint8_t ver;

    bsp_cs_select();
    (void)SPI1_RW(0x00);                 /* 地址高字节 */
    (void)SPI1_RW(0x39);                 /* 地址低字节：VERSIONR=0x0039 */
    (void)SPI1_RW(0x00);                 /* 控制字节 */
    ver = SPI1_RW(0xFF);
    bsp_cs_deselect();
    return ver;
}
