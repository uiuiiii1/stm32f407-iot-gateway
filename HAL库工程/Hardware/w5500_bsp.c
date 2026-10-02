#include "w5500_bsp.h"
#include "wizchip_conf.h"
#include "delay.h"
#include "spi.h"        /* CubeMX 生成：extern SPI_HandleTypeDef hspi1 */

/*
 * W5500(USR-ES1) 板级支持包（HAL 版）
 * 要点：
 * 1. SPI1/CS/RST 的外设与引脚初始化由 CubeMX 生成代码完成（MX_SPI1_Init/MX_GPIO_Init）
 * 2. nRST 复位时序：拉低>500us -> 释放 -> 等>=50ms（内部PLL稳定）再访问SPI
 * 3. PWDN 引脚已在硬件上接GND（正常工作态），软件不管
 */

static volatile uint32_t s_spi1Err = 0;  /* SPI1等待超时次数，>0=硬件层出现过异常 */

/* SPI1收发等待超时计数：>0说明SPI1出现过"标志位永不置位"的硬件异常 */
uint32_t W5500_BSP_SPI1Err(void) { return s_spi1Err; }

/* SPI1 全双工一字节（ioLibrary 回调用）
 * HAL 轮询收发自带超时（100ms），超时=硬件层面异常，带病返回 0xFF 与标准库版行为一致 */
static uint8_t SPI1_RW(uint8_t tx)
{
    uint8_t rx = 0xFF;

    if (HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 100) != HAL_OK)
    {
        s_spi1Err++;
        return 0xFF;
    }
    return rx;
}

static uint8_t bsp_spi_read(void)
{
    return SPI1_RW(0xFF);
}

static void bsp_spi_write(uint8_t val)
{
    (void)SPI1_RW(val);
}

static void bsp_cs_select(void)   { HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET); }
static void bsp_cs_deselect(void) { HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);   }

void W5500_BSP_Init(void)
{
    uint8_t txSize[8] = {2, 2, 2, 2, 2, 2, 2, 2};   /* 8个Socket收发缓冲各2KB（共16KB） */
    uint8_t rxSize[8] = {2, 2, 2, 2, 2, 2, 2, 2};

    bsp_cs_deselect();                       /* nSS 默认不选中 */
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);

    /* 硬件复位时序：拉低>500us -> 释放 -> 等>=50ms（PLL稳定） */
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
    Delay_ms(2);
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
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
