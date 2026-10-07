#include "w25q64.h"
#include "spi2.h"
#include "delay.h"

/*============================================================
  W25Q64 SPI NOR Flash 驱动（硬件 SPI2 @ 5.25MHz，模式0，CS=PB12 软件控制）
  - 移植自 F1 版 OTA 工程驱动，改动点：SPI1->SPI2、F1 GPIO->F4 GPIO、时钟使能换 AHB1/APB1
  - 擦除是写的前置条件（NOR 只能 1->0）；编程按 256B 页进行、不能跨页
  - 忙等待用 GetTick() 判超时，依赖 delay 模块的 SysTick 已初始化
  ============================================================*/

#define CMD_JEDEC_ID      0x9F
#define CMD_WRITE_EN      0x06
#define CMD_READ_SR1      0x05
#define CMD_READ_DATA     0x03
#define CMD_PAGE_PROG     0x02
#define CMD_SECTOR_ERASE  0x20
#define CMD_CHIP_ERASE    0xC7

#define SR1_BUSY          0x01

#define TIMEOUT_PROGRAM   10
#define TIMEOUT_SECTOR    1000
#define TIMEOUT_CHIP      300000UL

/* 由JEDEC ID自动识别的实际容量（W25Q80/16/32/64/128同一驱动协议，仅容量不同） */
static uint32_t s_FlashSize = 0x800000UL;   /* 默认按W25Q64的8MB，Init时按实际ID修正 */

/*---------------- 底层：CS 与 SPI 收发 ----------------*/

static void W25_CS_LOW(void)  { GPIO_ResetBits(W25Q64_CS_GPIO, W25Q64_CS_PIN); }
static void W25_CS_HIGH(void) { GPIO_SetBits(W25Q64_CS_GPIO, W25Q64_CS_PIN);   }

/* 发送一段数据（逐字节全双工，丢弃返回） */
static void W25_Tx(const uint8_t *Data, uint16_t Len)
{
    uint16_t i;
    for (i = 0; i < Len; i++)
    {
        (void)SPI2_RW(Data[i]);
    }
}

/* 接收一段数据：主机必须发字节才能移出从机数据，故发 0xFF */
static void W25_Rx(uint8_t *Data, uint16_t Len)
{
    uint16_t i;
    for (i = 0; i < Len; i++)
    {
        Data[i] = SPI2_RW(0xFF);
    }
}

/* 轮询状态寄存器BUSY位，TimeoutMs内未空闲返回超时 */
static uint8_t W25_WaitBusy(uint32_t TimeoutMs)
{
    uint8_t cmd = CMD_READ_SR1;
    uint8_t sr1;
    uint32_t t0 = GetTick();

    do
    {
        W25_CS_LOW();
        W25_Tx(&cmd, 1);
        W25_Rx(&sr1, 1);
        W25_CS_HIGH();

        if ((sr1 & SR1_BUSY) == 0)
        {
            return W25Q64_OK;
        }
    } while ((GetTick() - t0) < TimeoutMs);

    return W25Q64_ERR_TIMEOUT;
}

/* 排查探针用：读 SR1 原始值（bit1=WEL 写使能锁存，bit0=BUSY） */
uint8_t W25Q64_ReadSR1(uint8_t *Sr1)
{
    uint8_t cmd = CMD_READ_SR1;

    if (Sr1 == 0)
    {
        return W25Q64_ERR_PARAM;
    }
    W25_CS_LOW();
    W25_Tx(&cmd, 1);
    W25_Rx(Sr1, 1);
    W25_CS_HIGH();
    return W25Q64_OK;
}

static uint8_t W25_WriteEnable(void)
{
    uint8_t cmd = CMD_WRITE_EN;
    W25_CS_LOW();
    W25_Tx(&cmd, 1);
    W25_CS_HIGH();
    return W25Q64_OK;
}

/* 发送"命令 + 24位地址"，地址 MSB 先行 */
static void W25_SendAddrCmd(uint8_t Cmd, uint32_t Addr)
{
    uint8_t buf[4];
    buf[0] = Cmd;
    buf[1] = (uint8_t)(Addr >> 16);
    buf[2] = (uint8_t)(Addr >> 8);
    buf[3] = (uint8_t)(Addr);
    W25_Tx(buf, 4);
}

/*============================================================
  对外接口
  ============================================================*/

uint8_t W25Q64_ReadID(uint32_t *Id)
{
    uint8_t cmd = CMD_JEDEC_ID;
    uint8_t rsp[3] = {0};

    if (Id == 0)
    {
        return W25Q64_ERR_PARAM;
    }

    W25_CS_LOW();
    W25_Tx(&cmd, 1);
    W25_Rx(rsp, 3);
    W25_CS_HIGH();

    *Id = ((uint32_t)rsp[0] << 16) | ((uint32_t)rsp[1] << 8) | rsp[2];
    return W25Q64_OK;
}

uint8_t W25Q64_Init(void)
{
    uint32_t id;
    uint8_t  cmd = 0xAB;       /* 退出掉电模式 / 读取设备 ID */
    uint8_t  dummy[3];
    GPIO_InitTypeDef gpio = {0};

    /* 初始化CS引脚：PB12推挽输出，默认拉高（不选中） */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin   = W25Q64_CS_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(W25Q64_CS_GPIO, &gpio);
    W25_CS_HIGH();

    SPI2_Init();

    /* 发Release Power-down，顺带退出可能的深睡状态 */
    W25_CS_LOW();
    W25_Tx(&cmd, 1);
    W25_Rx(dummy, 3);
    W25_CS_HIGH();

    if (W25Q64_ReadID(&id) != W25Q64_OK)
    {
        return W25Q64_ERR_SPI;
    }

    /* 家族ID识别：协议相同，仅容量不同，按实际芯片设置容量上限 */
    switch (id)
    {
        case 0xEF4014UL: s_FlashSize = 0x100000UL;  break;   /* W25Q80  1MB */
        case 0xEF4015UL: s_FlashSize = 0x200000UL;  break;   /* W25Q16  2MB */
        case 0xEF4016UL: s_FlashSize = 0x400000UL;  break;   /* W25Q32  4MB */
        case 0xEF4017UL: s_FlashSize = 0x800000UL;  break;   /* W25Q64  8MB */
        case 0xEF4018UL: s_FlashSize = 0x1000000UL; break;   /* W25Q128 16MB */
        default:
            return W25Q64_ERR_ID;
    }
    return W25Q64_OK;
}

uint8_t W25Q64_Read(uint32_t Addr, uint8_t *Buf, uint32_t Len)
{
    if (Addr >= s_FlashSize || Len > s_FlashSize - Addr || Buf == 0)
    {
        return W25Q64_ERR_PARAM;
    }
    if (Len == 0)
    {
        return W25Q64_OK;
    }

    W25_CS_LOW();
    W25_SendAddrCmd(CMD_READ_DATA, Addr);
    W25_Rx(Buf, (uint16_t)Len);          /* 连续读自动越页 */
    W25_CS_HIGH();

    return W25Q64_OK;
}

uint8_t W25Q64_PageProgram(uint32_t Addr, const uint8_t *Buf, uint16_t Len)
{
    if (Addr >= s_FlashSize || Buf == 0)
    {
        return W25Q64_ERR_PARAM;
    }
    if (Len == 0)
    {
        return W25Q64_OK;
    }
    if (Len > W25Q64_PAGE_SIZE ||
        (Addr % W25Q64_PAGE_SIZE) + Len > W25Q64_PAGE_SIZE ||
        Addr + Len > s_FlashSize)
    {
        return W25Q64_ERR_PARAM;
    }

    if (W25_WriteEnable() != W25Q64_OK)
    {
        return W25Q64_ERR_SPI;
    }

    W25_CS_LOW();
    W25_SendAddrCmd(CMD_PAGE_PROG, Addr);
    W25_Tx(Buf, Len);
    W25_CS_HIGH();

    return W25_WaitBusy(TIMEOUT_PROGRAM);
}

uint8_t W25Q64_Write(uint32_t Addr, const uint8_t *Buf, uint32_t Len)
{
    uint32_t remain, pageRemain, chunk;

    if (Addr >= s_FlashSize || Len > s_FlashSize - Addr || Buf == 0)
    {
        return W25Q64_ERR_PARAM;
    }

    remain = Len;
    while (remain > 0)
    {
        pageRemain = W25Q64_PAGE_SIZE - (uint16_t)(Addr % W25Q64_PAGE_SIZE);
        chunk = (remain < pageRemain) ? remain : pageRemain;

        if (W25Q64_PageProgram(Addr, Buf, (uint16_t)chunk) != W25Q64_OK)
        {
            return W25Q64_ERR_SPI;
        }

        Addr   += chunk;
        Buf    += chunk;
        remain -= chunk;
    }
    return W25Q64_OK;
}

uint8_t W25Q64_EraseSector(uint32_t Addr)
{
    if (Addr >= s_FlashSize)
    {
        return W25Q64_ERR_PARAM;
    }

    if (W25_WriteEnable() != W25Q64_OK)
    {
        return W25Q64_ERR_SPI;
    }

    W25_CS_LOW();
    W25_SendAddrCmd(CMD_SECTOR_ERASE, Addr);
    W25_CS_HIGH();

    return W25_WaitBusy(TIMEOUT_SECTOR);
}

uint8_t W25Q64_ChipErase(void)
{
    uint8_t cmd = CMD_CHIP_ERASE;

    if (W25_WriteEnable() != W25Q64_OK)
    {
        return W25Q64_ERR_SPI;
    }

    W25_CS_LOW();
    W25_Tx(&cmd, 1);
    W25_CS_HIGH();

    return W25_WaitBusy(TIMEOUT_CHIP);
}
