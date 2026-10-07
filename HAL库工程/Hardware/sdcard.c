#include "sdcard.h"
#include "delay.h"

/*============================================================================
  MicroSD 卡 SPI 模式驱动 —— 软件 SPI 专用总线版（HAL 工程版，传输层用 HAL GPIO，协议层与 SPL 版一致）
  （接线与依赖见 sdcard.h 顶部说明；协议层与硬件 SPI 版完全一致）
  - 初始化流程（SD 规范上电序列）：CS 高 + ≥74 空时钟 → CMD0 进 IDLE → CMD8 判卡版本
    → ACMD41(带HCS) 等卡就绪（超时再试 CMD1 兜 MMC）→ CMD58 读 OCR 定寻址方式
    → CMD16 定块长 512 → 提速到 ~2MHz 正常读写
  - SDHC/SDXC（>2GB，本卡 8GB）按 LBA 块寻址，CMD17/24 参数即扇区号；
    SDv1/MMC 按字节寻址，参数 = LBA*512
  - 读写按单块命令循环实现：2MHz 下 512B 一块 ≈3ms，2s 采样节奏完全够用，
    换 CMD18/25 多块命令收益小且增加出错面
  ============================================================*/

/* 命令字（SPI 模式） */
#define CMD0    0     /* GO_IDLE_STATE      */
#define CMD1    1     /* SEND_OP_COND (MMC) */
#define CMD8    8     /* SEND_IF_COND（判 SDv2）*/
#define CMD9    9     /* SEND_CSD           */
#define CMD16   16    /* SET_BLOCKLEN       */
#define CMD17   17    /* READ_SINGLE_BLOCK  */
#define CMD24   24    /* WRITE_BLOCK        */
#define CMD55   55    /* APP_CMD（ACMD 前导）*/
#define CMD58   58    /* READ_OCR           */
#define ACMD41  41    /* SD_SEND_OP_COND    */

#define SD_BLOCK_SIZE     512UL
#define DATA_TOKEN_BLOCK  0xFE    /* 单块读/写的数据起始令牌 */
#define DATA_RESP_ACCEPT  0x05    /* 数据响应 x5 位 = 010 = 接受 */

#define TIMEOUT_CMD0      500UL   /* ms，CMD0 进入 IDLE */
#define TIMEOUT_ACMD41    1000UL  /* ms，卡内部初始化（规范上限 1s） */
#define TIMEOUT_CMD1      500UL   /* ms，MMC 兜底 */
#define TIMEOUT_TOKEN     300UL   /* ms，等数据令牌 */
#define TIMEOUT_BUSY      1000UL  /* ms，等写完成（卡内部搬运可能数百 ms） */
#define CMD0_CRC          0x95    /* CMD0 的固定 CRC（进入 SPI 模式的唯一要求） */
#define CMD8_CRC          0x87    /* CMD8(0x1AA) 的固定 CRC */
#define CMD_DUMMY_CRC     0x01    /* SPI 模式默认 CRC 关闭，其余命令 CRC 位填 1 */

/*---------------- 软件 SPI 专用总线（GPIOC：CS=PC6/SCK=PC7/MISO=PC8/MOSI=PC9） ----------------*/

#define SD_CS_PIN         GPIO_PIN_6
#define SD_SCK_PIN        GPIO_PIN_7
#define SD_MISO_PIN       GPIO_PIN_8
#define SD_MOSI_PIN       GPIO_PIN_9

#define SD_CS_SET()       (HAL_GPIO_WritePin(GPIOC, SD_CS_PIN, GPIO_PIN_SET))
#define SD_CS_CLR()       (HAL_GPIO_WritePin(GPIOC, SD_CS_PIN, GPIO_PIN_RESET))
#define SD_SCK_SET()      (HAL_GPIO_WritePin(GPIOC, SD_SCK_PIN, GPIO_PIN_SET))
#define SD_SCK_CLR()      (HAL_GPIO_WritePin(GPIOC, SD_SCK_PIN, GPIO_PIN_RESET))
#define SD_MOSI_SET()     (HAL_GPIO_WritePin(GPIOC, SD_MOSI_PIN, GPIO_PIN_SET))
#define SD_MOSI_CLR()     (HAL_GPIO_WritePin(GPIOC, SD_MOSI_PIN, GPIO_PIN_RESET))
#define SD_MISO_READ()    (HAL_GPIO_ReadPin(GPIOC, SD_MISO_PIN))

/* 半位周期（DWT CYCCNT 数）：Init 期 210≈400kHz（SD 规范识别期上限），
 * 正常读写 42≈2MHz（杜邦线+位拍的保守值；需求 2s/条，绰绰有余） */
#define SD_CYC_INIT       210UL
#define SD_CYC_FAST       42UL

/* 卡状态：是否块寻址（SDHC=1）、总扇区数 */
static uint8_t  s_IsHC = 0;
static uint32_t s_SectorCount = 0;
static uint32_t s_HalfBitCyc = SD_CYC_INIT;

/*---------------- 底层：CS / 速率 / 收发 ----------------*/

static void SD_CS_LOW(void)  { SD_CS_CLR(); }
static void SD_CS_HIGH(void) { SD_CS_SET(); }

static void SD_HalfBit(void)
{
    uint32_t t0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - t0) < s_HalfBitCyc) { ; }
}

/* 切速率：给半个 bit 的 CYCCNT 周期数 */
static void SD_SetBusSpeed(uint32_t HalfCyc)
{
    s_HalfBitCyc = HalfCyc;
}

/* 模式0（SCK 空闲低、上升沿采样/下降沿移位）全双工一个字节，MSB 先行 */
static uint8_t SD_RW(uint8_t Tx)
{
    uint8_t rx = 0;
    int8_t  i;
    for (i = 7; i >= 0; i--)
    {
        if (Tx & (1u << i)) { SD_MOSI_SET(); } else { SD_MOSI_CLR(); }
        SD_HalfBit();
        SD_SCK_SET();                    /* 上升沿：卡采样 DI / 移出 DO */
        SD_HalfBit();
        if (SD_MISO_READ()) { rx |= (1u << i); }
        SD_SCK_CLR();                    /* 下降沿 */
    }
    return rx;
}

static void SD_Tx(const uint8_t *Buf, uint16_t Len)
{
    while (Len--) { (void)SD_RW(*Buf++); }
}

static void SD_Rx(uint8_t *Buf, uint16_t Len)
{
    while (Len--) { *Buf++ = SD_RW(0xFF); }
}

/* 发命令：[cmd|0x40][arg32][crc][补一个0xFF]。返回前 CS 与响应读取由调用方控制 */
static void SD_SendCmdFrame(uint8_t Cmd, uint32_t Arg, uint8_t Crc)
{
    uint8_t buf[6];
    buf[0] = (uint8_t)(Cmd | 0x40);
    buf[1] = (uint8_t)(Arg >> 24);
    buf[2] = (uint8_t)(Arg >> 16);
    buf[3] = (uint8_t)(Arg >> 8);
    buf[4] = (uint8_t)(Arg);
    buf[5] = Crc;
    SD_Tx(buf, 6);
    (void)SD_RW(0xFF);   /* 命令帧后补一个时钟字节，部分卡要求 */
}

/* 读 R1 应答（最高位 0 = 有效），最多等 8 个字节；0xFF = 无应答 */
static uint8_t SD_GetR1(void)
{
    uint8_t i, r1;
    for (i = 0; i < 8; i++)
    {
        r1 = SD_RW(0xFF);
        if ((r1 & 0x80) == 0) { return r1; }
    }
    return 0xFF;
}

/* 发命令并取 R1（CS 包在内） */
static uint8_t SD_Cmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc)
{
    uint8_t r1;
    SD_CS_LOW();
    SD_SendCmdFrame(Cmd, Arg, Crc);
    r1 = SD_GetR1();
    return r1;
}

/* 等数据起始令牌（读命令后 / CSD 后），TimeoutMs 内见 token 返回 OK */
static uint8_t SD_WaitToken(uint32_t TimeoutMs)
{
    uint32_t t0 = GetTick();
    uint8_t  b;
    do
    {
        b = SD_RW(0xFF);
        if (b == DATA_TOKEN_BLOCK) { return SD_OK; }
    } while ((GetTick() - t0) < TimeoutMs);
    return SD_ERR_TIMEOUT;
}

/* 等卡内部写完成：DO 拉低表示忙，回 0xFF 即空闲 */
static uint8_t SD_WaitNotBusy(uint32_t TimeoutMs)
{
    uint32_t t0 = GetTick();
    do
    {
        if (SD_RW(0xFF) == 0xFF) { return SD_OK; }
    } while ((GetTick() - t0) < TimeoutMs);
    return SD_ERR_TIMEOUT;
}

/*============================================================
  对外接口
  ============================================================*/

uint32_t SD_GetSectorCount(void) { return s_SectorCount; }

uint8_t SD_Init(void)
{
    uint8_t  r1, ocr[4], i, isV2 = 0;
    uint32_t t0;
    GPIO_InitTypeDef gpio = {0};
    uint8_t  csd[16];
    uint32_t csize;

    /* 四线初始化：CS/SCK/MOSI 推挽输出（SCK 空闲低=模式0），MISO 输入上拉
     * （卡未驱动时读 1，便于与"被拉死"区分）。CS 默认高=不选中 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin   = SD_CS_PIN | SD_SCK_PIN | SD_MOSI_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin   = SD_MISO_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &gpio);
    SD_CS_SET();
    SD_SCK_CLR();

    /* 使能 DWT 周期计数器（半位延时用；与 AT24C64 软 I2C 同源手段，重复使能无害） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    s_IsHC = 0;
    s_SectorCount = 0;
    SD_SetBusSpeed(SD_CYC_INIT);

    /* 1) CS 高 + ≥74 个空时钟：卡内部上电同步（DI 保持高，即发 0xFF） */
    for (i = 0; i < 10; i++) { (void)SD_RW(0xFF); }

    /* 2) CMD0 进 IDLE：部分冷卡首帧吞掉，重试窗口 500ms */
    t0 = GetTick();
    r1 = 0xFF;
    while ((GetTick() - t0) < TIMEOUT_CMD0)
    {
        r1 = SD_Cmd(CMD0, 0, CMD0_CRC);
        if (r1 == 0x01) { break; }
        SD_CS_HIGH();
    }
    SD_CS_HIGH();
    if (r1 != 0x01) { return SD_ERR_NO_CARD; }   /* 无应答=没插卡/接线/供电 */

    /* 3) CMD8(0x1AA)：有电压窗应答 = SDv2；R1=0x05(非法命令) = SDv1/MMC */
    SD_CS_LOW();
    SD_SendCmdFrame(CMD8, 0x1AA, CMD8_CRC);
    r1 = SD_GetR1();
    if (r1 == 0x01)
    {
        SD_Rx(ocr, 4);                           /* R7 后 4 字节回显 */
        if (ocr[2] == 0x01 && ocr[3] == 0xAA) { isV2 = 1; }
    }
    SD_CS_HIGH();

    /* 4) ACMD41 等卡完成内部初始化（SDv2 带 HCS 位；MMC 用 CMD1 兜底）。
     *    CMD55→ACMD41 之间 CS 保持拉低（ACMD 必须紧跟 APP_CMD 且卡连续选中） */
    t0 = GetTick();
    r1 = 0xFF;
    while ((GetTick() - t0) < TIMEOUT_ACMD41)
    {
        r1 = SD_Cmd(CMD55, 0, CMD_DUMMY_CRC);    /* ACMD 前导 */
        if (r1 > 0x01) { break; }                /* 不支持 ACMD → 走 MMC 兜底 */
        r1 = SD_Cmd(ACMD41, isV2 ? (1UL << 30) : 0, CMD_DUMMY_CRC);
        SD_CS_HIGH();
        if (r1 == 0x00) { break; }
    }
    SD_CS_HIGH();
    if (r1 != 0x00)
    {
        t0 = GetTick();
        while ((GetTick() - t0) < TIMEOUT_CMD1)
        {
            r1 = SD_Cmd(CMD1, 0, CMD_DUMMY_CRC);
            SD_CS_HIGH();
            if (r1 == 0x00) { break; }
        }
        if (r1 != 0x00) { return SD_ERR_TIMEOUT; }
    }

    /* 5) CMD58 读 OCR：CCS(bit30)=1 → SDHC 块寻址（v2 专用判断） */
    if (isV2)
    {
        r1 = SD_Cmd(CMD58, 0, CMD_DUMMY_CRC);
        if (r1 != 0x00) { SD_CS_HIGH(); return SD_ERR_RW; }
        SD_Rx(ocr, 4);
        SD_CS_HIGH();
        s_IsHC = (ocr[0] & 0x40) ? 1 : 0;
    }

    /* 6) CMD16 定块长 512（SDHC 访问固定 512B，此命令对其无害） */
    if (SD_Cmd(CMD16, SD_BLOCK_SIZE, CMD_DUMMY_CRC) != 0x00)
    {
        SD_CS_HIGH();
        return SD_ERR_RW;
    }
    SD_CS_HIGH();

    /* 7) CMD9 读 CSD 取容量（判据先于提速——失败也按错误返回） */
    r1 = SD_Cmd(CMD9, 0, CMD_DUMMY_CRC);
    if (r1 != 0x00) { SD_CS_HIGH(); return SD_ERR_RW; }
    if (SD_WaitToken(TIMEOUT_TOKEN) != SD_OK) { SD_CS_HIGH(); return SD_ERR_TIMEOUT; }
    SD_Rx(csd, 16);
    (void)SD_RW(0xFF);                           /* CRC16 两字节 */
    (void)SD_RW(0xFF);
    SD_CS_HIGH();

    if ((csd[0] >> 6) == 1)                      /* CSD v2.0（SDHC/SDXC） */
    {
        csize = ((uint32_t)(csd[7] & 0x3F) << 16) | ((uint32_t)csd[8] << 8) | csd[9];
        s_SectorCount = (csize + 1) * 1024;      /* (C_SIZE+1)*512KB / 512B */
    }
    else                                         /* CSD v1.0（SDv1/MMC） */
    {
        uint32_t blLen = csd[5] & 0x0F;
        uint32_t mult  = ((csd[9] & 0x03) << 1) | (csd[10] >> 7);
        csize = ((uint32_t)(csd[6] & 0x03) << 10) | ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
        if (blLen < 9 || csize == 0) { return SD_ERR_RW; }
        s_SectorCount = (csize + 1) << (mult + 2 + blLen - 9);
    }

    SD_SetBusSpeed(SD_CYC_FAST);
    return SD_OK;
}

/* LBA -> 命令参数：SDHC 直接给扇区号，v1/MMC 给字节地址 */
static uint32_t SD_ArgFromLba(uint32_t Lba)
{
    return s_IsHC ? Lba : (Lba * SD_BLOCK_SIZE);
}

uint8_t SD_ReadSectors(uint32_t Lba, uint8_t *Buf, uint32_t Cnt)
{
    uint32_t n;
    uint8_t  r1;

    if (Buf == 0 || Cnt == 0 || s_SectorCount == 0 || Lba + Cnt > s_SectorCount)
    {
        return SD_ERR_PARAM;
    }

    for (n = 0; n < Cnt; n++)
    {
        r1 = SD_Cmd(CMD17, SD_ArgFromLba(Lba + n), CMD_DUMMY_CRC);
        if (r1 != 0x00) { SD_CS_HIGH(); return SD_ERR_RW; }
        if (SD_WaitToken(TIMEOUT_TOKEN) != SD_OK) { SD_CS_HIGH(); return SD_ERR_TIMEOUT; }
        SD_Rx(Buf + n * SD_BLOCK_SIZE, SD_BLOCK_SIZE);
        (void)SD_RW(0xFF);                       /* CRC16 两字节（CRC 已关，丢弃） */
        (void)SD_RW(0xFF);
        SD_CS_HIGH();
    }
    return SD_OK;
}

uint8_t SD_WriteSectors(uint32_t Lba, const uint8_t *Buf, uint32_t Cnt)
{
    uint32_t n;
    uint8_t  r1, resp;

    if (Buf == 0 || Cnt == 0 || s_SectorCount == 0 || Lba + Cnt > s_SectorCount)
    {
        return SD_ERR_PARAM;
    }

    for (n = 0; n < Cnt; n++)
    {
        r1 = SD_Cmd(CMD24, SD_ArgFromLba(Lba + n), CMD_DUMMY_CRC);
        if (r1 != 0x00) { SD_CS_HIGH(); return SD_ERR_RW; }

        (void)SD_RW(0xFF);                       /* 令牌前至少 1 字节间隔 */
        SD_RW(DATA_TOKEN_BLOCK);
        SD_Tx(Buf + n * SD_BLOCK_SIZE, SD_BLOCK_SIZE);
        (void)SD_RW(0xFF);                       /* CRC16 两字节（伪值） */
        (void)SD_RW(0xFF);

        resp = SD_RW(0xFF);                      /* 数据响应令牌 */
        if ((resp & 0x1F) != DATA_RESP_ACCEPT)
        {
            SD_CS_HIGH();
            return SD_ERR_RW;
        }
        if (SD_WaitNotBusy(TIMEOUT_BUSY) != SD_OK) { SD_CS_HIGH(); return SD_ERR_TIMEOUT; }
        SD_CS_HIGH();
    }
    return SD_OK;
}
