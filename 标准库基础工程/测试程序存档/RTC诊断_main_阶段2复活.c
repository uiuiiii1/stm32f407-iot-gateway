#include "stm32f4xx.h"
#include <stdio.h>
#include "delay.h"
#include "usart.h"
#include "i2c.h"
#include "ds1307.h"

/*
 * 标准库基础工程 —— 阶段2 RTC恢复：DS1307 + AT24C32（Tiny RTC模块）诊断测试
 *
 * 模块：Tiny RTC（DS1307实时时钟 + AT24C32 EEPROM 共用一条I2C总线，
 *       模块板载I2C上拉电阻、32768Hz晶振、充电电池座）
 * 接线：模块VCC -> 核心板5V（DS1307规格电压4.5~5.5V，接3.3V属超规格运行，
 *       走时停振/无应答的头号嫌疑）；模块GND -> 核心板GND；
 *       模块SCL -> PB6；模块SDA -> PB7（PB6/PB7为FT引脚，可承受5V上拉）。
 *       P1/P2两个排针任意接一个即可；SQW、DS脚与本次测试无关，不接。
 *
 * 测试流程（上电自动运行，结果走USART1串口 115200-8-N-1）：
 *  [1] I2C总线扫描0x08~0x77：应看到 DS1307@0x68 和 AT24C32@0x50~0x57
 *  [2] DS1307：原始寄存器转储（判断CH停振/全FF全00/非法BCD）-> CH停振自动清零 ->
 *      时间合法性校验（乱值=断电时电池没兜住的直接证据）-> 3秒走时测试 -> NVRAM读写测试
 *  [3] AT24C32：单字节写读 + 32字节页写读校验（写周期用ACK轮询等待）
 *  注意：测试【不会】重写RTC时间——时间一直延续，方便观察掉电保持；
 *  需要对时/重设基准时串口发 's'（写入 2026-10-01 12:00:00）。
 *  测完进入秒级实时时钟循环；串口发 'r' 重跑全部测试，'s' 重设默认时间。
 *
 * 注意：DS1307 NVRAM测试会覆盖0x08~0x3F，AT24C32测试会覆盖0x0040和0x0080两处。
 */

#define DS1307_ADDR8     0xD0   /* DS1307 7位地址0x68左移 */
#define EE_ADDR7_MIN     0x50
#define EE_ADDR7_MAX     0x57   /* A2/A1/A0全0时AT24C32=0x50，本模块经3k3上拉可能到0x57，以扫描结果为准 */
#define EE_TEST_BYTE     0x0040 /* AT24C32单字节测试地址 */
#define EE_TEST_PAGE     0x0080 /* AT24C32页写测试起始地址（页内32字节不跨页） */

#define RTC_DEF_YEAR     26     /* 默认时间 2026-10-01 12:00:00 */
#define RTC_DEF_MON      10
#define RTC_DEF_DATE     1
#define RTC_DEF_HOUR     12
#define RTC_DEF_MIN      0
#define RTC_DEF_SEC      0

/* ~3us忙等：手动I2C解锁时产生100kHz量级的SCL脉冲 */
static void UnlockDelay(void)
{
    volatile uint32_t j;
    for (j = 0; j < 500; j++)
        __NOP();
}

/*
 * I2C总线解锁与体检：程序复位/调试打断I2C传输后，DS1307可能还钳着SDA输出低电平，
 * 之后外设BUSY永远清不掉、所有传输超时（表现为"时好时坏、复位后全挂"）。
 * SDA被钳低时手动给最多9个SCL时钟让从机放线，再补一个STOP。
 * 模块后上电属正常场景：返回2只代表总线当前被拉死（如模块未供电），
 * 调用方自动重试即可，详细诊断提示只在连续卡住的第一轮打印。
 * 返回0=总线空闲可用；2=总线仍被钳低（等待接线/供电恢复）。
 */
static uint8_t I2C_BusUnlock(void)
{
    static uint8_t hintShown = 0;      /* 详细提示只打一次，自动重试期间不刷屏 */
    GPIO_InitTypeDef g;
    uint8_t i, sda, scl, pulsed = 0;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    g.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    g.GPIO_Mode  = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_OD;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &g);

    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
    Delay_ms(2);                           /* 充分释放后再读电平 */

    sda = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
    scl = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);

    if (!sda)                              /* 只有SDA被钳低才需要解锁 */
    {
        pulsed = 1;
        for (i = 0; i < 10 && GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == Bit_RESET; i++)
        {
            GPIO_ResetBits(GPIOB, GPIO_Pin_6); /* SCL低 */
            UnlockDelay();
            GPIO_SetBits(GPIOB, GPIO_Pin_6);   /* SCL高：从机移一格，直到放掉SDA */
            UnlockDelay();
        }
        /* 手动STOP：SCL高电平期间SDA由低到高 */
        GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
        UnlockDelay();
        GPIO_SetBits(GPIOB, GPIO_Pin_6);
        UnlockDelay();
        GPIO_SetBits(GPIOB, GPIO_Pin_7);
        Delay_ms(2);
        sda = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
        scl = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
    }

    if (sda && scl)
    {
        hintShown = 0;
        printf(pulsed ? "I2C bus unlock done: bus recovered\r\n"
                      : "I2C lines idle (SDA high, SCL high): bus OK\r\n");
        return 0;
    }

    if (!hintShown)
    {
        hintShown = 1;
        printf("I2C bus dead: SDA=%s SCL=%s\r\n",
               sda ? "high" : "LOW", scl ? "high" : "LOW");
        if (!sda && !scl)
            printf("BOTH lines low -> module almost certainly UNPOWERED (its onboard\r\n"
                   "  pull-ups drag the bus to the dead VCC rail).\r\n"
                   "  Power the module (VCC->5V, P1: BAT-GND-VCC-SDA-SCL-DS-SQ) or fix wiring.\r\n"
                   "  Auto-retrying every 1s; send 'r' to retry now.\r\n");
        else if (!sda)
            printf("SDA still held low -> shorted to GND / wrong pin. Auto-retrying every 1s\r\n");
        else
            printf("SCL still held low -> shorted to GND / swapped wires. Auto-retrying every 1s\r\n");
    }
    else
        printf("bus still dead (SDA=%s SCL=%s), auto-retrying...\r\n",
               sda ? "high" : "LOW", scl ? "high" : "LOW");
    return 2;
}

/* 总线扫描：返回应答的7位地址个数，存入found（最多maxCnt个） */
static uint8_t Bus_Scan(uint8_t *found, uint8_t maxCnt)
{
    uint8_t a, n = 0;

    printf("[1] I2C bus scan 0x08~0x77 (expect DS1307=0x68, AT24C32=0x50~0x57)\r\n");
    for (a = 0x08; a <= 0x77; a++)
    {
        if (I2C1_ProbeAddr((uint8_t)(a << 1)))
        {
            printf("    ACK at 7-bit 0x%02X\r\n", a);
            if (n < maxCnt)
                found[n] = a;
            n++;
        }
    }
    if (n == 0)
        printf("    NO device ACK! check: VCC->5V / GND common / SCL=PB6 / SDA=PB7 / wires swapped?\r\n");
    return n;
}

/* DS1307寄存器0x00~0x07原始转储：判NACK、全FF全00、CH停振位 */
static void DS1307_DumpRegs(void)
{
    uint8_t r[8];
    uint8_t i, err;

    err = I2C1_ReadBytes(DS1307_ADDR8, 0x00, r, 8);
    if (err)
    {
        printf("    raw dump FAIL err=%d (1=timeout 2=NACK)\r\n", err);
        return;
    }
    printf("    raw 0x00~0x07:");
    for (i = 0; i < 8; i++)
        printf(" %02X", r[i]);
    printf("\r\n");
    if (r[0] & 0x80)
        printf("    CH=1: oscillator HALTED (first power-up or backup battery dead)\r\n");
    printf("    hour reg=0x%02X (%s mode), control=0x%02X\r\n",
           r[2], (r[2] & 0x40) ? "12h" : "24h", r[7]);
}

/* 走时测试：隔3秒读两次，秒数应增加3（允许2~4覆盖读数偏差）
 * 返回0=走时正常；1=读失败；2=秒数不增（晶振不起振/供电不足） */
static uint8_t DS1307_TickTest(void)
{
    uint8_t h, m, s, y, mo, d;
    uint8_t s0, delta;

    if (DS1307_GetTime(&h, &m, &s, &y, &mo, &d))
        return 1;
    s0 = s;
    printf("    t0=%02u:%02u:%02u, wait 3s...", h, m, s);
    Delay_ms(3000);
    if (DS1307_GetTime(&h, &m, &s, &y, &mo, &d))
        return 1;
    delta = (uint8_t)((s + 60 - s0) % 60);
    printf(" t1=%02u:%02u:%02u (+%u s / 3 s)\r\n", h, m, s, delta);
    return (delta >= 2 && delta <= 4) ? 0 : 2;
}

/* DS1307 NVRAM读写测试：0x08~0x3F共56字节逐字节写读校验（覆盖原内容）
 * 返回0xFF=通信失败，否则=坏字节数 */
static uint8_t DS1307_RamTest(void)
{
    uint8_t a, wr, rd;
    uint8_t fail = 0;

    for (a = 0x08; a <= 0x3F; a++)
    {
        wr = (uint8_t)(a ^ 0xA5);
        if (I2C1_WriteBytes(DS1307_ADDR8, a, &wr, 1))
            return 0xFF;
        if (I2C1_ReadBytes(DS1307_ADDR8, a, &rd, 1))
            return 0xFF;
        if (rd != wr)
            fail++;
    }
    return fail;
}

/* AT24C32写：双字节字地址+数据（页写上限32字节，不允许跨页），
 * 写周期tWC约5ms用ACK轮询等待。返回0=成功 */
static uint8_t EE_Write(uint8_t addr8, uint16_t wordAddr, const uint8_t *p, uint16_t len)
{
    uint8_t buf[33];
    uint8_t i, err;

    if (len == 0 || len > 32)
        return 1;
    if ((wordAddr & 0x1F) + len > 32)      /* 不允许跨页写 */
        return 2;

    buf[0] = (uint8_t)wordAddr;            /* 低字节作数据首字节发 */
    for (i = 0; i < len; i++)
        buf[i + 1] = p[i];
    err = I2C1_WriteBytes(addr8, (uint8_t)(wordAddr >> 8), buf, (uint16_t)(len + 1));
    if (err)
        return err;

    for (i = 0; i < 20; i++)               /* ACK轮询最多约20ms */
    {
        Delay_ms(1);
        if (I2C1_ProbeAddr(addr8))
            return I2C_OK;
    }
    return I2C_ERR_TIMEOUT;
}

/* AT24C32完整测试：单字节写读 + 32字节页写读校验。返回0=全部通过 */
static uint8_t EE_Test(uint8_t addr7)
{
    uint8_t addr8 = (uint8_t)(addr7 << 1);
    uint8_t wbuf[32], rbuf[32];
    uint8_t i, err;
    uint8_t fail = 0;

    printf("[3] AT24C32 test @ 7-bit 0x%02X\r\n", addr7);

    /* 单字节写读：0xA5 -> 0x0040，读回校验 */
    rbuf[0] = 0;
    wbuf[0] = 0xA5;
    err = EE_Write(addr8, EE_TEST_BYTE, wbuf, 1);
    if (err == 0)
        err = I2C1_ReadAddr16(addr8, EE_TEST_BYTE, rbuf, 1);
    if (err == 0 && rbuf[0] == 0xA5)
        printf("    byte write/read @0x%04X: OK (A5)\r\n", EE_TEST_BYTE);
    else
    {
        printf("    byte write/read FAIL err=%d got=0x%02X\r\n", err, rbuf[0]);
        fail = 1;
    }

    /* 32字节页写 + 回读校验 */
    for (i = 0; i < 32; i++)
        wbuf[i] = (uint8_t)(i * 3 + 0x5C);
    err = EE_Write(addr8, EE_TEST_PAGE, wbuf, 32);
    if (err == 0)
        err = I2C1_ReadAddr16(addr8, EE_TEST_PAGE, rbuf, 32);
    if (err)
    {
        printf("    page test FAIL err=%d (1=timeout 2=NACK)\r\n", err);
        return 1;
    }
    for (i = 0; i < 32; i++)
    {
        if (rbuf[i] != wbuf[i])
        {
            fail = 1;
            printf("    page mismatch @0x%04X: want %02X got %02X\r\n",
                   EE_TEST_PAGE + i, wbuf[i], rbuf[i]);
        }
    }
    if (!fail)
        printf("    32-byte page write/verify @0x%04X: OK\r\n", EE_TEST_PAGE);

    return fail;
}

/* 全部诊断一次 */
static void RunAllTests(void)
{
    uint8_t found[8];
    uint8_t n, i;
    uint8_t dsFound = 0, ee7 = 0;
    uint8_t st, ramFail;
    uint8_t h, m, s, y, mo, d;

    n = Bus_Scan(found, 8);
    for (i = 0; i < n && i < 8; i++)
    {
        if (found[i] == 0x68)
            dsFound = 1;
        if (found[i] >= EE_ADDR7_MIN && found[i] <= EE_ADDR7_MAX)
            ee7 = found[i];
    }

    printf("[2] DS1307 RTC test\r\n");
    if (!dsFound)
    {
        printf("    DS1307 (0x68) NOT found on bus!\r\n");
        if (ee7)
            printf("    but EEPROM answers -> wiring OK, DS1307 chip/supply suspect\r\n");
    }
    else
    {
        DS1307_DumpRegs();

        st = DS1307_Init();
        if (st == DS1307_ERR_NO_ACK)
            printf("    DS1307_Init FAIL (NACK/timeout)\r\n");
        else if (st == DS1307_CLK_INVALID)
            printf("    CH was 1 -> cleared, oscillator restarted\r\n");
        else
            printf("    clock valid (CH=0)\r\n");

        /* 不自动重写时间（方便观察掉电保持，时间会一直延续下去）；
         * 只有寄存器乱掉/需要对时才发 's'。范围校验能直接点名电池失效的搅乱数据 */
        if (DS1307_GetTime(&h, &m, &s, &y, &mo, &d) == DS1307_OK)
        {
            if (s < 60 && m < 60 && h < 24 && d >= 1 && d <= 31 && mo >= 1 && mo <= 12)
                printf("    kept time: 20%02d-%02d-%02d %02d:%02d:%02d (not reset)\r\n",
                       y, mo, d, h, m, s);
            else
                printf("    registers GARBAGE (power lost & battery failed) -> press 's' to set baseline\r\n");
        }

        if (DS1307_TickTest() == 0)
            printf("    TICK PASS: oscillator running\r\n");
        else
            printf("    TICK FAIL: seconds not advancing -> X1 crystal or VCC<4.5V\r\n");

        ramFail = DS1307_RamTest();
        if (ramFail == 0xFF)
            printf("    RAM test: I2C error\r\n");
        else if (ramFail == 0)
            printf("    RAM 56 bytes: all OK\r\n");
        else
            printf("    RAM test: %u byte(s) bad\r\n", ramFail);
    }

    if (ee7)
        printf("    AT24C32 %s\r\n", EE_Test(ee7) ? "FAIL" : "PASS");
    else
        printf("[3] AT24C32 (0x50~0x57) NOT found on bus!\r\n");

    printf("=== test done ===\r\n");
}

int main(void)
{
    uint8_t h, m, s, y, mo, d;
    uint32_t t;
    uint8_t rtcFail = 0;

    Delay_Init();
    USART1_Init(115200);
    printf("\r\n===== Tiny RTC (DS1307 + AT24C32) diagnostic =====\r\n");
    printf("Wiring: VCC->5V, GND->GND, SCL->PB6, SDA->PB7\r\n");

    while (1)
    {
        /* 总线被钳死（如模块未上电）时每秒自动重试，供电/接线恢复后自动往下跑 */
        while (I2C_BusUnlock() == 2)
        {
            uint32_t tWait = GetTick();
            while ((GetTick() - tWait) < 1000)
            {
                if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
                {
                    uint8_t c = (uint8_t)USART_ReceiveData(USART1);
                    if (c == 'r' || c == 'R')
                        break;
                }
            }
        }
        I2C1_Init();

        RunAllTests();
        printf("Live clock started. 's'=set 12:00:00 baseline, 'r'=rerun tests (time NOT reset)\r\n");

        t = 0;
        rtcFail = 0;
        while (1)
        {
            if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
            {
                uint8_t c = (uint8_t)USART_ReceiveData(USART1);
                if (c == 'r' || c == 'R')
                    break;                 /* 重跑全部测试 */
                if (c == 's' || c == 'S')
                    DS1307_SetTime(RTC_DEF_YEAR, RTC_DEF_MON, RTC_DEF_DATE,
                                   RTC_DEF_HOUR, RTC_DEF_MIN, RTC_DEF_SEC);
            }

            if ((GetTick() - t) >= 1000)
            {
                t = GetTick();
                if (DS1307_GetTime(&h, &m, &s, &y, &mo, &d) == DS1307_OK)
                {
                    rtcFail = 0;
                    printf("RTC: 20%02d-%02d-%02d %02d:%02d:%02d\r\n", y, mo, d, h, m, s);
                }
                else
                {
                    /* 模块被拔电/插回时传输可能被打断：连续失败即做总线解锁+重初始化，
                     * 模块回电后自动恢复读数（本流程不重写时间，掉电保持看这里） */
                    rtcFail++;
                    if (rtcFail == 3 || (rtcFail > 3 && (rtcFail - 3) % 10 == 0))
                    {
                        printf("RTC lost x%u -> I2C bus unlock + re-init\r\n", rtcFail);
                        I2C_BusUnlock();
                        I2C1_Init();
                    }
                    else if (rtcFail % 5 == 0)
                        printf("RTC read FAIL x%u (module VCC unplugged?)\r\n", rtcFail);
                }
            }
        }
    }
}
