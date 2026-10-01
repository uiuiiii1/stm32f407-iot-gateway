#include "stm32f4xx.h"
#include <stdio.h>
#include "delay.h"
#include "usart.h"
#include "lcd_spi_154.h"
#include "w5500_bsp.h"
#include "socket.h"
#include "modbus.h"
#include "mqtt.h"

/*
 * 标准库基础工程 —— 阶段6：MQTT 上云（EMQX 公共服务器 broker.emqx.io:1883）
 * W5500 接线：SCLK=PA5 / MISO=PA6 / MOSI=PA7（SPI1），nSS=PC4，nRST=PC5
 * 网络：静态IP 192.168.0.250，经路由器出公网；Modbus 变送器接 USART2（DE=PA4）
 *
 * 功能：每2秒 Modbus 读温湿度 -> 每5秒以 JSON 发布到 gateway/gw001/data；
 * MQTT（Socket1）状态机嵌在"模块在线 + 链路UP"区域内；DNS（Socket2）解析域名。
 * 电脑用 MQTTX 连 broker.emqx.io:1883 订阅 gateway/gw001/data 验证。
 *
 * 【备份说明】2026-10-01 换 DS1307+AT24C32 诊断测试时被覆盖，本文件为原样备份，
 * 不在 Keil 工程文件列表内（不会被编译）。恢复阶段6时把内容拷回 main.c 即可。
 */

/* 链路状态行固定坐标刷新（局部覆盖，绿=通 红=断） */
static void LCD_DisplayLink(uint8_t up)
{
    LCD_SetColor(up ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(64, 70, up ? "UP  " : "DOWN");
    LCD_SetColor(LCD_BLACK);
}

/* MQTT 状态行固定坐标刷新（绿=ONLINE 红=OFF）
 * 内部记住上次画的状态，只在变化时重画；返回1=本次发生了切换（供串口日志用） */
static int LCD_DisplayMqtt(uint8_t ok)
{
    static uint8_t shown = 0xFF;
    if (shown == ok)
        return 0;
    shown = ok;
    LCD_SetColor(ok ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(64, 130, ok ? "ONLINE" : "OFF   ");
    LCD_SetColor(LCD_BLACK);
    return 1;
}

/* 温湿度×10值拼 JSON 数值："-" + 整数部分 + "." + 小数部分（不用%f，MicroLIB）
 * 返回写入字符数 */
static int FmtX10(char *out, int16_t v10)
{
    uint16_t abs10;
    char sign = 0;
    if (v10 < 0) { sign = '-'; abs10 = (uint16_t)(-(int32_t)v10); }
    else          abs10 = (uint16_t)v10;
    if (sign)
        return sprintf(out, "%c%d.%d", sign, abs10 / 10, abs10 % 10);
    return sprintf(out, "%d.%d", abs10 / 10, abs10 % 10);
}

int main(void)
{
    uint8_t  link = 0xFF;     /* 0xFF=初始未知，强制首刷 */
    uint8_t  linkNow;
    uint8_t  modOk = 1;       /* W5500模块在位状态（版本寄存器可读=在位） */
    uint32_t tOff = 0;        /* OFF状态下周期打印用 */
    uint8_t  verFail = 0;     /* 连续版本读失败计数（防抖：满5次才判离线） */
    uint8_t  linkDownCnt = 0; /* 连续LinkDown计数（防抖：满3次才关socket） */
    uint8_t  cfgFail = 0;     /* 连续配置不一致计数（防抖：满3次才重下发） */

    /* 阶段6 业务变量 */
    int16_t  temp10 = 0;      /* 温度×10（补码），Modbus 最近一次成功值 */
    uint16_t hum10 = 0;       /* 湿度×10 */
    uint8_t  mbFailCnt = 0;   /* Modbus 连续失败计数 */
    uint16_t pubCnt = 0;      /* 发布成功计数 */
    uint32_t tRead = 0, tPub = 0;   /* 2秒读 / 5秒发布 节拍 */
    char     json[80];
    int      jsonLen;

    Delay_Init();
    USART1_Init(115200);
    printf("System Start\r\n");

    SPI_LCD_Init();
    LCD_SetAsciiFont(&ASCII_Font16);
    LCD_SetBackColor(LCD_WHITE);
    LCD_SetColor(LCD_BLACK);
    LCD_Clear();

    LCD_DisplayString(16, 16, "Industrial Gateway");
    LCD_DisplayString(16, 40, "Stage6: MQTT");

    MB_USART_Init();
    W5500_BSP_Init();
    W5500_NetworkInit();
    MQTT_Init();
    printf("W5500 ready: IP 192.168.0.250, MQTT broker.emqx.io:1883\r\n");

    LCD_DisplayString(16, 70,  "Link:");
    LCD_DisplayString(16, 100, "IP: 192.168.0.250");
    LCD_DisplayString(16, 130, "MQTT:");
    LCD_DisplayString(16, 160, "PUB:0");
    LCD_DisplayMqtt(0);

    while (1)
    {
        /* 模块在位检测：W5500未供电时MISO上拉会把寄存器读成0xFF，链路读数全不可信。
         * 连续5次坏读才判定离线——单次SPI坏读（数据收发时的电源毛刺）不允许杀死TCP连接 */
        if (W5500_ReadVersion() != 0x04)
        {
            verFail++;
            if (verFail < 5)      /* 瞬时坏读：不动socket，快速重试保住TCP */
            {
                Delay_ms(10);
                continue;
            }

            /* 连续5次失败：确认离线 */
            if (modOk)
            {
                modOk = 0;
                tOff = GetTick();
                LCD_SetColor(LCD_RED);
                LCD_DisplayString(64, 70, "OFF ");
                LCD_SetColor(LCD_BLACK);
                LCD_DisplayMqtt(0);
                printf("W5500 module offline (5 consecutive bad version reads)\r\n");
            }
            if ((GetTick() - tOff) >= 2000)   /* 每2秒报一次实测版本值，用于判线 */
            {
                tOff = GetTick();
                printf("module offline, version read: 0x%02X spi1err=%d (0xFF=floating 0x00=stuck low)\r\n",
                       W5500_ReadVersion(), W5500_BSP_SPI1Err());
            }
            /* 串口发任意字符触发SPI1环回自检：先拔掉模块MISO/MOSI两根线、用一根杜邦线短接PA6-PA7 */
            if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
            {
                uint8_t cmd = (uint8_t)USART_ReceiveData(USART1);
                uint8_t tx[4] = {0xA5, 0x3C, 0x00, 0xFF};
                uint8_t rx[4];
                (void)cmd;
                printf("SPI1 loopback test running (PA6<->PA7 jumpered)...\r\n");
                if (W5500_BSP_SPILoopback(tx, rx, 4))
                    printf("loopback PASS -> SPI1/PA6/PA7 all fine, problem is MODULE side\r\n");
                else
                    printf("loopback FAIL: got %02X %02X %02X %02X -> core-board side (SPI1/PA6/PA7)\r\n",
                           rx[0], rx[1], rx[2], rx[3]);
            }
            /* 注意：确认离线后严禁触碰任何socket API（ioLibrary close 会死等 0xFF 读数导致冻结）；
             * 模块恢复供电后其Socket硬件自动回到CLOSED，无需远程关闭 */
            Delay_ms(500);
            continue;
        }
        verFail = 0;              /* 版本读正常：清零失败计数 */
        if (!modOk)                   /* 模块恢复供电：复位状态，强制刷新链路显示 */
        {
            modOk = 1;
            link = 0xFF;
            printf("W5500 module online\r\n");
        }

        /* 配置自愈（防抖）：芯片被意外复位（干扰/电源抖动）会丢失IP配置但PHY仍显示Link。
         * 连续3次校验不一致才重新下发——单次SPI坏读不触发重配 */
        if (W5500_CheckConfig() == 0)
        {
            if (cfgFail < 3)
                cfgFail++;
            if (cfgFail >= 3)
            {
                cfgFail = 0;
                link = 0xFF;              /* 芯片复位过，强制刷新链路显示 */
                printf("W5500 config lost (chip reset?) - re-applied\r\n");
                W5500_NetworkInit();
            }
        }
        else
            cfgFail = 0;

        /* 链路状态监测（变化才刷屏/打印） */
        linkNow = W5500_LinkStatus();
        if (linkNow != link)
        {
            link = linkNow;
            LCD_DisplayLink(link);
            printf("PHY %s\r\n", link ? "LinkUp" : "LinkDown");
            if (link)                     /* 链路恢复：立即重试MQTT（不等重连间隔自然到点） */
                tPub = GetTick();
        }

        if (!link)
        {
            /* 防抖：连续3次(~30ms)读都断才关socket——单次PHY读数毛刺不杀TCP连接 */
            if (linkDownCnt < 3)
            {
                linkDownCnt++;
                Delay_ms(10);
                continue;
            }
            linkDownCnt = 0;
            if (getSn_SR(MQTT_SOCK) != SOCK_CLOSED)
                close(MQTT_SOCK);         /* 链路确认断开：关socket，MQTT状态机自会重连 */
            LCD_DisplayMqtt(0);
            Delay_ms(200);
            continue;
        }
        linkDownCnt = 0;

        /* MQTT 状态机（模块在线 + 链路UP 区域内）：连接/保活/自动重连 */
        if (MQTT_Process() == MQ_MQTT_OK)
        {
            if (LCD_DisplayMqtt(1))
                printf("MQTT: ONLINE\r\n");

            /* 每2秒 Modbus 读温湿度（失败保留上次值并计数） */
            if ((uint32_t)(GetTick() - tRead) >= 2000)
            {
                tRead = GetTick();
                if (MODBUS_ReadTempHum(&temp10, &hum10) == MB_OK)
                    mbFailCnt = 0;
                else
                {
                    mbFailCnt++;
                    if (mbFailCnt <= 3 || mbFailCnt % 10 == 0)
                        printf("Modbus read fail x%d (keep last value)\r\n", mbFailCnt);
                }
            }

            /* 每5秒发布 JSON */
            if ((uint32_t)(GetTick() - tPub) >= 5000)
            {
                tPub = GetTick();
                jsonLen = sprintf(json, "{\"device\":\"%s\",\"temp\":", "gw001");
                jsonLen += FmtX10(json + jsonLen, temp10);
                jsonLen += sprintf(json + jsonLen, ",\"hum\":");
                jsonLen += FmtX10(json + jsonLen, (int16_t)hum10);
                jsonLen += sprintf(json + jsonLen, "}");

                if (MQTT_Publish(MQTT_TOPIC, (const uint8_t *)json, (uint16_t)jsonLen) == MQTT_OK)
                {
                    pubCnt++;
                    printf("PUB[%d]: %s\r\n", pubCnt, json);
                    LCD_SetColor(LCD_BLACK);
                    LCD_DisplayString(52, 160, "     ");
                    LCD_DisplayString(16, 160, "PUB:");
                    LCD_SetColor(LCD_BLACK);
                    {
                        char line[8];
                        sprintf(line, "%d", pubCnt);
                        LCD_DisplayString(52, 160, line);
                    }
                }
            }
        }
        else
        {
            LCD_DisplayMqtt(0);
            /* 离线期节拍基准前移：避免恢复在线瞬间连发 */
            tPub = GetTick();
        }

        Delay_ms(10);
    }
}
