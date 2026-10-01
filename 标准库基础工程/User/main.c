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
 * 标准库基础工程 —— 阶段7：仪表盘整合（里程碑A收官）
 * W5500 接线：SCLK=PA5 / MISO=PA6 / MOSI=PA7（SPI1），nSS=PC4，nRST=PC5
 * 网络：静态IP 192.168.0.250（见 w5500_bsp.h 配置区）；Modbus 变送器接 USART2（DE=PA4）
 *
 * 屏幕布局（240x240，全部固定坐标局部刷新，不整屏Clear）：
 *   y=16  Industrial Gateway
 *   y=40  Link: UP/DOWN（绿/红，防抖）
 *   y=64  MQTT: ONLINE/OFF
 *   y=92  Temp: xx.x C    （Modbus 2秒轮询，负温度带负号）
 *   y=116 Hum : xx.x %
 *   y=144 IP: 192.168.0.250
 *   y=168 PUB: n          （发布成功计数）
 *
 * 功能：每2秒 Modbus 读温湿度并刷新屏幕（不依赖网络状态，显示保持最后成功值）；
 * 每5秒以 JSON 发布到 gateway/gw001/data；MQTT 状态机嵌在"模块在线 + 链路UP"区域内。
 * 健壮性逻辑（在位检测/配置自愈/链路防抖）与阶段6一致，未改动。
 * RTC时间显示：DS1307模块确认故障已退换，改用F407内部RTC（板载32.768K晶振）；
 * CR1220电池到位前断电时间会丢，串口发 's' 设基准时间（SNTP网络对时为后续增量）。
 */

/* ===== 显示辅助 ===== */

/* 链路状态行固定坐标刷新（局部覆盖，绿=通 红=断） */
static void LCD_DisplayLink(uint8_t up)
{
    LCD_SetColor(up ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(64, 40, up ? "UP  " : "DOWN");
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
    LCD_DisplayString(64, 64, ok ? "ONLINE" : "OFF   ");
    LCD_SetColor(LCD_BLACK);
    return 1;
}

/* 温湿度×10值显示：符号(1字符)+3位整数+小数点+1位小数，共48像素，固定坐标局部覆盖 */
static void LCD_DisplayTenths(uint16_t x, uint16_t y, int16_t v10)
{
    uint16_t abs10;

    if (v10 < 0)
    {
        LCD_DisplayString(x, y, "-");
        abs10 = (uint16_t)(-(int32_t)v10);
    }
    else
    {
        LCD_DisplayString(x, y, " ");
        abs10 = (uint16_t)v10;
    }
    LCD_DisplayNumber(x + 8, y, abs10 / 10, 3);
    LCD_DisplayString(x + 32, y, ".");
    LCD_DisplayNumber(x + 40, y, abs10 % 10, 1);
}

/* 温湿度×10值拼 JSON 数值文本："-" + 整数部分 + "." + 小数部分（不用%f，MicroLIB）
 * 返回写入字符数 */
static int FmtX10(char *out, int16_t v10)
{
    if (v10 < 0)
        return sprintf(out, "-%d.%d", (int)(-(int32_t)v10) / 10, (int)(-(int32_t)v10) % 10);
    return sprintf(out, "%d.%d", (int)v10 / 10, (int)v10 % 10);
}

int main(void)
{
    int16_t  temp10 = 0;      /* 温度×10（补码），Modbus 最近一次成功值 */
    uint16_t hum10 = 0;       /* 湿度×10 */
    uint8_t  mbFailCnt = 0;   /* Modbus 连续失败计数 */
    uint16_t pubCnt = 0;      /* 发布成功计数 */
    uint32_t tRead = 0, tPub = 0;   /* 2秒读 / 5秒发布 节拍 */
    char     json[80];
    int      jsonLen;
    uint8_t  link = 0xFF;     /* 0xFF=初始未知，强制首刷 */
    uint8_t  linkNow;
    uint8_t  modOk = 1;       /* W5500模块在位状态（版本寄存器可读=在位） */
    uint32_t tOff = 0;        /* OFF状态下周期打印用 */
    uint8_t  verFail = 0;     /* 连续版本读失败计数（防抖：满5次才判离线） */
    uint8_t  linkDownCnt = 0; /* 连续LinkDown计数（防抖：满3次才关socket） */
    uint8_t  cfgFail = 0;     /* 连续配置不一致计数（防抖：满3次才重下发） */

    Delay_Init();
    USART1_Init(115200);
    printf("System Start\r\n");

    SPI_LCD_Init();
    LCD_SetAsciiFont(&ASCII_Font16);
    LCD_SetBackColor(LCD_WHITE);
    LCD_SetColor(LCD_BLACK);
    LCD_Clear();
    LCD_ShowNumMode(Fill_Space);  /* 数字不足位补空格 */

    /* 仪表盘静态框架 */
    LCD_DisplayString(16, 16,  "Industrial Gateway");
    LCD_DisplayString(16, 40,  "Link:");
    LCD_DisplayString(16, 64,  "MQTT:");
    LCD_DisplayString(16, 92,  "Temp:");
    LCD_DisplayString(16, 116, "Hum :");
    LCD_DisplayString(112, 92,  "C");
    LCD_DisplayString(112, 116, "%");
    LCD_DisplayString(16, 144, "IP: 192.168.0.250");
    LCD_DisplayString(16, 168, "PUB:");

    MB_USART_Init();
    W5500_BSP_Init();
    W5500_NetworkInit();
    MQTT_Init();
    printf("Gateway ready: IP 192.168.0.250, broker.emqx.io:1883, MQTT pub 5s\r\n");

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
                LCD_DisplayString(64, 40, "OFF ");
                LCD_DisplayMqtt(0);
                LCD_SetColor(LCD_BLACK);
                printf("W5500 module offline (5 consecutive bad version reads)\r\n");
            }
            if ((GetTick() - tOff) >= 2000)   /* 每2秒报一次实测版本值，用于判线 */
            {
                tOff = GetTick();
                printf("module offline, version read: 0x%02X spi1err=%d (0xFF=floating 0x00=stuck low)\r\n",
                       W5500_ReadVersion(), W5500_BSP_SPI1Err());
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

            /* 每5秒发布 JSON（温湿度用最近一次成功值） */
            if ((GetTick() - tPub) >= 5000)
            {
                tPub = GetTick();
                jsonLen = sprintf(json, "{\"device\":\"gw001\",\"temp\":");
                jsonLen += FmtX10(json + jsonLen, temp10);
                jsonLen += sprintf(json + jsonLen, ",\"hum\":");
                jsonLen += FmtX10(json + jsonLen, (int16_t)hum10);
                jsonLen += sprintf(json + jsonLen, "}");

                if (MQTT_Publish(MQTT_TOPIC, (const uint8_t *)json, (uint16_t)jsonLen) == MQTT_OK)
                {
                    pubCnt++;
                    printf("PUB[%d]: %s\r\n", pubCnt, json);
                    LCD_DisplayNumber(48, 192, pubCnt, 5);
                }
            }
        }
        else
        {
            LCD_DisplayMqtt(0);
            /* 离线期节拍基准前移：避免恢复在线瞬间连发 */
            tPub = GetTick();
        }

        /* 阶段7：每2秒 Modbus 读温湿度并刷新仪表盘（不依赖网络状态，失败保持上次显示） */
        if ((GetTick() - tRead) >= 2000)
        {
            tRead = GetTick();
            if (MODBUS_ReadTempHum(&temp10, &hum10) == MB_OK)
            {
                mbFailCnt = 0;
                LCD_DisplayTenths(64, 92, temp10);
                LCD_DisplayTenths(64, 116, (int16_t)hum10);
            }
            else
            {
                mbFailCnt++;
                if (mbFailCnt <= 2 || mbFailCnt % 10 == 0)
                    printf("Modbus read fail x%d (keep last display)\r\n", mbFailCnt);
            }
        }

        Delay_ms(10);
    }
}
