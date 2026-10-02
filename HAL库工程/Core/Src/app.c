#include "main.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "usart_app.h"
#include "lcd_spi_154.h"
#include "w5500_bsp.h"
#include "socket.h"
#include "modbus.h"
#include "mqtt.h"
#include "app.h"

/*
 * 阶段8：FreeRTOS 应用层 —— 任务、共享状态、心跳（与标准库版 app.c 功能一致）
 *
 * 任务与优先级（数值大=高）：
 *   vWatchdogTask  4  IWDG喂狗 + 各任务心跳巡检（1s周期，掉链自动复位）
 *   vCollectTask   3  Modbus 2s轮询 -> 共享温湿度（互斥保护）
 *   vNetworkTask   3  W5500健康检查(在位/自愈/链路防抖) + MQTT状态机 + 5s发布
 *   vDisplayTask   2  LCD仪表盘渲染（100ms，只重画变化字段）
 *
 * 共享状态（写加 xStateMutex，单变量读依赖对齐原子性）：
 *   sh_temp10/sh_hum10/sh_link/sh_mqttOk/sh_pubCnt/sh_modbusOk
 *
 * 注意：printf 为非线程安全，各任务日志频率已压低，偶发交错不影响功能。
 */

/* ===== 共享状态 ===== */
static SemaphoreHandle_t xStateMutex;
static volatile int16_t  sh_temp10 = 0;
static volatile uint16_t sh_hum10 = 0;
static volatile uint8_t  sh_link = 0;         /* 1=UP */
static volatile uint8_t  sh_mqttOk = 0;       /* 1=MQTT在线 */
static volatile uint16_t sh_pubCnt = 0;
static volatile uint8_t  sh_modbusOk = 1;

/* ===== 看门狗心跳 ===== */
#define HB_WATCHDOG 0
#define HB_COLLECT  1
#define HB_NETWORK  2
#define HB_DISPLAY  3
#define HB_COUNT    4
#define HB_STALE_MS 3000              /* 心跳超过3秒视为异常（DNS阻塞≤10s由IWDG 32.8s兜底） */
static volatile uint32_t hbTick[HB_COUNT];

/* ===== 显示辅助 ===== */

/* 链路状态行固定坐标刷新（内部记住上次状态，变化才重画） */
static void LCD_DisplayLink(uint8_t up)
{
    static uint8_t shown = 0xFF;
    if (shown == up)
        return;
    shown = up;
    LCD_SetColor(up ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(64, 40, up ? "UP  " : "DOWN");
    LCD_SetColor(LCD_BLACK);
}

/* MQTT 状态行固定坐标刷新（绿=ONLINE 红=OFF），返回1=本次发生了切换 */
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

/* 温湿度×10值拼 JSON 数值文本（不用%f，MicroLIB），返回写入字符数 */
static int FmtX10(char *out, int16_t v10)
{
    if (v10 < 0)
        return sprintf(out, "-%d.%d", (int)(-(int32_t)v10) / 10, (int)(-(int32_t)v10) % 10);
    return sprintf(out, "%d.%d", (int)v10 / 10, (int)v10 % 10);
}

/* ===== 心跳 ===== */
static void Heartbeat(uint8_t id)
{
    hbTick[id] = xTaskGetTickCount();
}

/* ===== 看门狗（寄存器直写，语义与标准库版完全一致） ===== */
static void IWDG_Start(void)
{
    /* 调试期冻结 IWDG（断点不停狗），发布版本可注释该行 */
    SET_BIT(DBGMCU->APB1FZ, DBGMCU_APB1_FZ_DBG_IWDG_STOP);

    IWDG->KR = 0x5555;                            /* 解除 PR/RLR 写保护 */
    IWDG->PR = 7;                                 /* LSI 32kHz/256 = 125Hz */
    IWDG->RLR = 0xFFF;                            /* 4095/125 ≈ 32.8s 超时 */
    IWDG->KR = 0xAAAA;                            /* 喂一次 */
    IWDG->KR = 0xCCCC;                            /* 启动 */
}

/* ===== 任务 ===== */

/* 看门狗：初始化IWDG（约32.8s超时），1s巡检心跳；任一任务心跳落后3秒→停喂→系统复位 */
static void vWatchdogTask(void *pv)
{
    (void)pv;
    uint8_t reported = 0;

    IWDG_Start();

    for (;;)
    {
        uint32_t now = xTaskGetTickCount();
        uint8_t  i, allAlive = 1;

        for (i = 0; i < HB_COUNT; i++)
        {
            if ((now - hbTick[i]) > HB_STALE_MS)
                allAlive = 0;
        }

        if (allAlive)
        {
            reported = 0;
            IWDG->KR = 0xAAAA;                    /* 喂狗 */
        }
        else if (!reported)
        {
            reported = 1;
            printf("watchdog: task heartbeat stale, IWDG not fed\r\n");
        }

        vTaskDelay(1000);
    }
}

/* 采集：Modbus 2秒轮询 -> 共享温湿度（互斥保护） */
static void vCollectTask(void *pv)
{
    (void)pv;
    int16_t  t10;
    uint16_t h10;
    uint8_t  fail = 0;

    for (;;)
    {
        if (MODBUS_ReadTempHum(&t10, &h10) == MB_OK)
        {
            fail = 0;
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                sh_temp10 = t10;
                sh_hum10 = h10;
                sh_modbusOk = 1;
                xSemaphoreGive(xStateMutex);
            }
        }
        else
        {
            fail++;
            if (fail <= 2 || fail % 10 == 0)
                printf("Modbus read fail x%d (keep last)\r\n", fail);
        }

        Heartbeat(HB_COLLECT);
        vTaskDelay(2000);
    }
}

/* 网络：W5500健康检查 + MQTT状态机 + 5秒发布
 * 在位检测/自愈/链路防抖逻辑与阶段7裸机版参数一致，仅任务化 */
static void vNetworkTask(void *pv)
{
    (void)pv;
    char     json[80];
    int      jsonLen;
    uint8_t  verFail = 0;
    uint32_t tOff = 0;
    uint8_t  modOk = 1;       /* 模块在位（网络任务自己的防抖状态） */
    uint8_t  link = 0;        /* 链路状态（网络任务自己的防抖状态） */
    uint8_t  linkDownCnt = 0;
    uint8_t  cfgFail = 0;
    uint32_t tPub = 0;

    for (;;)
    {
        /* 在位检测：连续5次坏读才判离线（离线期严禁触碰socket API——0xFF读数会冻结close） */
        if (W5500_ReadVersion() != 0x04)
        {
            verFail++;
            if (verFail < 5)
            {
                vTaskDelay(10);
                continue;
            }
            if (modOk)
            {
                modOk = 0;
                printf("W5500 module offline (5 consecutive bad version reads)\r\n");
            }
            if ((xTaskGetTickCount() - tOff) >= 2000)
            {
                tOff = xTaskGetTickCount();
                printf("module offline, version read: 0x%02X spi1err=%d\r\n",
                       W5500_ReadVersion(), W5500_BSP_SPI1Err());
            }
            vTaskDelay(500);
            continue;
        }
        verFail = 0;
        if (!modOk)
        {
            modOk = 1;
            printf("W5500 module online\r\n");
        }

        /* 配置自愈（防抖：连续3次不一致才重下发） */
        if (W5500_CheckConfig() == 0)
        {
            if (cfgFail < 3)
                cfgFail++;
            if (cfgFail >= 3)
            {
                cfgFail = 0;
                printf("W5500 config lost (chip reset?) - re-applied\r\n");
                W5500_NetworkInit();
            }
        }
        else
            cfgFail = 0;

        /* 链路状态（防抖：连续3次DOWN才关socket） */
        {
            uint8_t linkNow = W5500_LinkStatus();
            if (linkNow != link)
            {
                link = linkNow;
                printf("PHY %s\r\n", link ? "LinkUp" : "LinkDown");
            }
            sh_link = link;           /* 同步给显示任务 */

            if (!link)
            {
                linkDownCnt++;
                if (linkDownCnt >= 3)
                {
                    linkDownCnt = 0;
                    if (getSn_SR(MQTT_SOCK) != SOCK_CLOSED)
                        close(MQTT_SOCK);      /* 模块在线，close安全 */
                    sh_mqttOk = 0;
                }
                vTaskDelay(10);
                continue;
            }
            linkDownCnt = 0;
        }

        /* MQTT 状态机（DNS/TCP/CONNACK/keepalive/重连 均在其内部，带超时） */
        if (MQTT_Process() == MQ_MQTT_OK)
        {
            if (!sh_mqttOk)
            {
                sh_mqttOk = 1;
                printf("MQTT: ONLINE\r\n");
            }

            /* 每5秒发布 JSON（温湿度取共享最新值） */
            if ((xTaskGetTickCount() - tPub) >= 5000)
            {
                tPub = xTaskGetTickCount();
                jsonLen = sprintf(json, "{\"device\":\"gw001\",\"temp\":");
                jsonLen += FmtX10(json + jsonLen, sh_temp10);
                jsonLen += sprintf(json + jsonLen, ",\"hum\":");
                jsonLen += FmtX10(json + jsonLen, (int16_t)sh_hum10);
                jsonLen += sprintf(json + jsonLen, "}");

                if (MQTT_Publish(MQTT_TOPIC, (const uint8_t *)json, (uint16_t)jsonLen) == MQTT_OK)
                {
                    sh_pubCnt++;
                    printf("PUB[%d]: %s\r\n", sh_pubCnt, json);
                }
            }
        }

        Heartbeat(HB_NETWORK);
        vTaskDelay(10);
    }
}

/* 显示：LCD仪表盘渲染（100ms），只重画变化的字段 */
static void vDisplayTask(void *pv)
{
    (void)pv;
    static int16_t  shownTemp = 0x7FFF;   /* 哨兵值：强制首刷 */
    static uint16_t shownHum = 0xFFFF;
    static uint16_t shownPub = 0xFFFF;

    for (;;)
    {
        LCD_DisplayLink(sh_link);
        LCD_DisplayMqtt(sh_mqttOk);

        if (sh_temp10 != shownTemp)
        {
            shownTemp = sh_temp10;
            LCD_DisplayTenths(64, 92, sh_temp10);
        }
        if (sh_hum10 != shownHum)
        {
            shownHum = sh_hum10;
            LCD_DisplayTenths(64, 116, (int16_t)sh_hum10);
        }
        if (sh_pubCnt != shownPub)
        {
            shownPub = sh_pubCnt;
            LCD_DisplayNumber(48, 192, shownPub, 5);
        }

        Heartbeat(HB_DISPLAY);
        vTaskDelay(100);
    }
}

/* ===== 应用初始化：创建互斥锁 + 全部任务（调度器由 main 启动） ===== */
void APP_Init(void)
{
    /* 共享状态互斥锁 */
    xStateMutex = xSemaphoreCreateMutex();

    /* 心跳基准清零（调度器启动后各任务随即刷新） */
    memset((void *)hbTick, 0, sizeof(hbTick));

    xTaskCreate(vWatchdogTask, "WDG", 128,  NULL, 4, NULL);
    xTaskCreate(vCollectTask,  "COL", 512,  NULL, 3, NULL);
    xTaskCreate(vNetworkTask,  "NET", 512,  NULL, 3, NULL);
    xTaskCreate(vDisplayTask,  "DSP", 512,  NULL, 2, NULL);
}
