#include "main.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "usart_app.h"
#include "lcd_spi_154.h"
#include "w5500_bsp.h"
#include "socket.h"
#include "modbus.h"
#include "mqtt.h"
#include "rtc_app.h"
#include "datalog.h"
#include "sdlog.h"
#include "sntp.h"
#include "ota.h"
#include "air780e.h"
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
static volatile uint8_t  sh_mqttOk = 0;       /* 上云通道：1=正常（以太网或4G任一条通） 0=离线 */
static volatile uint8_t  sh_4gUp = 0;         /* 1=当前上云走的是 4G 备份通道（显示任务用） */
static volatile uint16_t sh_pubCnt = 0;
static volatile uint8_t  sh_modbusOk = 1;

/* ===== 看门狗心跳 ===== */
#define HB_WATCHDOG 0
#define HB_COLLECT  1
#define HB_NETWORK  2
#define HB_STORAGE  3
#define HB_DISPLAY  4
#define HB_COUNT    5
#define HB_STALE_MS 3000              /* 心跳超过3秒视为异常（DNS阻塞≤10s由IWDG 32.8s兜底） */
static volatile uint32_t hbTick[HB_COUNT];
static const char *const hbName[HB_COUNT] = { "watchdog", "collect", "network", "storage", "display" };

/* ===== 阶段9：断网缓存数据流 ===== */
typedef struct {
    uint32_t ts;      /* Unix 秒（UTC）；0=时间未同步，不可缓存 */
    int16_t  t10;
    uint16_t h10;
} DataMsg;

static QueueHandle_t xDataQueue;     /* COL -> STG：原始读数（深度16） */
static QueueHandle_t xReplayQueue;   /* STG -> NET：待补传记录（深度8） */

/* 跨任务标志（单写多读，volatile 足够） */
static volatile uint8_t gTimeSynced = 0;      /* 1=RTC 已经有可信时间（SNTP/电池） */
static volatile uint8_t gReplayReq = 0;       /* 1=网络任务请求补传（链路恢复且缓存非空） */
static volatile uint8_t gDrainActive = 0;     /* 1=存储任务正在灌补传队列 */
static volatile uint8_t gDrainDone = 0;       /* 1=存储任务灌完（补传队列可能还有尾货） */

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

/* MQTT 上云状态行固定坐标刷新（只重画变化字段），返回1=本次发生了切换
 *   st=0 离线（红 OFF） / 1 以太网上云（绿 ONLINE） / 2 4G 备份上云（绿 4G OK）
 * 4G 接管时也显示绿：上云确实通着，只是换了条路——屏上直接看得出"切到 4G 了" */
static int LCD_DisplayMqtt(uint8_t st)
{
    static uint8_t shown = 0xFF;
    if (shown == st)
        return 0;
    shown = st;
    LCD_SetColor(st ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(64, 64, (st == 2) ? "4G OK " : ((st == 1) ? "ONLINE" : "OFF   "));
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

#if AIR780E_ENABLE
/* 4G 驱动里的 AT 长阻塞（单个命令最长 8s，整段启动最长 ~19s）期间，
 * air780e.c 通过这个回调替网络任务喂心跳——否则看门狗任务会在 3s 后
 * 报 "network heartbeat stale" 并停喂 IWDG */
static void NetHeartbeat(void)
{
    Heartbeat(HB_NETWORK);
}
#endif

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
        Heartbeat(HB_WATCHDOG);   /* 给自己也打卡（漏了它=3秒后自判卡死、永远不喂狗、33秒必复位） */
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
            /* 找出第一个心跳超时的任务点名打印（reported保证只刷一次） */
            for (i = 0; i < HB_COUNT && (now - hbTick[i]) <= HB_STALE_MS; i++)
                ;
            reported = 1;
            printf("watchdog: %s heartbeat stale, IWDG not fed\r\n", hbName[i]);
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
    uint16_t dropCnt = 0;
    DataMsg   m;

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
            /* 阶段9：读数入队（时间未同步时 ts=0，存储任务不缓存无时间戳数据） */
            m.ts  = gTimeSynced ? RTC_GetUnix() : 0;
            m.t10 = t10;
            m.h10 = h10;
            if (m.ts != 0 && xQueueSend(xDataQueue, &m, 0) != pdTRUE)
            {
                dropCnt++;
                if (dropCnt <= 2 || dropCnt % 50 == 0)
                    printf("data queue full, dropped x%d\r\n", dropCnt);
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
    /* ---- 阶段9 补充 ---- */
    uint8_t  wasDown = 0;              /* 链路曾断标志（恢复瞬间触发补传） */
    uint8_t  sntpFirst = 1;            /* SNTP首次尝试标志（上线后立刻对时） */
    uint32_t tSntp = 0;                /* 上次SNTP尝试时刻（失败60秒重试） */
    uint32_t tReplay = 0;              /* 补传节流（每条间隔>=20ms） */
    uint32_t replayCnt = 0;            /* 补传成功条数 */
    DataMsg   pend;                    /* 补传在途一条（发布失败保留重试） */
    uint8_t   havePend = 0;
    char      rjson[96];               /* 补传JSON缓冲（含ts字段） */
    int       rjsonLen;
    /* ---- 阶段13.4 4G 备份通道 ---- */
    uint32_t  t4gDown = 0;             /* 以太网断开的起始时刻（0=在线） */
    uint8_t   g4gOn = 0;               /* 4G 通道接管标志 */
    uint32_t  t4gPub = 0;              /* 4G 上次发布时刻 */
    uint32_t  t4gRetry = 0;            /* 4G 启动失败重试节流 */
    uint8_t   pubFail = 0;             /* 4G 连续发布失败计数（≥3 触发重新握手） */

    for (;;)
    {
        /* 打卡必须在所有 continue 之前：离线/模块离线分支会 continue 跳过循环尾，
         * 挂在循环尾的打卡会让"拔网线=网络任务永不打卡=看门狗33秒复位" */
        Heartbeat(HB_NETWORK);

#if AIR780E_ENABLE
        /* ===== 阶段13.4：4G 备份通道（以太网断 AIR780E_SWITCH_MS 后接管发布） =====
         * 启动序列最坏连续阻塞 ~36s（10s 探 AT + 0.3s 硬复位 + 20s 等模块重启 + 6s 诊断），
         * 但全程都有人打卡：本 loop 每圈打一次，AIR780E_At / AIR780E_HardReset / AIR780E_Diag
         * 内部还会通过 AIR780E_SetHeartbeatCb 注册的回调反复替本任务打卡（APP_Init 里注册）。
         * ⚠️ 会触发复位的是"某个任务心跳陈旧 3s"，不是"总时长 32.8s"——只要心跳不断，
         *    阻塞 36s 也安全（看门狗照常喂 IWDG）。
         * 启动失败 AIR780E_RETRY_MS 后重试。稳态每 5s 一次 MPUB。 */
        if (sh_link == 0)
        {
            if (t4gDown == 0)
            {
                t4gDown = xTaskGetTickCount();
            }
            if (!g4gOn && (xTaskGetTickCount() - t4gDown) >= AIR780E_SWITCH_MS &&
                (xTaskGetTickCount() - t4gRetry) >= AIR780E_RETRY_MS)
            {
                uint8_t i, ok = 0;
                AIR780E_Init();
                printf("[4G] startup: RDY=%u\r\n", AIR780E_IsOnline());
                for (i = 0; i < 10; i++)            /* 等模块协议栈就绪（~10s） */
                {
                    Heartbeat(HB_NETWORK);
                    if (AIR780E_WaitReady(1000) == AIR780E_OK) { ok = 1; break; }
                }
                if (!ok && AIR780E_HardReset())     /* 一声不吭 = 模块卡死，硬复位一次再等它启动 */
                {
                    printf("[4G] AT no reply -> hard reset module, wait boot\r\n");
                    /* 实测：模块从复位释放到能答 AT 要 ~20.2s，20s 窗口差 0.2s 没抓到，
                     * 白等一轮 5s 重试，所以放宽到 25s。探到就 break，正常情况不增加任何耗时 */
                    for (i = 0; i < 25; i++)
                    {
                        Heartbeat(HB_NETWORK);
                        if (AIR780E_WaitReady(1000) == AIR780E_OK) { ok = 1; break; }
                    }
                }
                if (ok)
                {
                    Heartbeat(HB_NETWORK);
                    ok = (AIR780E_MQTTStart("broker.emqx.io", 1883) == AIR780E_OK);
                }
                if (ok)
                {
                    g4gOn = 1;
                    sh_mqttOk = 1;                  /* 4G 上云通道打通：LCD 的 MQTT 行不该再是红的 */
                    printf("[4G] channel ON (eth down %us)\r\n",
                           (unsigned)((xTaskGetTickCount() - t4gDown) / 1000));
                }
                else
                {
                    t4gRetry = xTaskGetTickCount();
                    sh_mqttOk = 0;                  /* 以太网断着、4G 也没起来 = 真的离线 */
                    AIR780E_Diag();                 /* 失败必带诊断：RDY/收字节/帧错误/AT回话hex */
                    printf("[4G] start FAIL, retry in %us\r\n",
                           (unsigned)(AIR780E_RETRY_MS / 1000));
                }
            }
            if (g4gOn && (xTaskGetTickCount() - t4gPub) >= 5000)
            {
                t4gPub = xTaskGetTickCount();
                jsonLen = sprintf(json, "{\"device\":\"gw001\",\"temp\":");
                jsonLen += FmtX10(json + jsonLen, sh_temp10);
                jsonLen += sprintf(json + jsonLen, ",\"hum\":");
                jsonLen += FmtX10(json + jsonLen, (int16_t)sh_hum10);
                jsonLen += sprintf(json + jsonLen, "}");    /* ⚠️ 别漏：缺了它发出的就是坏 JSON */
                json[jsonLen] = '\0';
                if (AIR780E_MQTTPublishJson("gateway/gw001/data", json) == AIR780E_OK)
                {
                    printf("[4G] PUB: %s\r\n", json);
                    pubFail = 0;
                    sh_mqttOk = 1;
                }
                else
                {
                    pubFail++;
                    printf("[4G] PUB FAIL\r\n");
                    if (pubFail >= 3)
                    {
                        /* 连发 3 条都不过：4G 侧 MQTT 会话多半已经死了（模块被踢/掉网），
                         * 而 g4gOn 一直是 1，不重置就永远卡在"启动过了但发不出去"。
                         * 先补一条 AT 探活，把两种情况分开处理：
                         *   还回 AT   → 会话死了但模块活着：重跑 MQTTStart 就够
                         *   连 AT 都不回 → 模块卡死 或 RX 线掉了：直接硬复位（60s 节流保护）
                         * 注意：407 分不清"模块卡死"和"只是听不到"，所以这里是宁可错杀——
                         * 对活着的模块代价只是重启十几秒，而 4G 是备份通道，数据本来就先进
                         * W25Q64 缓存、联网后补传，不会丢。 */
                        pubFail = 0;
                        g4gOn = 0;
                        sh_mqttOk = 0;
                        t4gRetry = xTaskGetTickCount();
                        printf("[4G] pub fail x3, re-handshake\r\n");
                        if (AIR780E_At("AT", "OK", 0, 0, 1500) != AIR780E_OK)
                        {
                            printf("[4G] AT silent -> hard reset module\r\n");
                            (void)AIR780E_HardReset();
                        }
                    }
                }
                Heartbeat(HB_NETWORK);
            }
        }
        else
        {
            t4gDown = 0;
            if (g4gOn)                          /* 以太网恢复：停 4G 发布（会话自然闲置） */
            {
                g4gOn = 0;
                printf("[4G] channel OFF (eth back)\r\n");
            }
        }
        sh_4gUp = g4gOn;        /* 镜像给显示任务：区分"以太网上云"和"4G 备份上云" */
#endif
        OTA_Poll();                 /* 阶段10.3：OTA 下载超时处理 */
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

            /* 阶段9：链路 DOWN->UP 恢复瞬间，缓存非空则请求补传 */
            if (!link)
                wasDown = 1;
            else if (wasDown)
            {
                wasDown = 0;
                if (DL_Count() > 0 && !OTA_IsBusy())   /* OTA 期间不发起补传 */
                {
                    gReplayReq = 1;
                    printf("replay requested: %u cached\r\n", (unsigned)DL_Count());
                }
            }

            if (!link)
            {
                linkDownCnt++;
                if (linkDownCnt >= 3)
                {
                    linkDownCnt = 0;
                    if (getSn_SR(MQTT_SOCK) != SOCK_CLOSED)
                        close(MQTT_SOCK);      /* 模块在线，close安全 */
                    /* 有 4G 顶着的时候别把"上云正常"判死——这个标志现在含义是
                     * "上云通道是否正常"（以太网或 4G 任一条通即正常），由各自通道维护 */
                    if (!sh_4gUp)
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
                OTA_NotifyAlive();   /* A2：运行正常 → 清 bootloader 待确认计数锁存 */
                /* 阶段10.3：每次上线订阅 OTA 下行主题（断线重连后自动重订阅） */
                MQTT_Subscribe(OTA_TOPIC_CMD, 0);
                MQTT_Subscribe(OTA_TOPIC_FW, 0);
                /* 阶段10.4：新固件首次上线 → 发运行确认（每个版本只发一次） */
                if (OTA_NeedConfirm())
                {
                    char cj[48];
                    int  cjLen = sprintf(cj, "{\"ota\":\"ok\",\"ver\":\"%s\"}", OTA_VER_STR);
                    if (MQTT_Publish(OTA_TOPIC_CMD, (const uint8_t *)cj, (uint16_t)cjLen) == MQTT_OK)
                    {
                        OTA_ConfirmMark();
                        printf("OTA: confirmed ver=%s\r\n", OTA_VER_STR);
                    }
                    else
                        printf("OTA: confirm publish fail\r\n");
                }
            }

            /* ===== 阶段9：SNTP 对时（首次上线立刻尝试，失败60s重试） ===== */
            if (!gTimeSynced && (sntpFirst || (xTaskGetTickCount() - tSntp) >= 60000))
            {
                uint32_t unix;
                sntpFirst = 0;
                tSntp = xTaskGetTickCount();
                Heartbeat(HB_NETWORK);     /* SNTP阻塞<=5s，先打卡再阻塞，避免误报stale */
                printf("SNTP: syncing...\r\n");
                if (SNTP_GetUnix(&unix) == 0)
                {
                    RTC_SetUnix(unix);
                    gTimeSynced = 1;
                    printf("SNTP: synced, unix=%u\r\n", (unsigned)unix);
                }
                else
                    printf("SNTP: fail, will retry\r\n");
            }

            /* ===== 阶段9：补传发布（每条>=20ms节流；失败保留在途下轮重试）
             * OTA 下载期间挂起补传，避免带宽/Flash 并发抢占 ===== */
            if (gReplayReq && !OTA_IsBusy() && (xTaskGetTickCount() - tReplay) >= 20)
            {
                if (!havePend && xQueueReceive(xReplayQueue, &pend, 0) == pdTRUE)
                    havePend = 1;
                if (havePend)
                {
                    rjsonLen  = sprintf(rjson, "{\"device\":\"gw001\",\"replay\":1,\"ts\":%u",
                                        (unsigned)pend.ts);
                    rjsonLen += sprintf(rjson + rjsonLen, ",\"temp\":");
                    rjsonLen += FmtX10(rjson + rjsonLen, pend.t10);
                    rjsonLen += sprintf(rjson + rjsonLen, ",\"hum\":");
                    rjsonLen += FmtX10(rjson + rjsonLen, (int16_t)pend.h10);
                    rjsonLen += sprintf(rjson + rjsonLen, "}");

                    if (MQTT_Publish(MQTT_REPLAY_TOPIC, (const uint8_t *)rjson,
                                     (uint16_t)rjsonLen) == MQTT_OK)
                    {
                        havePend = 0;
                        tReplay = xTaskGetTickCount();
                        replayCnt++;
                        /* 缓存消费（DL_Pop）由存储任务在入队时完成，Flash所有权单一 */
                        if (replayCnt <= 3 || replayCnt % 20 == 0)
                            printf("replay[%u] ts=%u\r\n",
                                   (unsigned)replayCnt, (unsigned)pend.ts);
                    }
                    else
                        printf("replay: publish FAIL\r\n");
                }

                /* 灌完（gDrainDone）且队列尾货发完 -> 补传结束 */
                if (!havePend && gDrainDone && uxQueueMessagesWaiting(xReplayQueue) == 0)
                {
                    gReplayReq = 0;
                    gDrainDone = 0;
                    printf("replay done: %u records resent, %u left\r\n",
                           (unsigned)replayCnt, (unsigned)DL_Count());
                    replayCnt = 0;
                }
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
        else if (sh_mqttOk)
        {
            /* MQTT 掉线：清在线标志，下次上线时重新订阅 OTA 主题 */
            sh_mqttOk = 0;
        }

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
        LCD_DisplayMqtt(sh_mqttOk ? (sh_4gUp ? 2 : 1) : 0);

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

/* 存储任务：消费数据队列；离线写 W25Q64 环形缓存，联网收到补传请求后
 * 把缓存记录灌进补传队列（网络任务负责发布，Flash 和 MQTT socket 职责分离） */
static void vStorageTask(void *pv)
{
    (void)pv;
    DataMsg   m;
    uint32_t  ts, tLog = 0, tFail = 0;
    uint16_t  h10;
    int16_t   t10;
    uint32_t  cacheCnt = 0;            /* 本次离线期间新缓存条数 */
    uint32_t  pumped = 0;              /* 本次补传入队条数 */
    uint32_t  badCnt = 0;              /* 补传时跳过的坏记录条数 */
    uint32_t  drainLoops = 0;          /* drain 循环计数：每64条让CPU一次，防饿死显示任务 */
    uint8_t   flashOk = 0;
    uint8_t   canaryDone = 0;          /* 金丝雀探针：联网后只做一次写读校验 */

    /* Flash 初始化（失败不放弃：每30秒重试，模块后上电也能恢复） */
    while (DL_Init() != W25Q64_OK)
    {
        { uint32_t id = 0; W25Q64_ReadID(&id);
          printf("datalog: W25Q64 not ready (JEDEC=%06X), retry in 30s\r\n", (unsigned)id); }
        Heartbeat(HB_STORAGE);
        vTaskDelay(30000);
    }
    flashOk = 1;
    printf("datalog: ready, %u cached\r\n", (unsigned)DL_Count());

    /* SD 卡按天 CSV 归档：挂载 + 打开当日文件（失败自动降级，60s 重探测） */
    SDLOG_Init();

    for (;;)
    {
        Heartbeat(HB_STORAGE);

        /* 金丝雀探针（踩坑#42 防御）：网线全程不动，联网+对时后主动写一条并回读，
         * 校验后消费掉。OK=写入链路健康；FAIL=写命令被干扰（串口立即报警） */
        if (!canaryDone && sh_link == 1 && gTimeSynced)
        {
            canaryDone = 1;
            if (DL_Append(RTC_GetUnix(), 0, 0) == W25Q64_OK)
            {
                uint32_t pts;
                int16_t  pt;
                uint16_t ph;
                if (DL_Peek(&pts, &pt, &ph) == W25Q64_OK)
                {
                    (void)DL_Pop();          /* 校验完消费掉，不留 0/0 测试记录进补传 */
                    printf("[DL] canary online: OK (ts=%lu)\r\n", (unsigned long)pts);
                }
                else
                {
                    printf("[DL] canary online: PEEK FAIL\r\n");
                }
            }
            else
            {
                printf("[DL] canary online: APPEND FAIL\r\n");
            }
        }

        /* 收采集读数（阻塞500ms，保证心跳节奏） */
        if (xQueueReceive(xDataQueue, &m, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            SDLOG_Log(m.ts, m.t10, m.h10);   /* SD 归档：在线离线都写，失败静默降级 */

            if (sh_link == 0)          /* 离线：写环形缓存 */
            {
                if (flashOk && DL_Append(m.ts, m.t10, m.h10) == W25Q64_OK)
                {
                    cacheCnt++;
                    if ((xTaskGetTickCount() - tLog) >= 30000)
                    {
                        tLog = xTaskGetTickCount();
                        printf("offline caching: +%u this time, %u total\r\n",
                               (unsigned)cacheCnt, (unsigned)DL_Count());
                    }
                }
                else
                {
                    /* 追加失败（EMI毛刺/Flash异常）：读数退回队头下轮重试，打印限流30s */
                    xQueueSendToFront(xDataQueue, &m, 0);
                    if ((xTaskGetTickCount() - tFail) >= 30000)
                    {
                        tFail = xTaskGetTickCount();
                        printf("datalog: append fail, retrying\r\n");
                    }
                }
            }
            /* 在线：实时值由网络任务走共享状态发布，队列读数直接丢弃 */
        }

        /* 补传：网络任务已请求且链路在，把缓存灌进补传队列（OTA 下载期间挂起） */
        if (gReplayReq && sh_link && !gDrainActive && flashOk && !OTA_IsBusy())
        {
            gDrainActive = 1;
            gDrainDone = 0;
            pumped = 0;
            badCnt = 0;
            while (DL_Count() > 0)
            {
                Heartbeat(HB_STORAGE);
                /* 每64条让CPU一次（1ms）：大数据量/清坏记录的自旋循环不能独占CPU，
                 * 否则显示任务(prio2)被饿死→display heartbeat stale→看门狗不吃IWDG→33s复位 */
                if ((++drainLoops & 0x3F) == 0)
                    vTaskDelay(1);
                if (DL_Peek(&ts, &t10, &h10) != W25Q64_OK)
                {
                    badCnt++;
                    DL_Pop();          /* CRC坏/空槽记录：跳过（数量计入drain结束统计） */
                    continue;
                }
                m.ts = ts; m.t10 = t10; m.h10 = h10;
                /* 队列满时等网络任务消费（200ms一拍，期间喂心跳） */
                while (xQueueSend(xReplayQueue, &m, pdMS_TO_TICKS(200)) != pdTRUE)
                {
                    Heartbeat(HB_STORAGE);
                    if (!gReplayReq)   /* 网络任务中途取消（如MQTT长时间离线） */
                        break;
                }
                if (!gReplayReq)
                    break;
                DL_Pop();              /* 入队成功才消费（发布成功由网络任务侧已保证顺序） */
                pumped++;
            }
            /* 空跑（网络任务尚未清标志）不刷屏 */
            if (pumped || badCnt)
                printf("drain done: sent=%u, bad=%u, left=%u\r\n",
                       (unsigned)pumped, (unsigned)badCnt, (unsigned)DL_Count());
            DL_BookmarkCommit();    /* 补传结束：绕过30s限流，把书签无条件落盘 */
            gDrainActive = 0;
            gDrainDone = 1;
            cacheCnt = 0;
        }
    }
}

/* ===== 应用初始化：创建互斥锁 + 全部任务（调度器由 main 启动） ===== */
void APP_Init(void)
{
    /* 共享状态互斥锁 */
    xStateMutex = xSemaphoreCreateMutex();

    /* 心跳基准清零（调度器启动后各任务随即刷新） */
    memset((void *)hbTick, 0, sizeof(hbTick));

    /* 阶段9：数据/补传队列 */
    xDataQueue    = xQueueCreate(16, sizeof(DataMsg));
    xReplayQueue = xQueueCreate(8, sizeof(DataMsg));

    /* 阶段10.3：注册 MQTT 下行回调（OTA 指令/分块） */
    MQTT_SetMsgCb(OTA_OnMqtt);

#if AIR780E_ENABLE
    /* 阶段13.4：把 NET 任务心跳注入 4G 驱动——AT 长阻塞期间由驱动回调喂狗 */
    AIR780E_SetHeartbeatCb(NetHeartbeat);
#endif

    xTaskCreate(vWatchdogTask, "WDG", 128,  NULL, 4, NULL);
    xTaskCreate(vCollectTask,  "COL", 512,  NULL, 3, NULL);
    xTaskCreate(vNetworkTask,  "NET", 512,  NULL, 3, NULL);
    xTaskCreate(vStorageTask,  "STG", 512,  NULL, 3, NULL);
    xTaskCreate(vDisplayTask,  "DSP", 512,  NULL, 2, NULL);
}
