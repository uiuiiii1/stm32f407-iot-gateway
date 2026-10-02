#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "usart.h"
#include "lcd_spi_154.h"
#include "w5500_bsp.h"
#include "socket.h"
#include "modbus.h"
#include "mqtt.h"
#include "app.h"

/*
 * 阶段8：FreeRTOS 应用层 —— 任务、共享状态、心跳
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
 * 相对阶段7裸机版：
 *   - 离线/自愈/防抖逻辑从裸机大循环原样迁入 vNetworkTask，参数不变
 *   - 显示逻辑独立为 vDisplayTask（LCD 只在 DisplayTask 访问，消除跨任务竞争）
 * 注意：printf 为非线程安全，各任务日志频率已压低，偶发交错不影响功能。
 */

/* ===== 共享状态 ===== */
/* 互斥锁：保护下面共享变量，防止多任务同时读写打架 */
static SemaphoreHandle_t xStateMutex;

static volatile int16_t  sh_temp10 = 0;    /* 温度（×10，整数存，比如25.3℃存253） */
static volatile uint16_t sh_hum10 = 0;     /* 湿度（×10） */
static volatile uint8_t  sh_link = 0;      /* 网线状态：1=插着 0=拔了 */
static volatile uint8_t  sh_mqttOk = 0;    /* MQTT状态：1=在线 0=掉线 */
static volatile uint16_t sh_pubCnt = 0;    /* MQTT成功发布次数 */
static volatile uint8_t  sh_modbusOk = 1;  /* 传感器状态：1=正常 0=故障 */
/* ===== 看门狗心跳 ===== */
#define HB_WATCHDOG 0   /* 看门狗任务自己的心跳ID */
#define HB_COLLECT  1   /* 采集任务心跳ID */
#define HB_NETWORK  2   /* 网络任务心跳ID */
#define HB_DISPLAY  3   /* 显示任务心跳ID */
#define HB_COUNT    4   /* 总任务数（数组大小，for循环用） */
#define HB_STALE_MS 3000              /* 心跳超过3秒视为异常（DNS阻塞≤10s由IWDG 24s兜底） */
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

/* ===== 任务 ===== */

/* 看门狗：初始化IWDG（约24s超时），1s巡检心跳；任一任务心跳落后3秒→停喂→系统复位。
 * 调试期用 DBGMCU 冻结 IWDG（断点不停狗），发布版本可注释该行 */
static void vWatchdogTask(void *pv)
{
    (void)pv;
    uint8_t reported = 0;  // 故障打印标记，防止重复刷屏输出报错信息
    DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);  // 调试断点时，停止独立看门狗，防止调试时芯片复位
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  // 解锁IWDG寄存器，允许修改预分频、重载值
    IWDG_SetPrescaler(IWDG_Prescaler_256);    /* LSI 32kHz/256 = 125Hz，设置看门狗时钟预分频 */
    IWDG_SetReload(0xFFF);                    /* 4095/125 ≈ 32.8s 超时，超过这个时间没喂狗就复位 */
    IWDG_ReloadCounter();                     // 第一次喂狗，启动看门狗
    IWDG_Enable();                            // 开启独立硬件看门狗
    for (;;)
    {
        uint32_t now = xTaskGetTickCount();   // 获取当前FreeRTOS系统滴答时间
        uint8_t  i, allAlive = 1;
        // 遍历所有任务心跳记录
        for (i = 0; i < HB_COUNT; i++)
        {
            // 如果某个任务心跳超过阈值，判定任务卡死
            if ((now - hbTick[i]) > HB_STALE_MS)
                allAlive = 0;
        }
        if (allAlive)
        {
            reported = 0;
            IWDG_ReloadCounter();              // 全部任务正常，喂硬件看门狗
        }
        else if (!reported)
        {
            reported = 1;
            printf("watchdog: task heartbeat stale, IWDG not fed\r\n"); // 打印任务卡死提示
        }
        vTaskDelay(1000);                     // 1秒检测一次
    }
}

/* 采集任务：Modbus 2秒轮询读取温湿度，共享变量用互斥量保护 */
static void vCollectTask(void *pv)
{
    (void)pv;                          
    int16_t  t10;                      // 温度 ×10 存储值
    uint16_t h10;                      // 湿度 ×10 存储值
    uint8_t  fail = 0;                 // Modbus读取失败计数
    for (;;)
    {
        // Modbus读取温湿度
        if (MODBUS_ReadTempHum(&t10, &h10) == MB_OK)
        {
            fail = 0;                  // 读取成功，清空失败计数器
            // 获取互斥锁，最多等待100个系统滴答，保护全局共享变量
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                sh_temp10 = t10;       // 更新共享温度
                sh_hum10 = h10;         // 更新共享湿度
                sh_modbusOk = 1;        // 标记Modbus通信正常
                xSemaphoreGive(xStateMutex); // 释放互斥锁，别的任务可以读取
            }
        }
        else
        {
            fail++;                    // 读取失败，失败计数+1
            // 前2次失败每次打印；之后每累计失败10次才打印一次，减少串口刷屏
            if (fail <= 2 || fail % 10 == 0)
                printf("Modbus read fail x%d (keep last)\r\n", fail);
        }
        Heartbeat(HB_COLLECT);         // 采集任务心跳打卡，看门狗检测存活
        vTaskDelay(2000);              // 间隔2秒轮询一次
    }
}


/* 网络任务：W5500健康检查 + MQTT状态机 + 每5秒发布一次温湿度JSON */
static void vNetworkTask(void *pv)
{
    (void)pv;                          // 消除未使用参数警告
    char     json[80];                 // JSON字符串缓冲区，最大80字节
    int      jsonLen;                  // 记录当前JSON已经拼了多少字节
    uint8_t  verFail = 0;              // W5500版本号连续读错计数
    uint32_t tOff = 0;                 // 离线日志上次打印时间，用来限流
    uint8_t  modOk = 1;                // W5500模块在位状态（1=在线 0=离线）
    uint8_t  link = 0;                 // 网线物理链路状态（1=插着 0=拔了）
    uint8_t  linkDownCnt = 0;          // 链路DOWN连续计数，防抖用
    uint8_t  cfgFail = 0;              // 网络配置丢失连续计数，防抖用
    uint32_t tPub = 0;                 // 上次MQTT发布时间
    for (;;)
    {
        /* ========== 第一步：W5500模块在位检测 ==========
         * 连续5次读版本号失败，才判定模块离线（防止SPI偶发干扰误判）
         * 离线期间绝对不能调用socket API，否则读到0xFF会导致close卡死 */
        if (W5500_ReadVersion() != 0x04)   // W5500芯片版本号固定是0x04
        {
            verFail++;
            if (verFail < 5)
            {
                vTaskDelay(10);
                continue;              // 还没连续错5次，继续重试
            }
            if (modOk)
            {
                modOk = 0;
                printf("W5500 module offline (5 consecutive bad version reads)\r\n");
            }
            // 离线日志每2秒打印一次，防止串口疯狂刷屏
            if ((xTaskGetTickCount() - tOff) >= 2000)
            {
                tOff = xTaskGetTickCount();
                printf("module offline, version read: 0x%02X spi1err=%d\r\n",
                       W5500_ReadVersion(), W5500_BSP_SPI1Err());
            }
            vTaskDelay(500);
            continue;                  // 模块离线，跳过后面所有网络操作
        }
        verFail = 0;                   // 读对了，清零失败计数
        if (!modOk)
        {
            modOk = 1;
            printf("W5500 module online\r\n");
        }

        /* ========== 第二步：网络配置自愈 ==========
         * W5500硬件复位后寄存器配置会丢，连续3次检测到配置不一致，重新下发IP/掩码/网关 */
        if (W5500_CheckConfig() == 0)
        {
            if (cfgFail < 3)
                cfgFail++;
            if (cfgFail >= 3)
            {
                cfgFail = 0;
                printf("W5500 config lost (chip reset?) - re-applied\r\n");
                W5500_NetworkInit();   // 重新初始化网络配置
            }
        }
        else
            cfgFail = 0;

        /* ========== 第三步：网线链路状态检测 ==========
         * 连续3次检测到链路DOWN，才关闭MQTT socket（防止网线接触不良频繁重连） */
        {
            uint8_t linkNow = W5500_LinkStatus();
            if (linkNow != link)
            {
                link = linkNow;
                printf("PHY %s\r\n", link ? "LinkUp" : "LinkDown");
            }
            sh_link = link;            // 更新共享链路状态，给显示任务用

            if (!link)
            {
                linkDownCnt++;
                if (linkDownCnt >= 3)
                {
                    linkDownCnt = 0;
                    // 链路断了，关闭MQTT socket（模块在线时调用close是安全的）
                    if (getSn_SR(MQTT_SOCK) != SOCK_CLOSED)
                        close(MQTT_SOCK);
                    sh_mqttOk = 0;     // MQTT肯定也断了
                }
                vTaskDelay(10);
                continue;              // 链路断，不跑MQTT
            }
            linkDownCnt = 0;
        }

        /* ========== 第四步：MQTT状态机 ==========
         * DNS解析、TCP连接、CONNACK、keepalive、自动重连全部封装在MQTT_Process内部，带超时不会死等 */
        if (MQTT_Process() == MQ_MQTT_OK)
        {
            if (!sh_mqttOk)
            {
                sh_mqttOk = 1;
                printf("MQTT: ONLINE\r\n");
            }
            /* 每5秒发布一次温湿度JSON数据 */
            if ((xTaskGetTickCount() - tPub) >= 5000)
            {
                tPub = xTaskGetTickCount();
                // 手动拼接JSON字符串，不用cJSON库省内存，不用%f兼容MicroLIB
                jsonLen = sprintf(json, "{\"device\":\"gw001\",\"temp\":");
                jsonLen += FmtX10(json + jsonLen, sh_temp10);    // 拼温度
                jsonLen += sprintf(json + jsonLen, ",\"hum\":");
                jsonLen += FmtX10(json + jsonLen, (int16_t)sh_hum10); // 拼湿度
                jsonLen += sprintf(json + jsonLen, "}");

                if (MQTT_Publish(MQTT_TOPIC, (const uint8_t *)json, (uint16_t)jsonLen) == MQTT_OK)
                {
                    sh_pubCnt++;        // 发布成功，计数+1
                    printf("PUB[%d]: %s\r\n", sh_pubCnt, json);
                }
            }
        }
        Heartbeat(HB_NETWORK);          // 网络任务心跳打卡
        vTaskDelay(10);                 // 10ms轮询一次，响应快
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
