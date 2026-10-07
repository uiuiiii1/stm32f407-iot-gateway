#include "air780e.h"
#include "usart.h"      /* extern UART_HandleTypeDef huart3（CubeMX 生成，.ioc 已声明 USART3） */
#include "delay.h"
#include <stdio.h>
#include <string.h>

/*============================================================================
  Air780E 驱动（合宙 AT 固件版，HAL 库实现，见 air780e.h 顶部说明）
  - USART3：PC10(TX)/PC11(RX) AF7；115200-8N1（时钟/GPIO 由 MX_USART3_UART_Init 配好）
  - RDY：PC12 输入下拉（模块连上服务器输出 3.3V 高）；RST：PE0 开漏输出，空闲高
  - RX：HAL_UART_Receive_IT 单字节中断 + 4KB 环形缓冲；错误走 HAL_UART_ErrorCallback
  - TX：HAL_UART_Transmit 阻塞发送（AT 命令低频，可接受）
  - AIR780E_At 的捕获缓冲为静态单实例——只允许 NET 任务串行调用
  ============================================================*/

#if AIR780E_ENABLE

#define AIR780E_RX_SIZE     4096UL
#define AIR780E_RX_MASK     (AIR780E_RX_SIZE - 1)

#define AIR780E_BROKER_DEF  "broker.emqx.io"

static char     s_rxRing[AIR780E_RX_SIZE];
static volatile uint16_t s_rxHead = 0;
static volatile uint16_t s_rxTail = 0;
static volatile uint32_t s_rxTotal = 0;     /* 诊断：上电以来收到的总字节数 */

/* 诊断：收帧/线路错误计数——判断"线上到底有没有波形"的硬件级证据（逐位检测，不会漏）：
 *   四项全 0 且 rxTotal=0 → PC11 从头到尾没有一个起始位（模块没发 / RX 线没接到 PC11）
 *   FE/NE 在涨（rxTotal 可能是 0）→ 线上确有电平活动但成帧失败（波特率/电平/接触问题） */
static volatile uint32_t s_errFE = 0;       /* 帧错误：停止位不对 */
static volatile uint32_t s_errNE = 0;       /* 噪声：采样不一致 */
static volatile uint32_t s_errORE = 0;      /* 溢出：上一字节没及时取走 */
static volatile uint32_t s_errPE = 0;       /* 校验错（本工程无校验，正常恒 0） */

/* 心跳回调：AIR780E_MQTTStart 最长阻塞 ~19s，期间必须替 NET 任务喂狗，
 * 否则 app.c 看门狗任务（3s 判陈旧）会打印 "network heartbeat stale" 并停喂 IWDG */
static void (*s_hbCb)(void) = 0;

/* HAL_UART_Receive_IT 的单字节接收位（每收一字节回调里重挂起） */
static uint8_t s_rxByte = 0;

#if AIR780E_RST_ENABLE
static uint32_t s_lastRstTick = 0;          /* 上次硬复位时刻：0=上电后还没按过 */
#endif

void AIR780E_SetHeartbeatCb(void (*cb)(void))
{
    s_hbCb = cb;
}

static void AIR780E_Beat(void)
{
    if (s_hbCb != 0)
    {
        s_hbCb();
    }
}

/*---------------- RX 环形缓冲（RxCpltCallback 调，中断上下文） ----------------*/

void AIR780E_RxPush(uint8_t b)
{
    uint16_t next = (uint16_t)((s_rxHead + 1) & AIR780E_RX_MASK);
    s_rxTotal++;
    if (next == s_rxTail)
    {
        return;                                     /* 满：丢最新（只影响当条命令超时重试） */
    }
    s_rxRing[s_rxHead] = (char)b;
    s_rxHead = next;
}

/* HAL 回调（所有 UART 共用，按实例分流；定义在驱动内以强符号覆盖 __weak 版） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3)
    {
        return;
    }
    AIR780E_RxPush(s_rxByte);
    /* 必须重新挂起下一次接收，否则只收到第一个字节 */
    (void)HAL_UART_Receive_IT(huart, &s_rxByte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3)
    {
        return;
    }
    /* FE/NE/ORE/PE 分别计数（HAL 把错误位集合进 ErrorCode，读一次即得） */
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0) { s_errORE++; }
    if ((huart->ErrorCode & HAL_UART_ERROR_FE)  != 0) { s_errFE++;  }
    if ((huart->ErrorCode & HAL_UART_ERROR_NE)  != 0) { s_errNE++;  }
    if ((huart->ErrorCode & HAL_UART_ERROR_PE)  != 0) { s_errPE++;  }
    /* 错误后 HAL 已停止本次接收：重挂起下一字节（ORE 尤其必须，否则不再收数据） */
    (void)HAL_UART_Receive_IT(huart, &s_rxByte, 1);
}

/*---------------- 初始化 ----------------*/

void AIR780E_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* USART3 的时钟/GPIO/波特率由 MX_USART3_UART_Init（main.c）配好，这里只补引脚与接收 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* RDY：PC12 输入下拉（未接线时读 0=离线，与真实状态一致） */
    gpio.Pin  = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &gpio);

#if AIR780E_RST_ENABLE
    /* 模块 RST：PE0 开漏输出，空闲高（复位无效）。
     * ⚠️ 开漏 = "只拉低、不往上灌电流"——模块 IO 域电平不好确定（1.8V/3.3V 都见过），
     *    推挽强驱高电平万一顶到低压域会从保护二极管倒灌。RST 是输入且内部有上拉，
     *    开漏 + 内部上拉足够；万一你的板子 RST 需要强上拉，把 Mode 改回 OUTPUT_PP。
     * ⚠️ 必须先置 ODR=1 再配成输出：HAL_GPIO_Init 不改 ODR，而复位值是 0，
     *    若先配输出，这一瞬间会把模块直接拉复位——而 AIR780E_Init 每次 4G 启动都调用，
     *    等于每 5s 把模块按一次，它永远起不来 */
    HAL_GPIO_WritePin(AIR780E_RST_PORT, AIR780E_RST_PIN, GPIO_PIN_SET);  /* 先置 ODR=1 */
    gpio.Pin   = AIR780E_RST_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AIR780E_RST_PORT, &gpio);
#endif

    /* USART3 接收中断：抢占优先级 6（数值>5，低于 FreeRTOS 临界优先级）。重复调用幂等 */
    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* 启动单字节中断接收：正在收时返回 HAL_BUSY，忽略即可（回调里已持续重挂起） */
    (void)HAL_UART_Receive_IT(&huart3, &s_rxByte, 1);
}

uint8_t AIR780E_IsOnline(void)
{
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET) ? 1 : 0;
}

/*---------------- 硬复位（救"模块卡死"）----------------
 * 场景：模块 AT 固件卡死/进休眠/误入下载模式时，模块一直有电，
 *       重启 407 是救不了它的——只有把 RST 拉低重来。
 * 节流是必须的：模块重启 + 协议栈就绪要十几秒，若每次 4G 启动失败都按一次，
 *       它会永远停在启动过程中。所以 60s 内只按一次，中间让重试循环去探 AT。 */
#if AIR780E_RST_ENABLE
uint8_t AIR780E_HardReset(void)
{
    uint32_t now = GetTick();
    uint32_t t0;

    if (s_lastRstTick != 0 && (now - s_lastRstTick) < AIR780E_RST_MIN_GAP_MS)
    {
        return 0;                                   /* 节流：模块还在重启，别又按回去 */
    }
    s_lastRstTick = now;

    HAL_GPIO_WritePin(AIR780E_RST_PORT, AIR780E_RST_PIN, GPIO_PIN_RESET);
    t0 = GetTick();
    while ((GetTick() - t0) < AIR780E_RST_MS)       /* 低电平保持期间照常喂狗心跳 */
    {
        AIR780E_Beat();
        Delay_ms(10);
    }
    HAL_GPIO_WritePin(AIR780E_RST_PORT, AIR780E_RST_PIN, GPIO_PIN_SET); /* 释放：模块开始重新启动 */
    return 1;
}
#else
uint8_t AIR780E_HardReset(void)
{
    return 0;                                       /* 未接线：永不动作 */
}
#endif

/*---------------- AT 引擎 ----------------*/

static void AIR780E_SendLine(const char *s)
{
    uint16_t n = (uint16_t)strlen(s);
    if (n != 0)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)s, n, 200);
    }
    HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, 200);
}

uint8_t AIR780E_At(const char *cmd, const char *expect,
                   char *resp, uint16_t respLen, uint32_t timeoutMs)
{
    static char cap[1024];              /* 静态单实例：仅 NET 任务串行调用 */
    uint32_t    capLen = 0;
    uint32_t    t0 = GetTick();

    if (cmd == 0 || expect == 0)
    {
        return AIR780E_ERR_PARAM;
    }

    s_rxTail = s_rxHead;                /* 丢弃旧数据 */
    cap[0] = '\0';

    AIR780E_SendLine(cmd);

    while ((GetTick() - t0) < timeoutMs)
    {
        while (s_rxTail != s_rxHead && capLen < sizeof(cap) - 1)
        {
            cap[capLen++] = s_rxRing[s_rxTail];
            s_rxTail = (uint16_t)((s_rxTail + 1) & AIR780E_RX_MASK);
        }
        cap[capLen] = '\0';

        if (strstr(cap, expect) != 0)
        {
            if (resp != 0 && respLen != 0)
            {
                uint16_t n = (capLen < respLen - 1) ? (uint16_t)capLen : (uint16_t)(respLen - 1);
                memcpy(resp, cap, n);
                resp[n] = '\0';
            }
            return AIR780E_OK;
        }
        AIR780E_Beat();                     /* 长时间等回话时替调用任务喂狗，防看门狗误判 */
        Delay_ms(1);                        /* 让出 CPU：等待期间低优先级任务（显示）得以运行 */
    }

    if (resp != 0 && respLen != 0)      /* 超时也交出已收内容，便于诊断 */
    {
        uint16_t n = (capLen < respLen - 1) ? (uint16_t)capLen : (uint16_t)(respLen - 1);
        memcpy(resp, cap, n);
        resp[n] = '\0';
    }
    return AIR780E_ERR_TIMEOUT;
}

/*---------------- 就绪等待 ----------------*/

uint8_t AIR780E_WaitReady(uint32_t timeoutMs)
{
    uint32_t t0 = GetTick();
    uint8_t  st;

    /* 上电后协议栈就绪需 ~30s（此前 AT 可能 +CME ERROR:4），轮询 AT 握手 */
    while ((GetTick() - t0) < timeoutMs)
    {
        st = AIR780E_At("AT", "OK", 0, 0, 1000);
        if (st == AIR780E_OK)
        {
            return AIR780E_OK;
        }
    }
    return AIR780E_ERR_TIMEOUT;
}

/*---------------- 诊断：发一条 AT，把模块回话的原始字节打出来 ----------------
 * 用途：启动失败时定位故障在哪一根线上——
 *   RDY=0                    → 模块没电/没开机（先查供电）
 *   rxTotal=0 且 heard 0B    → RX 线（PC11←模块TXD）不通，或 TX 线断了模块没收到
 *   heard 里有 41 54(AT回显) → TX/RX 都通，看内容判断 baud/固件
 *   heard 有乱码             → 波特率或电平问题
 *
 * 增强：rxTotal=0 时原来只能看到"0 字节"，无法区分
 *   「线上静默」/「线上有波形但 PC11 收不到」/「PC11 被钉在低电平」。
 * 现在补三样硬证据：
 *   1) FE/NE/ORE/PE 计数（UART 硬件逐位检测，任何起始位/噪声都会被记下来）
 *   2) 2s 内 PC11 原始电平低采样数（AF 模式下 IDR 仍是真实引脚电平）
 *   3) CR1 快照（RE/UE/RXNEIE 是否真的开着）+ SR 快照
 * 判据写在下面打印处。 */
void AIR780E_Diag(void)
{
    char     cap[96];
    uint32_t i, n, cr1, sr;
    uint16_t lowSeen = 0;

    printf("[4G] diag: RDY=%u rxTotal=%lu\r\n",
           (unsigned)AIR780E_IsOnline(), (unsigned long)s_rxTotal);

    /* 先把遗留的 RX/错误标志清干净（读 SR 再读 DR），让下面 2s 成为一段干净的观察窗 */
    (void)huart3.Instance->SR;
    (void)huart3.Instance->DR;

    s_rxTail = s_rxHead;                /* 丢弃旧数据 */
    AIR780E_SendLine("AT");

    /* 2s 观察窗：每 10ms 采样一次 PC11 原始电平（AF 模式下 IDR 仍反映真实引脚电平）。
     * 100Hz 采样抓不到 115200 的位——它只回答"引脚是不是被钉在某个电平"；
     * "线上到底有没有波形"由上面的错误计数器回答（硬件逐位比对，不会漏掉一个起始位）。
     * 注意：反过来说，若线上真的被钉在低电平，UART 会把它当连续帧收成 0x00，
     * rxTotal 一定 > 0 —— 所以 rxTotal=0 基本排除"线被短到地"。 */
    for (i = 0; i < 200; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_RESET)
        {
            lowSeen++;
        }
        AIR780E_Beat();
        Delay_ms(10);                   /* 给模块 2s 回话（含协议栈未就绪时的慢应答） */
    }

    sr  = huart3.Instance->SR;
    cr1 = huart3.Instance->CR1;

    /* 核心判据：
     *   rxTotal=0 且 FE=NE=ORE=PE=0 且 low=0/200
     *        → PC11 全程"静默高"：模块一个起始位都没发出来（或这根线根本没接到 PC11）
     *   FE/NE 计数 > 0（此时 rxTotal 也可能是 0）
     *        → 线上确有真实电平活动但成帧失败：波特率/电平匹配/接触问题
     *   low > 0 → 引脚被钉在低电平（短路，或被别的设备拉死）
     *   CR1 = 0x202C 才是"接收机确实开着"（UE|TE|RE|RXNEIE），对不上就是配置/时钟问题 */
    printf("[4G] diag: CR1=0x%04X SR=0x%04X low=%u/200 FE=%lu NE=%lu ORE=%lu PE=%lu\r\n",
           (unsigned)cr1, (unsigned)sr, (unsigned)lowSeen,
           (unsigned long)s_errFE, (unsigned long)s_errNE,
           (unsigned long)s_errORE, (unsigned long)s_errPE);

    n = 0;
    while (s_rxTail != s_rxHead && n < sizeof(cap))
    {
        cap[n++] = s_rxRing[s_rxTail];
        s_rxTail = (uint16_t)((s_rxTail + 1) & AIR780E_RX_MASK);
    }
    printf("[4G] diag: heard %luB:", (unsigned long)n);
    for (i = 0; i < n && i < 48; i++)
    {
        printf(" %02X", (unsigned char)cap[i]);
    }
    printf("\r\n");

    /* 模块应答正常但 MQTT 起不来时，看这两条：信号强度 0-31（10 以下基本连不上），
     * GPRS 附着 1=已附网。这里 no reply 要和上面那行合起来看：
     *   FE/NE 有值 → 线上有波形（波特率/电平/接触），只是没解析出 OK
     *   全 0 且 low=0/200 → PC11 一直是静默高，模块根本没在发（或这根线没接到 PC11） */
    if (AIR780E_At("AT+CSQ", "OK", cap, sizeof(cap), 2000) == AIR780E_OK)
    {
        printf("[4G] diag: CSQ: %s\r\n", cap);
    }
    else
    {
        printf("[4G] diag: CSQ no reply\r\n");
    }
    if (AIR780E_At("AT+CGATT?", "+CGATT", cap, sizeof(cap), 2000) == AIR780E_OK)
    {
        printf("[4G] diag: CGATT: %s\r\n", cap);
    }
    else
    {
        printf("[4G] diag: CGATT no reply\r\n");
    }
}

/*---------------- MQTT 会话（实机验证过的 M 命令序列） ----------------*/

uint8_t AIR780E_MQTTStart(const char *broker, uint16_t port)
{
    char cmd[96];
    char resp[96];
    uint8_t st = AIR780E_ERR_FAIL;
    uint8_t attempt;

    if (broker == 0)
    {
        return AIR780E_ERR_PARAM;
    }

    /* 模块的 socket/MQTT 会话不随 407 复位而复位（模块一直有电）：407 重启/听不到之后，
     * 模块可能还挂着上次的会话——MIPSTART 会被顶回 "ALREADY CONNECT"（实测重连死循环）。
     * 对策：MIPSTART 没拿到 CONNECT OK（含 ALREADY CONNECT 残局）→ 先 MDISCONNECT+MIPCLOSE
     * 清残局再重试一次。只有失败路径才付这个代价，正常启动不变慢。 */
    for (attempt = 0; attempt < 2; attempt++)
    {
        snprintf(cmd, sizeof(cmd), "AT+MIPSTART=%s,%u", broker, (unsigned)port);
        st = AIR780E_At(cmd, "OK", resp, sizeof(resp), 3000);
        if (st == AIR780E_OK)
        {
            if (strstr(resp, "CONNECT OK") != 0)
            {
                break;                          /* ACK + URC 都到了：TCP 已连上 */
            }
            if (AIR780E_At("", "CONNECT OK", resp, sizeof(resp), 5000) == AIR780E_OK)
            {
                break;                          /* ACK 先到、URC 后到 */
            }
        }
        printf("[4G] MIPSTART no CONNECT OK (heard: %s), teardown old session, retry\r\n", resp);
        (void)AIR780E_At("AT+MDISCONNECT", "OK", 0, 0, 1500);
        (void)AIR780E_At("AT+MIPCLOSE",    "OK", 0, 0, 1500);
    }
    if (st != AIR780E_OK || strstr(resp, "CONNECT OK") == 0)
    {
        printf("[4G] MIPSTART FAIL, heard: %s\r\n", resp);
        return AIR780E_ERR_FAIL;
    }

    /* MQTT 握手：clean session=1，keepalive=300s */
    if (AIR780E_At("AT+MCONNECT=1,300", "CONNACK OK", resp, sizeof(resp), 8000) != AIR780E_OK)
    {
        printf("[4G] MCONNECT no CONNACK, heard: %s\r\n", resp);
        return AIR780E_ERR_FAIL;
    }
    return AIR780E_OK;
}

uint8_t AIR780E_MQTTPublish(const char *topic, const char *payload)
{
    /* 静态单实例：只允许 NET 任务调用（同 AIR780E_At）。
     * 放栈上的话这一帧要吃掉 672B，NET 任务栈只有 2KB */
    static char cmd[512];
    static char resp[160];
    uint8_t st;

    if (topic == 0 || payload == 0 || strlen(payload) > 400)
    {
        return AIR780E_ERR_PARAM;
    }

    /* ⚠️ <message> 必须用双引号括住（合宙 Air780E AT 手册 MPUB 明写"须用双引号括住"）。
     * 少了这对引号模块直接回 ERROR——而且这也是"载荷里的逗号不会被当成参数分隔符"
     * 的原因：引号内部的逗号属于消息内容。2026-10-07 实测 PUB FAIL 就是这个。 */
    snprintf(cmd, sizeof(cmd), "AT+MPUB=%s,0,0,\"%s\"", topic, payload);

    st = AIR780E_At(cmd, "OK", resp, sizeof(resp), 3000);
    if (st != AIR780E_OK)
    {
        /* 把模块原话打出来：有 ERROR 就是语法/会话问题，一字不回才是链路问题。
         * resp 够长（160B）能同时装下命令回显和紧跟其后的 ERROR */
        printf("[4G] MPUB no OK (err=%u), heard: %s\r\n", (unsigned)st, resp);
    }
    return st;
}

uint8_t AIR780E_MQTTIsConnected(void)
{
    char resp[32];
    if (AIR780E_At("AT+MQTTSTATU", "+MQTTSTATU", resp, sizeof(resp), 2000) != AIR780E_OK)
    {
        return AIR780E_ERR_TIMEOUT;     /* 查询失败按未连接处理 */
    }
    return (strstr(resp, ":1") != 0) ? 1 : 0;
}

/* <message> 内容里的特殊字符按合宙规范转义（消息本身的双引号由 MQTTPublish 加）：
 *   "  → \22    回车 → \0D    换行 → \0A    反斜杠 → \5C
 * （C 源码写 "\\22"，线上是 4 个字符 \ 2 2；漏转义任一字符都会让模块解析错位） */
uint8_t AIR780E_MQTTPublishJson(const char *topic, const char *json)
{
    char esc[512];
    uint16_t i = 0, o = 0;

    if (json == 0)
    {
        return AIR780E_ERR_PARAM;
    }
    while (json[i] != '\0' && o < sizeof(esc) - 4)
    {
        switch (json[i])
        {
            case '"':   esc[o++] = '\\'; esc[o++] = '2'; esc[o++] = '2'; break;
            case '\r':  esc[o++] = '\\'; esc[o++] = '0'; esc[o++] = 'D'; break;
            case '\n':  esc[o++] = '\\'; esc[o++] = '0'; esc[o++] = 'A'; break;
            case '\\':  esc[o++] = '\\'; esc[o++] = '5'; esc[o++] = 'C'; break;
            default:    esc[o++] = json[i];                              break;
        }
        i++;
    }
    esc[o] = '\0';
    return AIR780E_MQTTPublish(topic, esc);
}

#else /* !AIR780E_ENABLE：全短路，零开销 */

void    AIR780E_Init(void)   { (void)0; }
uint8_t AIR780E_IsOnline(void)        { return 0; }
uint8_t AIR780E_WaitReady(uint32_t timeoutMs) { (void)timeoutMs; return AIR780E_ERR_DISABLED; }
void    AIR780E_Diag(void)            { }
void    AIR780E_SetHeartbeatCb(void (*cb)(void)) { (void)cb; }
uint8_t AIR780E_HardReset(void)       { return 0; }
uint8_t AIR780E_MQTTStart(const char *broker, uint16_t port)
{
    (void)broker; (void)port; return AIR780E_ERR_DISABLED;
}
uint8_t AIR780E_MQTTPublish(const char *topic, const char *payload)
{
    (void)topic; (void)payload; return AIR780E_ERR_DISABLED;
}
uint8_t AIR780E_MQTTIsConnected(void) { return 0; }
uint8_t AIR780E_At(const char *cmd, const char *expect,
                   char *resp, uint16_t respLen, uint32_t timeoutMs)
{
    (void)cmd; (void)expect; (void)resp; (void)respLen; (void)timeoutMs;
    return AIR780E_ERR_DISABLED;
}

#endif /* AIR780E_ENABLE */
