/*
 * mqtt.c —— 精简 MQTT 3.1.1 客户端（QoS0），基于 ioLibrary socket API
 *
 * 协议要点：报文 = 固定头(1B) + 剩余长度(1~4B变长) + 载荷
 *   CONNECT 0x10 | CONNACK 0x20 02 00 00（第3字节非0=拒绝）| PUBLISH 0x30（QoS0无报文ID）
 *   PINGREQ C0 00 | DISCONNECT E0 00
 * 所有等待均带超时，绝不死等；断线/拒绝自动 close 回 MQ_IDLE，重连间隔 >= 2s。
 */
#include <stdio.h>
#include <string.h>
#include "mqtt.h"
#include "socket.h"
#include "delay.h"
#if MQTT_USE_DNS
#include "dns.h"
#endif

/* ---------- 内部状态 ---------- */
static MQTT_State state = MQ_IDLE;
static uint8_t  brokerIp[4] = MQTT_BROKER_IP;   /* 当前使用的 broker IP（DNS 成功后覆盖） */
static uint16_t localPort = 2000;               /* 本地临时端口，每次重连递增避免 TIME_WAIT 复用 */
static uint32_t tPing = 0;                      /* 上次 PINGREQ 时刻 */
static uint32_t tRetry = 0;                     /* 上次连接尝试时刻（重连间隔控制） */
static uint8_t  subState = 0;                   /* MQ_TCP_OK 内子步骤：0=建TCP 1=等CONNACK 2=轮询连接 */
static uint32_t tStep = 0;                      /* 子步骤起始时刻（超时用） */
static uint32_t tConnWait = 0;                  /* 发起连接时刻（轮询超时用） */
static uint16_t pingCnt = 0;                    /* PINGREQ 计数（串口打印用） */
static uint8_t  s_sendFail = 0;                 /* 连续 send 失败计数（≥阈值才断线重连，防公网瞬时抖动风暴） */

/* 连续发送失败多少次才判连接死亡并重连（本地 broker 无感；公网高峰防重连风暴） */
#define MQTT_SEND_FAIL_LIMIT   3

/* ---------- 下行接收（阶段10.3 新增） ---------- */
static MQTT_MsgCb msgCb = 0;                    /* 下行 PUBLISH 回调（OTA 用） */
static uint16_t   subPid = 0x0001;              /* SUBSCRIBE 报文 ID，递增 */
static uint8_t    mqttRx[1280];                 /* 下行流缓冲（最大单包：512B hex + JSON ≈ 1060B） */
static uint16_t   mqttRxLen = 0;                /* 已缓冲字节数 */

static void MQTT_Pump(void);                    /* 下行流解析（定义在文件后部） */

#if MQTT_USE_DNS
static uint8_t  dnsBuf[512];                    /* DNS 模块工作缓冲区（DNS_init 要求） */
static uint8_t  dnsOk = 0;                      /* 域名解析成功标志 */
#endif

/* ---------- 剩余长度变长编码：返回编码字节数 ----------
 * len<128 直接1字节；更大则每字节低7位 + 进位标志(0x80)，先写低位 */
static uint8_t MQTT_EncodeLength(uint32_t len, uint8_t *buf)
{
    uint8_t n = 0;
    do {
        uint8_t d = len % 128;
        len /= 128;
        if (len > 0) d |= 0x80;
        buf[n++] = d;
    } while (len > 0 && n < 4);
    return n;
}

/* 发送完整缓冲区（send 返回值不足视为失败） */
static uint8_t SendAll(const uint8_t *buf, uint16_t len)
{
    int32_t r = send(MQTT_SOCK, (uint8_t *)buf, len);
    return (r == (int32_t)len) ? 1 : 0;
}

/* ---------- 初始化 ---------- */
void MQTT_Init(void)
{
    state = MQ_IDLE;
    tRetry = 0;
#if MQTT_USE_DNS
    dnsOk = 0;
    DNS_init(MQTT_DNS_SOCK, dnsBuf);
    printf("MQTT: init, sock=%d, dns sock=%d, domain=%s\r\n",
           MQTT_SOCK, MQTT_DNS_SOCK, MQTT_DOMAIN);
#else
    printf("MQTT: init, sock=%d, static IP %d.%d.%d.%d:%d\r\n", MQTT_SOCK,
           brokerIp[0], brokerIp[1], brokerIp[2], brokerIp[3], MQTT_BROKER_PORT);
#endif
}

/* ---------- MQTT_Process：主循环每轮调用 ---------- */
uint8_t MQTT_Process(void)
{
    switch (state)
    {
    case MQ_IDLE:
        /* 重连间隔控制：公共服务器禁止高频重连，>=2s */
        if ((uint32_t)(GetTick() - tRetry) < 2000)
            break;
        tRetry = GetTick();
        subState = 0;
        tStep = GetTick();
        state = MQ_TCP_OK;
        /* fallthrough：立即进入 MQ_TCP_OK 首步，节省一轮 */

    case MQ_TCP_OK:
        if (subState == 0)          /* 步骤1：建立 TCP 连接 */
        {
            if (getSn_SR(MQTT_SOCK) == SOCK_CLOSED)
            {
#if MQTT_USE_DNS
                /* 首次或解析失败时重解析（DNS_run 为阻塞式但内部带超时） */
                if (!dnsOk)
                {
                    uint8_t dnsSrv[4] = MQTT_DNS_SERVER;
                    uint8_t resolved[4];
                    printf("MQTT: DNS resolve %s ...\r\n", MQTT_DOMAIN);
                    if (DNS_run(dnsSrv, (uint8_t *)MQTT_DOMAIN, resolved) == 1)
                    {
                        memcpy(brokerIp, resolved, 4);
                        dnsOk = 1;
                        printf("MQTT: DNS ok -> %d.%d.%d.%d\r\n",
                               resolved[0], resolved[1], resolved[2], resolved[3]);
                    }
                    else
                    {
                        printf("MQTT: DNS fail, fallback to static IP\r\n");
                        /* 保留 brokerIp 的硬编码初值，直接用方案1的IP尝试 */
                    }
                }
#endif
                localPort++;
                if (socket(MQTT_SOCK, Sn_MR_TCP, localPort, 0x00) != MQTT_SOCK)
                {
                    printf("MQTT: socket() fail\r\n");
                    close(MQTT_SOCK);
                    state = MQ_IDLE;
                    break;
                }
                /* 非阻塞发起连接：设目标/端口/CR_CONNECT，随后在 subState==2 逐轮轮询。
                 * 不用阻塞 connect()——对端不响应时 W5500 默认重试极久（~20s+），
                 * 会卡死网络任务（心跳 stale→看门狗）且 sh_mqttOk 来不及清（LCD 不变红） */
                setSn_DIPR(MQTT_SOCK, brokerIp);
                setSn_DPORTR(MQTT_SOCK, MQTT_BROKER_PORT);
                setSn_CR(MQTT_SOCK, Sn_CR_CONNECT);
                subState = 2;
                tConnWait = GetTick();
            }
            else if (getSn_SR(MQTT_SOCK) == SOCK_ESTABLISHED)
            {
                /* TCP 已连上：组装并发 CONNECT */
                const char cid[] = MQTT_CLIENTID;
                uint8_t pkt[64];
                uint8_t var[10] = {0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04,
                                   0x02,                      /* Clean Session */
                                   (uint8_t)(MQTT_KEEPALIVE >> 8),
                                   (uint8_t)(MQTT_KEEPALIVE & 0xFF)};
                uint16_t cidLen = (uint16_t)strlen(cid);
                uint16_t rem = (uint16_t)(sizeof(var) + 2 + cidLen);
                uint16_t p = 0;
                uint8_t n;

                pkt[p++] = 0x10;                    /* CONNECT 固定头 */
                n = MQTT_EncodeLength(rem, pkt + p);
                p = (uint16_t)(p + n);
                memcpy(pkt + p, var, sizeof(var));  p += sizeof(var);
                pkt[p++] = (uint8_t)(cidLen >> 8);
                pkt[p++] = (uint8_t)(cidLen & 0xFF);
                memcpy(pkt + p, cid, cidLen);       p += cidLen;

                if (!SendAll(pkt, p))
                {
                    printf("MQTT: CONNECT send fail\r\n");
                    close(MQTT_SOCK);
                    state = MQ_IDLE;
                    break;
                }
                subState = 1;
                tStep = GetTick();
            }
            else if ((uint32_t)(GetTick() - tStep) > MQTT_TCP_TIMEOUT)
            {
                /* connect 超时（目标不可达/被拒），关闭重试 */
                printf("MQTT: TCP connect timeout (sr=%d)\r\n", getSn_SR(MQTT_SOCK));
                close(MQTT_SOCK);
                state = MQ_IDLE;
            }
            break;
        }
        else if (subState == 2)          /* 轮询连接完成（每轮非阻塞检查，不卡任务） */
        {
            if (getSn_SR(MQTT_SOCK) == SOCK_ESTABLISHED)
            {
                /* TCP 已连上：组装并发 CONNECT（逻辑与 subState==0 的 ESTABLISHED 分支相同） */
                const char cid[] = MQTT_CLIENTID;
                uint8_t pkt[64];
                uint8_t var[10] = {0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04,
                                   0x02,                      /* Clean Session */
                                   (uint8_t)(MQTT_KEEPALIVE >> 8),
                                   (uint8_t)(MQTT_KEEPALIVE & 0xFF)};
                uint16_t cidLen = (uint16_t)strlen(cid);
                uint16_t rem = (uint16_t)(sizeof(var) + 2 + cidLen);
                uint16_t p = 0;
                uint8_t n;

                pkt[p++] = 0x10;
                n = MQTT_EncodeLength(rem, pkt + p);
                p = (uint16_t)(p + n);
                memcpy(pkt + p, var, sizeof(var));  p += sizeof(var);
                pkt[p++] = (uint8_t)(cidLen >> 8);
                pkt[p++] = (uint8_t)(cidLen & 0xFF);
                memcpy(pkt + p, cid, cidLen);       p += cidLen;

                if (!SendAll(pkt, p))
                {
                    printf("MQTT: CONNECT send fail\r\n");
                    close(MQTT_SOCK);
                    state = MQ_IDLE;
                    break;
                }
                subState = 1;
                tStep = GetTick();
            }
            else if (getSn_SR(MQTT_SOCK) == SOCK_CLOSED ||
                     (getSn_IR(MQTT_SOCK) & Sn_IR_TIMEOUT))
            {
                setSn_IR(MQTT_SOCK, Sn_IR_TIMEOUT);
                printf("MQTT: connect() reject (sr=%d)\r\n", getSn_SR(MQTT_SOCK));
                close(MQTT_SOCK);
                state = MQ_IDLE;
            }
            else if ((uint32_t)(GetTick() - tConnWait) > MQTT_TCP_TIMEOUT)
            {
                printf("MQTT: TCP connect timeout (sr=%d)\r\n", getSn_SR(MQTT_SOCK));
                close(MQTT_SOCK);
                state = MQ_IDLE;
            }
            break;
        }

        /* subState==1：步骤2：等 CONNACK（带超时） */
        {
            uint8_t rxBuf[8];
            int32_t r = recv(MQTT_SOCK, rxBuf, sizeof(rxBuf));
            if (r >= 4 && rxBuf[0] == 0x20)
            {
                if (rxBuf[3] == 0x00)               /* 第3字节=0x00 接受 */
                {
                    state = MQ_MQTT_OK;
                    s_sendFail = 0;
                    tPing = GetTick();
                    pingCnt = 0;
                    printf("MQTT: CONNECTED to %d.%d.%d.%d (clientid=%s)\r\n",
                           brokerIp[0], brokerIp[1], brokerIp[2], brokerIp[3],
                           MQTT_CLIENTID);
                }
                else
                {
                    printf("MQTT: CONNACK refused, reason=0x%02X\r\n", rxBuf[3]);
                    close(MQTT_SOCK);
                    state = MQ_IDLE;
                }
            }
            else if (r < 0 || (uint32_t)(GetTick() - tStep) > MQTT_CONN_TIMEOUT)
            {
                printf("MQTT: CONNACK wait timeout (r=%d)\r\n", (int)r);
                close(MQTT_SOCK);
                state = MQ_IDLE;
            }
        }
        break;

    case MQ_MQTT_OK:
        /* TCP 断开监视：变回CLOSED/CLOSE_WAIT 说明服务器断开或网络故障 */
        if (getSn_SR(MQTT_SOCK) != SOCK_ESTABLISHED)
        {
            printf("MQTT: connection lost (sr=%d), reconnecting\r\n", getSn_SR(MQTT_SOCK));
            mqttRxLen = 0;               /* 清下行缓冲，防旧数据串包 */
            close(MQTT_SOCK);
            state = MQ_IDLE;
            break;
        }
        /* 处理下行消息（PUBLISH/SUBACK/PINGRESP） */
        MQTT_Pump();
        /* keepalive：每 30s 发一次 PINGREQ（keepalive=60s，超90s不发会被踢） */
        if ((uint32_t)(GetTick() - tPing) >= (MQTT_KEEPALIVE * 1000 / 2))
        {
            uint8_t ping[2] = {0xC0, 0x00};
            if (SendAll(ping, 2))
            {
                s_sendFail = 0;
                tPing = GetTick();
                pingCnt++;
                if (pingCnt % 4 == 1)   /* 2分钟打印一次，刷屏别太凶 */
                    printf("MQTT: PINGREQ #%d\r\n", pingCnt);
            }
            else
            {
                /* 瞬时失败不立刻断线：连续 MQTT_SEND_FAIL_LIMIT 次才判死重连 */
                if (++s_sendFail >= MQTT_SEND_FAIL_LIMIT)
                {
                    printf("MQTT: PINGREQ send fail x%d (sr=%d)\r\n",
                           s_sendFail, getSn_SR(MQTT_SOCK));
                    close(MQTT_SOCK);
                    state = MQ_IDLE;
                }
                else
                    printf("MQTT: PINGREQ send fail x%d (transient)\r\n", s_sendFail);
            }
        }
        break;

    default:
        state = MQ_IDLE;
        break;
    }
    return (uint8_t)state;
}

/* ---------- 发布 PUBLISH（QoS0） ---------- */
uint8_t MQTT_Publish(const char *topic, const uint8_t *payload, uint16_t len)
{
    uint8_t  pkt[256];
    uint16_t topicLen;
    uint32_t rem;
    uint16_t p = 0;
    uint8_t  n;

    if (state != MQ_MQTT_OK)
        return MQTT_ERR_OFFLINE;
    if (topic == 0 || payload == 0)
        return MQTT_ERR_PARAM;

    topicLen = (uint16_t)strlen(topic);
    rem = (uint32_t)topicLen + 2 + len;
    if (rem > 200)                      /* 缓冲区上限内留余量 */
        return MQTT_ERR_LEN;

    pkt[p++] = 0x30;                    /* PUBLISH 固定头，QoS0 无报文ID */
    n = MQTT_EncodeLength(rem, pkt + p);
    p = (uint16_t)(p + n);
    pkt[p++] = (uint8_t)(topicLen >> 8);
    pkt[p++] = (uint8_t)(topicLen & 0xFF);
    memcpy(pkt + p, topic, topicLen);   p += topicLen;
    memcpy(pkt + p, payload, len);      p = (uint16_t)(p + len);

    if (!SendAll(pkt, p))
    {
        /* 瞬时失败不立刻断线：连续 MQTT_SEND_FAIL_LIMIT 次才判死重连（公网防重连风暴） */
        if (++s_sendFail >= MQTT_SEND_FAIL_LIMIT)
        {
            printf("MQTT: PUBLISH send fail x%d (sr=%d)\r\n",
                   s_sendFail, getSn_SR(MQTT_SOCK));
            close(MQTT_SOCK);
            state = MQ_IDLE;
        }
        return MQTT_ERR_SEND;
    }
    s_sendFail = 0;
    return MQTT_OK;
}

/* ---------- 主动优雅断开 ---------- */
void MQTT_Disconnect(void)
{
    uint8_t pkt[2] = {0xE0, 0x00};
    if (getSn_SR(MQTT_SOCK) == SOCK_ESTABLISHED)
        SendAll(pkt, 2);
    close(MQTT_SOCK);
    state = MQ_IDLE;
    printf("MQTT: disconnected\r\n");
}

/* ---------- 订阅 ---------- */
void MQTT_Subscribe(const char *topic, uint8_t qos)
{
    uint8_t  pkt[512];
    uint16_t tl = (uint16_t)strlen(topic);
    uint16_t p = 0, n;
    uint32_t rem = (uint32_t)(2 + 2 + tl + 1);   /* pid + topic + qos */

    if (state != MQ_MQTT_OK)
        return;

    pkt[p++] = 0x82;                            /* SUBSCRIBE 固定头（QoS>=1） */
    n = MQTT_EncodeLength(rem, pkt + p);  p += n;
    pkt[p++] = (uint8_t)(subPid >> 8);
    pkt[p++] = (uint8_t)(subPid & 0xFF);
    subPid++;
    pkt[p++] = (uint8_t)(tl >> 8);
    pkt[p++] = (uint8_t)(tl & 0xFF);
    memcpy(pkt + p, topic, tl);          p += tl;
    pkt[p++] = qos;

    if (!SendAll(pkt, p))
    {
        printf("MQTT: SUBSCRIBE send fail (sr=%d)\r\n", getSn_SR(MQTT_SOCK));
        close(MQTT_SOCK);
        state = MQ_IDLE;
        return;
    }
    printf("MQTT: SUBSCRIBE %s (qos%d)\r\n", topic, qos);
}

void MQTT_SetMsgCb(MQTT_MsgCb cb)
{
    msgCb = cb;
}

/* ---------- 下行流解析（阶段10.3）----------
 * 在线期每轮调：收可用数据进缓冲，逐个解析完整 MQTT 报文
 * 支持：PUBLISH(QoS0) 分发给回调 / SUBACK 标记订阅成功 / PINGRESP 忽略
 * 非阻塞：先查 RX_RSR 再 recv，避免 recv 阻塞任务 */
static void MQTT_Pump(void)
{
    uint16_t off = 0;

    /* 读可用数据进缓冲 */
    while ((uint16_t)getSn_RX_RSR(MQTT_SOCK) > 0 && mqttRxLen < sizeof(mqttRx))
    {
        int32_t n = recv(MQTT_SOCK, mqttRx + mqttRxLen, (uint16_t)(sizeof(mqttRx) - mqttRxLen));
        if (n <= 0)
            break;
        mqttRxLen = (uint16_t)(mqttRxLen + n);
    }

    /* 解析：固定头(1B)+剩余长度可变+载荷 */
    while ((uint16_t)(mqttRxLen - off) >= 2)
    {
        uint8_t  type = (uint8_t)(mqttRx[off] >> 4);
        uint32_t rl = 0;
        uint16_t p = (uint16_t)(off + 1);
        uint8_t  shift = 0;
        uint8_t  incomplete = 0;
        uint32_t total;

        while (shift < 28)
        {
            if (p >= mqttRxLen) { incomplete = 1; break; }
            rl |= (uint32_t)(mqttRx[p] & 0x7F) << shift;
            if (!(mqttRx[p] & 0x80)) break;
            p++;
            shift += 7;
        }
        if (incomplete)
            break;                         /* 剩余长度字节没收全 */
        p++;
        total = (uint32_t)(p - off) + rl;
        if ((uint32_t)mqttRxLen - off < total)
            break;                         /* 载荷没收全，留在缓冲等下一轮 */

        switch (type)
        {
        case 0x03:  /* PUBLISH（QoS0 无报文ID） */
            if (rl >= 2)
            {
                uint16_t tl = (uint16_t)(((uint16_t)mqttRx[p] << 8) | mqttRx[p + 1]);
                if (rl >= (uint32_t)tl + 2)
                {
                    const uint8_t *topic = mqttRx + p + 2;
                    const uint8_t *payload = topic + tl;
                    uint16_t plen = (uint16_t)(rl - (uint32_t)tl - 2);
                    if (msgCb)
                        msgCb(topic, tl, payload, plen);
                }
            }
            break;
        case 0x09:  /* SUBACK */
            printf("[RX] SUBACK\r\n");
            break;
        case 0x0D:  /* PINGRESP */
            break;
        default:    /* 其它：丢弃载荷 */
            break;
        }
        off = (uint16_t)(off + total);
    }

    /* 消费掉已解析部分 */
    if (off > 0)
    {
        memmove(mqttRx, mqttRx + off, mqttRxLen - off);
        mqttRxLen = (uint16_t)(mqttRxLen - off);
    }
}
