#ifndef __MQTT_H
#define __MQTT_H

#include "main.h"     /* HAL 版：由 CubeMX 生成，含 stm32f4xx_hal.h 与引脚标签宏 */

/*
 * 阶段6：精简 MQTT 3.1.1 客户端（QoS0，基于 W5500 硬件TCP/IP socket）
 * 报文只实现 5 种：CONNECT / CONNACK(解析) / PUBLISH(QoS0) / PINGREQ / DISCONNECT
 * 状态机：MQ_IDLE -> MQ_TCP_OK -> MQ_MQTT_OK，断线自动回 MQ_IDLE 重连（间隔>=2s）
 */

/* ===== 配置区 ===== */
/* Broker 模式选择（测试用，快速切换）：
 *   0 = 公网 broker.emqx.io（DNS 解析，公共服务器，高峰爱断线）
 *   1 = 本地/局域网 broker（静态 IP 直连，不开 DNS；电脑跑 amqtt，见 start_broker.cmd）
 * 公网恢复后改回 0 即可。 */
#define MQTT_LOCAL_BROKER   1

#if MQTT_LOCAL_BROKER
#define MQTT_BROKER_IP      {192, 168, 0, 106}        /* 电脑局域网 IP（amqtt 本地 broker；DHCP 变化后同步改这里） */
#define MQTT_USE_DNS        0
#else
/* 方案1：nslookup broker.emqx.io 查到的 IP（2026-10-01 查询，CNAME=prod-blue.public-broker.com） */
#define MQTT_BROKER_IP      {44, 232, 241, 40}
#define MQTT_USE_DNS        1
#endif
#define MQTT_BROKER_PORT    1883
#define MQTT_CLIENTID       "gw001-lxb407"          /* 公共服务器必须全局唯一，冲突会被互踢 */
#define MQTT_TOPIC          "gateway/gw001/data"
#define MQTT_REPLAY_TOPIC   "gateway/gw001/replay"   /* 阶段9：断网缓存补传专用主题 */
#define MQTT_KEEPALIVE      60                      /* 秒；PINGREQ 周期 = KEEPALIVE/2 = 30s */

#define MQTT_SOCK           1                       /* MQTT 占用 Socket 编号（DNS 用 2，已错开） */
#define MQTT_DNS_SOCK       2                       /* DNS 查询占用 Socket 编号 */

/* 方案2：1=连接前用 DNS 模块解析域名（失败自动回退硬编码IP）；0=只用硬编码IP（本地模式不开） */
#define MQTT_DOMAIN         "broker.emqx.io"
#define MQTT_DNS_SERVER     {192, 168, 0, 1}        /* DNS 服务器 = 局域网网关 */

/* TCP 连接/CONNACK 等待超时（毫秒） */
#define MQTT_TCP_TIMEOUT    5000
#define MQTT_CONN_TIMEOUT   5000

/* 返回值：0=成功，非0=错误（MB 风格） */
#define MQTT_OK             0
#define MQTT_ERR_PARAM      1   /* 参数错误 */
#define MQTT_ERR_OFFLINE    2   /* 当前不在线（未完成 CONNECT） */
#define MQTT_ERR_SEND       3   /* socket 发送失败/未发完 */
#define MQTT_ERR_LEN        4   /* 载荷超长 */

/* 连接状态 */
typedef enum {
    MQ_IDLE = 0,    /* 空闲/待重连（内部处理重连间隔） */
    MQ_TCP_OK,      /* TCP 已连上，CONNECT/CONNACK 进行中 */
    MQ_MQTT_OK      /* MQTT 在线，可发布 */
} MQTT_State;

/* 初始化：记录 broker 配置，分配 Socket（不触碰任何 socket API，
 * 模块在位检测由 main.c 负责，在线才允许调用 MQTT_Process） */
void     MQTT_Init(void);

/* 状态机：主循环每轮调用。
 * 内部负责：DNS解析(可选) -> TCP连接(SOCK_INIT->connect->ESTABLISHED) ->
 * 发CONNECT等CONNACK(带超时) -> 在线期每30s发PINGREQ、监视TCP断开自动重连
 * 返回：MQ_MQTT_OK=在线；MQ_IDLE/MQ_TCP_OK=当前所处状态 */
uint8_t  MQTT_Process(void);

/* 在线时发布一条 PUBLISH（QoS0，无报文ID）
 * 返回：MQTT_OK=成功；其余错误码见上 */
uint8_t  MQTT_Publish(const char *topic, const uint8_t *payload, uint16_t len);

/* 订阅主题（SUBSCRIBE，QoS0/1），需 MQTT 在线时调用 */
void     MQTT_Subscribe(const char *topic, uint8_t qos);

/* 下行消息回调：收到 PUBLISH 时回调（topic 带长度，payload 为载荷数据） */
typedef void (*MQTT_MsgCb)(const uint8_t *topic, uint16_t topicLen,
                           const uint8_t *payload, uint16_t len);
void     MQTT_SetMsgCb(MQTT_MsgCb cb);

/* 主动优雅断开（发 DISCONNECT 后关 socket，回 MQ_IDLE） */
void     MQTT_Disconnect(void);

/* 优雅断开报文：E0 00（2字节） */
#define MQTT_PACKET_DISCONNECT  0xE0

#endif /* __MQTT_H */
