#ifndef __SNTP_H
#define __SNTP_H

#include "stm32f4xx.h"

/*
 * 阶段9：SNTP 网络对时（UDP，Socket 3）
 * 电池未到货前 RTC 断电丢时间，联网后用 SNTP 拉取 UTC 时间写入 RTC；
 * 采集/缓存记录的时间戳都以此为准。服务器 ntp.aliyun.com（国内直连快），
 * DNS 解析失败回退硬编码 IP。
 */

#define SNTP_SOCK          3                          /* MQTT=1, DNS=2, SNTP=3 */
#define SNTP_DOMAIN        "ntp.aliyun.com"
#define SNTP_SERVER_IP     {203, 107, 6, 88}          /* DNS 失败时的回退 IP */
#define SNTP_PORT          123
#define SNTP_TIMEOUT_MS    3000

/* 阻塞式对时：DNS解析(≤2s) + UDP请求/应答(≤3s)，全程带超时。
 * 仅供网络任务在 MQTT 在线时调用（与 DNS 查询互斥，不会并发）。
 * 返回：0=成功且 *outUnix 已写入 UTC 秒；非0=失败（调用方按周期重试） */
uint8_t SNTP_GetUnix(uint32_t *outUnix);

#endif /* __SNTP_H */
