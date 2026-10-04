#include "sntp.h"
#include <stdio.h>
#include <string.h>
#include "delay.h"
#include "socket.h"
#include "dns.h"
#include "mqtt.h"       /* 复用 MQTT_DNS_SERVER / DNS_run（同一套 DNS 配置） */

/*
 * SNTP 精简客户端：只取服务器 Transmit Timestamp（报文[40..43]）秒字段。
 * NTP 起始 1900-01-01，Unix 起始 1970-01-01，差 2208988800 秒。
 * 不做亚秒/游走修正——网关时间戳精度到秒足够。
 */

uint8_t SNTP_GetUnix(uint32_t *outUnix)
{
    uint8_t dnsSrv[4] = MQTT_DNS_SERVER;
    uint8_t srv[4] = SNTP_SERVER_IP;
    uint8_t pkt[48];
    uint8_t rip[4];
    uint16_t rport;
    uint32_t t0;
    int32_t r;

    /* 域名解析（阻塞但内部带 GetTick 超时；失败回退硬编码 IP） */
    if (DNS_run(dnsSrv, (uint8_t *)SNTP_DOMAIN, rip) != 1)
    {
        rip[0] = srv[0]; rip[1] = srv[1]; rip[2] = srv[2]; rip[3] = srv[3];
        printf("SNTP: DNS fail, fallback %d.%d.%d.%d\r\n", rip[0], rip[1], rip[2], rip[3]);
    }

    if (socket(SNTP_SOCK, Sn_MR_UDP, 30000 + SNTP_SOCK, 0x00) != SNTP_SOCK)
        return 1;

    /* 48字节请求：LI=0 VN=4 Mode=3(client)，其余填0 */
    memset(pkt, 0, sizeof(pkt));
    pkt[0] = 0x1B;
    if (sendto(SNTP_SOCK, pkt, 48, rip, SNTP_PORT) < 0)
    {
        close(SNTP_SOCK);
        return 2;
    }

    /* 等应答：带超时，绝不死等 */
    t0 = GetTick();
    for (;;)
    {
        r = recvfrom(SNTP_SOCK, pkt, sizeof(pkt), rip, &rport);
        if (r >= 48)
        {
            uint32_t ntpSec = ((uint32_t)pkt[40] << 24) | ((uint32_t)pkt[41] << 16)
                            | ((uint32_t)pkt[42] << 8) | (uint32_t)pkt[43];
            close(SNTP_SOCK);
            if (ntpSec < 2208988800UL)      /* 应答异常（早于1970） */
                return 3;
            *outUnix = ntpSec - 2208988800UL;
            return 0;
        }
        if ((GetTick() - t0) > SNTP_TIMEOUT_MS)
            break;
        Delay_ms(10);
    }
    close(SNTP_SOCK);
    return 4;
}
