#ifndef __W5500_BSP_H
#define __W5500_BSP_H

#include "main.h"

/* W5500(USR-ES1) 接线：SCLK=PA5 / MISO=PA6 / MOSI=PA7（SPI1，AF5）
 * nSS=PC4、nRST=PC5（本BSP软件控制），PWDN=接GND（固定正常工作态），nINT=不接（轮询方式） */

/* ===== 网络配置区：静态IP，按你的路由器网段调整 ===== */
#define W5500_MAC      {0x02, 0x00, 0x00, 0x00, 0x00, 0x01}   /* 本地管理地址，可自定义 */
#define W5500_IP       {192, 168, 0, 250}
#define W5500_SN       {255, 255, 255, 0}
#define W5500_GW       {192, 168, 0, 1}
#define W5500_IP_TEXT  "192.168.0.250"
/* ==================================================== */

/* 底层初始化：硬件复位时序+注册ioLibrary回调+分配收发缓冲
 * （SPI1外设与CS/RST引脚初始化由 MX_SPI1_Init / MX_GPIO_Init 完成） */
void W5500_BSP_Init(void);

/* 写入静态网络信息（MAC/IP/子网掩码/网关，取自上方配置区） */
void W5500_NetworkInit(void);

/* 网线连接状态：1=LinkUp，0=LinkDown */
uint8_t W5500_LinkStatus(void);

/* 读W5500版本寄存器（正常=0x04）：≠0x04说明模块未供电/未接上（此时链路读数不可信） */
uint8_t W5500_ReadVersion(void);

/* 配置自愈校验：校验芯片IP配置，丢失（芯片被意外复位）则自动重新下发
 * 返回1=配置完好；0=曾丢失已自动恢复 */
uint8_t W5500_CheckConfig(void);

/* SPI1收发等待超时计数：>0说明SPI1硬件层出现过异常（配合串口日志判断卡死原因） */
uint32_t W5500_BSP_SPI1Err(void);

#endif /* __W5500_BSP_H */
