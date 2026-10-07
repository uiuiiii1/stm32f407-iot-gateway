#ifndef __CAN_H
#define __CAN_H

#include "stm32f4xx.h"

/*============================================================================
  bxCAN1 驱动（阶段12：CAN 总线采集）
  - 引脚：PB8=CAN1_RX / PB9=CAN1_TX（AF9）——计划书原 PA11/12 与板上 USB 冲突，已修正
  - 速率 500kbps（APB1 42MHz：Prescaler=6, BS1=11tq, BS2=2tq, SJW=1，采样点 85.7%）
  - 两种模式：LoopBack=1 环回自测（Tx 帧不出芯片直接进自己的 Rx FIFO，无需收发器）；
    LoopBack=0 正常模式（配 TJA1050 收发器上真总线）
  - 接收采用轮询（CAN1_Poll），不占中断——CAN 数据率低（周期性读数），FIFO0 硬件
    可缓存 3 帧，2ms 级轮询窗口足够；若未来高频 CAN 需改 FIFO0 中断
  - 过滤器 0：掩码模式全接收（后续按需收紧）
  ============================================================================*/

/* 错误码 */
#define CAN_OK              0x00
#define CAN_ERR_PARAM       0x01
#define CAN_ERR_TIMEOUT     0x02
#define CAN_ERR_INIT        0x03

/* 默认传感器帧 ID（真总线联测时与对端约定） */
#define CAN_ID_SENSOR       0x100

uint8_t CAN1_Init(uint8_t LoopBack);         /* 1=环回自测模式 0=正常模式 */
uint8_t CAN1_Send(uint32_t StdId, const uint8_t *Data, uint8_t Len);   /* Len 1~8 */
uint8_t CAN1_Poll(uint32_t *StdId, uint8_t *Data, uint8_t *Len);       /* FIFO0 有帧则取出 */
uint8_t CAN1_SelfTest(void);                 /* 环回自测：发 2 帧收 2 帧比对（须先以环回模式 Init） */

#endif /* __CAN_H */
