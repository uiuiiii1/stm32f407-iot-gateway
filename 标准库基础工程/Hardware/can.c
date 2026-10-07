#include "can.h"
#include "delay.h"
#include <stdio.h>

/*============================================================================
  bxCAN1 驱动（见 can.h 顶部说明）
  - 环回模式说明：TX 与 RX 在芯片内部互连，帧不进收发器——GPIO/时序/过滤器/
    收发路径全部真实，唯独差分物理层例外（那层等 TJA1050 到位联测）
  - 超时全部基于 GetTick()（调度器启动后有效；自测在 COL 任务初始化段调用）
  ============================================================*/

#define CAN_TX_TIMEOUT_MS   20UL

/*---------------- 初始化 ----------------*/

uint8_t CAN1_Init(uint8_t LoopBack)
{
    GPIO_InitTypeDef        gpio = {0};
    CAN_InitTypeDef         can  = {0};
    CAN_FilterInitTypeDef   flt  = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* PB8=CAN1_RX：复用输入上拉（总线空闲显性电平由收发器保证；环回模式不依赖） */
    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    /* PB9=CAN1_TX：复用推挽 */
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);

    /* F4 必须显式路由 AF9（CAN1 在 PB8/9 的复用编号）——AF 漏配是本项目老坑 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);

    CAN_DeInit(CAN1);
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;              /* bus-off 后自动恢复（真总线容错） */
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = ENABLE;              /* 禁止自动重传：失败立即返回，别堵任务 */
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = LoopBack ? CAN_Mode_LoopBack : CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1  = CAN_BS1_11tq;        /* 42MHz/6=7MHz tq；1+11+2=14tq/bit */
    can.CAN_BS2  = CAN_BS2_2tq;         /* = 500kbps，采样点 (1+11)/14=85.7% */
    can.CAN_Prescaler = 6;
    if (CAN_Init(CAN1, &can) == CAN_InitStatus_Failed)
    {
        return CAN_ERR_INIT;
    }

    /* 过滤器 0：32 位掩码模式，ID/掩码全 0 = 全接收，帧入 FIFO0 */
    flt.CAN_FilterNumber        = 0;
    flt.CAN_FilterMode          = CAN_FilterMode_IdMask;
    flt.CAN_FilterScale         = CAN_FilterScale_32bit;
    flt.CAN_FilterIdHigh        = 0x0000;
    flt.CAN_FilterIdLow         = 0x0000;
    flt.CAN_FilterMaskIdHigh    = 0x0000;
    flt.CAN_FilterMaskIdLow     = 0x0000;
    flt.CAN_FilterFIFOAssignment= CAN_FIFO0;
    flt.CAN_FilterActivation    = ENABLE;
    CAN_FilterInit(&flt);

    return CAN_OK;
}

/*---------------- 发送 ----------------*/

uint8_t CAN1_Send(uint32_t StdId, const uint8_t *Data, uint8_t Len)
{
    CanTxMsg        tx;
    uint8_t         mb;
    uint32_t        t0 = GetTick();

    if (Data == 0 || Len == 0 || Len > 8)
    {
        return CAN_ERR_PARAM;
    }

    tx.StdId = StdId;
    tx.ExtId = 0;
    tx.IDE   = CAN_Id_Standard;
    tx.RTR   = CAN_RTR_Data;
    tx.DLC   = Len;
    for (mb = 0; mb < Len; mb++)
    {
        tx.Data[mb] = Data[mb];
    }

    mb = CAN_Transmit(CAN1, &tx);
    if (mb == CAN_TxStatus_NoMailBox)
    {
        return CAN_ERR_TIMEOUT;
    }

    /* 等发送完成（RQCP/TXOK 任一置位即邮箱释放）；NART 已开，失败不重试不堵塞 */
    while (CAN_TransmitStatus(CAN1, mb) == CAN_TxStatus_Failed)
    {
        if ((GetTick() - t0) >= CAN_TX_TIMEOUT_MS)
        {
            CAN_CancelTransmit(CAN1, mb);
            return CAN_ERR_TIMEOUT;
        }
    }
    return CAN_OK;
}

/*---------------- 接收（轮询） ----------------*/

uint8_t CAN1_Poll(uint32_t *StdId, uint8_t *Data, uint8_t *Len)
{
    CanRxMsg rx;

    if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0)
    {
        return CAN_ERR_TIMEOUT;         /* 无帧：调用方当"本次没数据"处理 */
    }
    CAN_Receive(CAN1, CAN_FIFO0, &rx);

    if (rx.IDE != CAN_Id_Standard || rx.RTR != CAN_RTR_Data)
    {
        return CAN_ERR_PARAM;           /* 只收标准数据帧，其余丢弃 */
    }
    *StdId = rx.StdId;
    *Len   = rx.DLC;
    for (rx.DLC = 0; rx.DLC < *Len; rx.DLC++)
    {
        Data[rx.DLC] = rx.Data[rx.DLC];
    }
    return CAN_OK;
}

/*---------------- 环回自测（须先 CAN1_Init(1)） ----------------*/

uint8_t CAN1_SelfTest(void)
{
    uint8_t  tx[2][8] = {
        {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
        {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80}
    };
    uint32_t ids[2] = {0x123, 0x321};
    uint32_t rid;
    uint8_t  rdata[8];
    uint8_t  rlen;
    uint8_t  i, j, pass = 0;

    for (i = 0; i < 2; i++)
    {
        if (CAN1_Send(ids[i], tx[i], 8) != CAN_OK)
        {
            printf("[CAN1] selftest: send FAIL (frame %u)\r\n", (unsigned)i);
            return CAN_ERR_TIMEOUT;
        }
    }

    /* 发送完成后两帧应先后出现在 FIFO0（环回模式下 TX 即 RX，到达顺序=发送顺序） */
    for (i = 0; i < 2; i++)
    {
        if (CAN1_Poll(&rid, rdata, &rlen) != CAN_OK)
        {
            printf("[CAN1] selftest: recv FAIL (got %u of 2)\r\n", (unsigned)i);
            return CAN_ERR_TIMEOUT;
        }
        if (rid != ids[i] || rlen != 8)
        {
            printf("[CAN1] selftest: id/dlc mismatch (id=%03X dlc=%u)\r\n",
                   (unsigned)rid, (unsigned)rlen);
            return CAN_ERR_PARAM;
        }
        for (j = 0; j < 8; j++)
        {
            if (rdata[j] != tx[i][j])
            {
                printf("[CAN1] selftest: data mismatch\r\n");
                return CAN_ERR_PARAM;
            }
        }
        pass++;
    }

    if (pass != 2 || CAN_MessagePending(CAN1, CAN_FIFO0) != 0)
    {
        printf("[CAN1] selftest: FAIL (pass=%u, pending=%u)\r\n",
               (unsigned)pass, (unsigned)CAN_MessagePending(CAN1, CAN_FIFO0));
        return CAN_ERR_PARAM;
    }
    printf("[CAN1] selftest PASS (2 frames, loopback)\r\n");
    return CAN_OK;
}
