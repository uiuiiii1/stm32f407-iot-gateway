#include "modbus.h"

/*
 * Modbus RTU 主站（RS485 半双工：SP3485 + USART2）
 * DE/RE 短接在 PA4：发送前置高，等 TC 标志（最后一字节移位完成）后拉低转接收
 * 变送器寄存器：0x0000=湿度(×10 无符号)，0x0001=温度(×10 有符号补码)
 */

#define MB_DE_PORT   GPIOA
#define MB_DE_PIN    GPIO_Pin_4

#define MB_TIMEOUT   200000   /* 单字节等待上限，168MHz 下约几十毫秒 */

static void MB_DE_Send(void) { GPIO_SetBits(MB_DE_PORT, MB_DE_PIN); }
static void MB_DE_Recv(void) { GPIO_ResetBits(MB_DE_PORT, MB_DE_PIN); }

void MB_USART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = MB_DE_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(MB_DE_PORT, &GPIO_InitStructure);
    MB_DE_Recv();                        /* 默认接收态 */

    USART_InitStructure.USART_BaudRate            = MB_BAUDRATE;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

/* CRC16：多项式 0xA001，初值 0xFFFF，附加时低字节在前 */
static uint16_t MB_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else                crc >>= 1;
        }
    }
    return crc;
}

/* 丢弃接收缓冲残留字节（顺带清除 ORE 溢出标志） */
static void MB_FlushRx(void)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
        USART_ReceiveData(USART2);
}

/* DE 置高发送整帧，等 TC 后置低——TC=最后一字节移位完成，提前拉低会截断帧尾 */
static void MB_SendFrame(const uint8_t *d, uint16_t len)
{
    uint16_t i;

    MB_DE_Send();
    for (i = 0; i < len; i++)
    {
        USART_SendData(USART2, d[i]);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    }
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    MB_DE_Recv();
}

/* 带超时收一个字节，返回 0=成功 */
static uint8_t MB_RecvByte(uint8_t *b)
{
    uint32_t timeout = MB_TIMEOUT;

    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET)
    {
        if ((timeout--) == 0)
            return MB_ERR_TIMEOUT;
    }
    *b = (uint8_t)USART_ReceiveData(USART2);
    return MB_OK;
}

/*
 * 读保持寄存器（功能码 03），失败自动重试 2 次（共 3 次尝试）
 * 响应帧：addr func byteCount data[2*count] crcL crcH
 */
uint8_t MODBUS_ReadRegs(uint16_t regStart, uint16_t count, uint16_t *out)
{
    uint8_t  tx[8];
    uint8_t  rx[3 + 2 * 4];              /* 本工程最多读 4 个寄存器 */
    uint16_t crc;
    uint8_t  rxLen = (uint8_t)(5 + 2 * count);
    uint8_t  err = MB_OK;
    uint8_t  attempt, i;

    if (count == 0 || count > 4)
        return MB_ERR_FRAME;

    tx[0] = MB_SLAVE_ADDR;        // 字节0：从站地址
    tx[1] = 0x03;                 // 字节1：功能码 03 = 读保持寄存器
    tx[2] = (uint8_t)(regStart >> 8); // 字节2：起始寄存器地址【高字节】
    tx[3] = (uint8_t)(regStart & 0xFF);// 字节3：起始寄存器地址【低字节】
    tx[4] = 0x00;                 // 字节4：要读取寄存器数量【高字节】
    tx[5] = (uint8_t)count;       // 字节5：要读取寄存器数量【低字节】
    crc = MB_CRC16(tx, 6);        // 拿前面 tx[0]~tx[5] 这6个业务字节，计算CRC16
    tx[6] = (uint8_t)(crc & 0xFF);// 字节6：CRC【低字节】（Modbus RTU规定低字节放前面）
    tx[7] = (uint8_t)(crc >> 8);  // 字节7：CRC【高字节】

    for (attempt = 0; attempt < 3; attempt++)
    {
        MB_FlushRx();
        MB_SendFrame(tx, 8);

        for (i = 0; i < rxLen; i++)
        {
            err = MB_RecvByte(&rx[i]);
            if (err)
                break;                   /* 超时：重试 */
        }

        if (err == MB_OK)
        {
            if (rx[0] != MB_SLAVE_ADDR || rx[1] != 0x03 || rx[2] != 2 * count)
                err = MB_ERR_FRAME;
            else
            {
                crc = MB_CRC16(rx, rxLen - 2);
                if (crc != (uint16_t)(rx[rxLen - 2] | (rx[rxLen - 1] << 8)))
                    err = MB_ERR_CRC;
            }
        }

        if (err == MB_OK)
        {
            for (i = 0; i < count; i++)
                out[i] = (uint16_t)((rx[3 + 2 * i] << 8) | rx[4 + 2 * i]);
            return MB_OK;
        }
    }
    return err;
}

/* 读温湿度：寄存器 0x0000=湿度(×10)，0x0001=温度(×10 补码，按有符号解析) */
uint8_t MODBUS_ReadTempHum(int16_t *temp10, uint16_t *hum10)
{
    uint16_t regs[2];
    uint8_t  err;

    err = MODBUS_ReadRegs(MB_REG_START, MB_REG_COUNT, regs);
    if (err)
        return err;

    *hum10  = regs[0];
    *temp10 = (int16_t)regs[1];          /* 0xFF9B -> -101 -> -10.1℃ */
    return MB_OK;
}
