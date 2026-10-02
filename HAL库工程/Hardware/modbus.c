#include "modbus.h"
#include "usart.h"      /* CubeMX 生成的 huart2 */

/*
 * Modbus RTU 主站（RS485 半双工：SP3485 + USART2）—— HAL 版
 * DE/RE 短接在 PA4：发送前置高，HAL_UART_Transmit 内部等 TC 后返回，再拉低转接收
 * 变送器寄存器：0x0000=湿度(×10 无符号)，0x0001=温度(×10 有符号补码)
 */

#define MB_BYTE_TIMEOUT_MS  50   /* 单字节等待上限：9600bps 一字节约1ms，留足余量 */
#define MB_SEND_TIMEOUT_MS  100  /* 整帧发送上限（8字节约8.3ms） */

static void MB_DE_Send(void) { HAL_GPIO_WritePin(MB_DE_GPIO_Port, MB_DE_Pin, GPIO_PIN_SET); }
static void MB_DE_Recv(void) { HAL_GPIO_WritePin(MB_DE_GPIO_Port, MB_DE_Pin, GPIO_PIN_RESET); }

/* USART2 的时钟/引脚/波特率、PA4 输出方向都由 MX_GPIO_Init / MX_USART2_UART_Init 完成，
 * 这里只保证上电后处于接收态 */
void MB_USART_Init(void)
{
    MB_DE_Recv();
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
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
        (void)(huart2.Instance->DR);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
}

/* DE 置高发送整帧。HAL_UART_Transmit 阻塞到 TC 置位才返回，
 * 所以返回后立刻拉低 DE 不会截断帧尾 */
static void MB_SendFrame(const uint8_t *d, uint16_t len)
{
    MB_DE_Send();
    HAL_UART_Transmit(&huart2, (uint8_t *)d, len, MB_SEND_TIMEOUT_MS);
    MB_DE_Recv();
}

/* 带超时收一个字节，返回 0=成功 */
static uint8_t MB_RecvByte(uint8_t *b)
{
    uint32_t start = HAL_GetTick();

    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == RESET)
    {
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
            __HAL_UART_CLEAR_OREFLAG(&huart2);
        if ((HAL_GetTick() - start) > MB_BYTE_TIMEOUT_MS)
            return MB_ERR_TIMEOUT;
    }
    *b = (uint8_t)(huart2.Instance->DR & 0xFF);
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

    tx[0] = MB_SLAVE_ADDR;
    tx[1] = 0x03;
    tx[2] = (uint8_t)(regStart >> 8);
    tx[3] = (uint8_t)(regStart & 0xFF);
    tx[4] = 0x00;
    tx[5] = (uint8_t)count;
    crc = MB_CRC16(tx, 6);
    tx[6] = (uint8_t)(crc & 0xFF);       /* CRC 低字节在前 */
    tx[7] = (uint8_t)(crc >> 8);

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
