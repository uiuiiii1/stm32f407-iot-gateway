#include "at24c64_slot.h"

/* ===== 双槽实现 =====
 * 槽记录：[0]=tag [1..2]='S''L' [3..6]=seq(LE) [7..14]=载荷 [15]=CRC8(前15字节)
 * 写入轮替，读取取"序号大且校验通过"的槽。 */

/* CRC8（多项式 0x07） */
static uint8_t slot_crc8(const uint8_t *p, uint32_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        uint8_t i;
        crc ^= *p++;
        for (i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* 校验一槽并解出 seq/载荷。返回 1=有效 */
static uint8_t slot_valid(const uint8_t *buf, uint8_t tag, uint32_t *seq, uint8_t *payload)
{
    uint8_t i;
    if (buf[0] != tag) return 0;
    if (buf[1] != 0x53 || buf[2] != 0x4C) return 0;   /* "SL" */
    if (slot_crc8(buf, 15) != buf[15]) return 0;
    *seq = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) |
           ((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 24);
    for (i = 0; i < 8; i++) payload[i] = buf[7 + i];
    return 1;
}

uint8_t AT24C64_SlotRead(uint8_t tag, uint8_t *payload, uint16_t len)
{
    uint8_t buf[SLOT_SIZE];
    uint8_t a[8], b[8];
    uint32_t seqA = 0, seqB = 0;
    uint8_t okA, okB;
    uint8_t i;

    if (payload == 0 || len > 8) return AT24C64_ERR_PARAM;

    if (AT24C64_Read(SLOT_A_ADDR, buf, SLOT_SIZE) != AT24C64_OK) return AT24C64_ERR_NAK;
    okA = slot_valid(buf, tag, &seqA, a);

    if (AT24C64_Read(SLOT_B_ADDR, buf, SLOT_SIZE) != AT24C64_OK) return AT24C64_ERR_NAK;
    okB = slot_valid(buf, tag, &seqB, b);

    if (!okA && !okB) return SLOT_ERR_EMPTY;

    /* 取序号大的有效槽（带符号差比较抗回绕） */
    if (okA && okB)
    {
        if ((int32_t)(seqA - seqB) >= 0) { for (i = 0; i < len; i++) payload[i] = a[i]; }
        else                             { for (i = 0; i < len; i++) payload[i] = b[i]; }
    }
    else if (okA) { for (i = 0; i < len; i++) payload[i] = a[i]; }
    else          { for (i = 0; i < len; i++) payload[i] = b[i]; }

    return AT24C64_OK;
}

uint8_t AT24C64_SlotWrite(uint8_t tag, const uint8_t *payload, uint16_t len)
{
    uint8_t buf[SLOT_SIZE];
    uint8_t a[8], b[8];
    uint32_t seqA = 0, seqB = 0;
    uint8_t okA, okB;
    uint32_t newSeq;
    uint8_t  target;                 /* 0=写A槽，1=写B槽 */
    uint16_t i;

    if (payload == 0 || len > 8) return AT24C64_ERR_PARAM;

    /* 读两槽，找最新有效序号 */
    if (AT24C64_Read(SLOT_A_ADDR, buf, SLOT_SIZE) != AT24C64_OK) return AT24C64_ERR_NAK;
    okA = slot_valid(buf, tag, &seqA, a);

    if (AT24C64_Read(SLOT_B_ADDR, buf, SLOT_SIZE) != AT24C64_OK) return AT24C64_ERR_NAK;
    okB = slot_valid(buf, tag, &seqB, b);

    /* 序号 +1；无有效槽则从 1 开始 */
    newSeq = 0;
    if (okA && seqA > newSeq) newSeq = seqA;
    if (okB && seqB > newSeq) newSeq = seqB;
    newSeq++;

    /* 目标 = 轮替到"另一槽"：两槽都有效写序号小者，只有一槽有效写空槽，全无效写A */
    if (okA && okB) target = ((int32_t)(seqA - seqB) > 0) ? 1 : 0;
    else if (okA)   target = 1;
    else if (okB)   target = 0;
    else            target = 0;

    /* 组装记录 */
    buf[0] = tag;
    buf[1] = 0x53; buf[2] = 0x4C;
    buf[3] = (uint8_t)newSeq;
    buf[4] = (uint8_t)(newSeq >> 8);
    buf[5] = (uint8_t)(newSeq >> 16);
    buf[6] = (uint8_t)(newSeq >> 24);
    for (i = 0; i < 8; i++)
        buf[7 + i] = (i < len) ? payload[i] : 0x00;
    buf[15] = slot_crc8(buf, 15);

    if (AT24C64_Write(target ? SLOT_B_ADDR : SLOT_A_ADDR, buf, SLOT_SIZE) != AT24C64_OK)
        return AT24C64_ERR_NAK;

    /* 读回校验：目标槽重新读，CRC 通过才算成功 */
    if (AT24C64_Read(target ? SLOT_B_ADDR : SLOT_A_ADDR, buf, SLOT_SIZE) != AT24C64_OK)
        return AT24C64_ERR_NAK;
    if (slot_crc8(buf, 15) != buf[15])
        return AT24C64_ERR_NAK;

    return AT24C64_OK;
}
