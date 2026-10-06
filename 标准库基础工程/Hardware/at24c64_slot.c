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
    uint8_t buf[SLOT_SIZE];     // 临时缓冲区，存放读出的整条16字节槽数据
    uint8_t a[8], b[8];         // 保存A槽、B槽解析后的载荷数据(最多8字节)
    uint32_t seqA = 0, seqB = 0;// A槽序号、B槽序号
    uint8_t okA, okB;           // A/B槽是否校验有效 1有效 0无效
    uint8_t i;

    // 参数检查：空指针或者读取长度超过载荷最大8字节，返回参数错误
    if (payload == 0 || len > 8) return AT24C64_ERR_PARAM;

    // 读取A槽全部16字节
    if (AT24C64_Read(SLOT_A_ADDR, buf, SLOT_SIZE) != AT24C64_OK) return AT24C64_ERR_NAK;
    okA = slot_valid(buf, tag, &seqA, a); // 校验A槽，解析序号和载荷

    // 读取B槽全部16字节
    if (AT24C64_Read(SLOT_B_ADDR, buf, SLOT_SIZE) != AT24C64_OK) return AT24C64_ERR_NAK;
    okB = slot_valid(buf, tag, &seqB, b); // 校验B槽，解析序号和载荷

    // A、B两个槽全都无效，没有可用备份
    if (!okA && !okB) return SLOT_ERR_EMPTY;

    /* 取序号大的有效槽（序号越大代表写入时间越新）
     * 带符号差比较抗回绕：两槽序号恒差 1，0xFFFFFFFF→0 回绕时
     * (int32)(0 - 0xFFFFFFFF)=+1 仍判 0 槽更新，不会卡死在旧槽 */
    if (okA && okB) // A、B都合法，比较版本序号
    {
        if ((int32_t)(seqA - seqB) >= 0) 
        { 
            for (i = 0; i < len; i++) payload[i] = a[i]; // A更新，拷贝A槽载荷
        }
        else              
        { 
            for (i = 0; i < len; i++) payload[i] = b[i]; // B更新，拷贝B槽载荷
        }
    }
    else if (okA) // 只有A槽有效，使用A槽数据
    { 
        for (i = 0; i < len; i++) payload[i] = a[i]; 
    }
    else          // 只有B槽有效，使用B槽数据
    { 
        for (i = 0; i < len; i++) payload[i] = b[i]; 
    }

    return AT24C64_OK; // 读取成功
}


uint8_t AT24C64_SlotWrite(uint8_t tag, const uint8_t *payload, uint16_t len)
{
    uint8_t buf[SLOT_SIZE];       // 组装整条16字节槽数据缓冲区
    uint8_t a[8], b[8];           // 存放A/B槽解析后的载荷
    uint32_t seqA = 0, seqB = 0;  // A槽序号、B槽序号
    uint8_t okA, okB;             // A/B槽是否校验有效
    uint32_t newSeq;              // 本次写入的新版本序号
    uint8_t  target;              // 目标槽：0=A槽，1=B槽
    uint16_t i;

    // 参数校验：空指针或者载荷长度超过8字节，返回参数错误
    if (payload == 0 || len > 8) return AT24C64_ERR_PARAM;

    /* 读两槽，找最新有效序号 */
    if (AT24C64_Read(SLOT_A_ADDR, buf, SLOT_SIZE) != AT24C64_OK) return AT24C64_ERR_NAK;
    okA = slot_valid(buf, tag, &seqA, a); // 校验解析A槽
    if (AT24C64_Read(SLOT_B_ADDR, buf, SLOT_SIZE) != AT24C64_OK) return AT24C64_ERR_NAK;
    okB = slot_valid(buf, tag, &seqB, b); // 校验解析B槽

    /* 序号 +1；无有效槽则从 1 开始 */
    newSeq = 0;
    if (okA && seqA > newSeq) newSeq = seqA;
    if (okB && seqB > newSeq) newSeq = seqB;
    newSeq++;

    /* 目标 = 轮替到另一槽，不覆盖最新有效记录（带符号差抗回绕，写"较旧"槽） */
    if (okA && okB) target = ((int32_t)(seqA - seqB) > 0) ? 1 : 0; // 两槽都有效，写序号小的槽
    else if (okA)   target = 1;  // 仅A有效，写B槽
    else if (okB)   target = 0;  // 仅B有效，写A槽
    else            target = 0;  // 首次写入，写A槽

    /* 组装16字节槽记录 */
    buf[0] = tag;
    buf[1] = 0x53; buf[2] = 0x4C;        // 魔数"SL"
    buf[3] = (uint8_t)newSeq;             // seq小端4字节
    buf[4] = (uint8_t)(newSeq >> 8);
    buf[5] = (uint8_t)(newSeq >> 16);
    buf[6] = (uint8_t)(newSeq >> 24);
    // 拷贝载荷，不足8字节补0
    for (i = 0; i < 8; i++)
        buf[7 + i] = (i < len) ? payload[i] : 0x00;
    buf[15] = slot_crc8(buf, 15); // 计算前15字节CRC8

    // 将组装好的数据写入选中的目标槽
    if (AT24C64_Write(target ? SLOT_B_ADDR : SLOT_A_ADDR, buf, SLOT_SIZE) != AT24C64_OK)
        return AT24C64_ERR_NAK;

    /* 读回校验：写完重读，CRC通过才算写入成功 */
    if (AT24C64_Read(target ? SLOT_B_ADDR : SLOT_A_ADDR, buf, SLOT_SIZE) != AT24C64_OK)
        return AT24C64_ERR_NAK;
    if (slot_crc8(buf, 15) != buf[15])
        return AT24C64_ERR_NAK;

    return AT24C64_OK;
}

