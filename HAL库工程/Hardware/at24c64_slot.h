#ifndef __AT24C64_SLOT_H
#define __AT24C64_SLOT_H

#include "main.h"
#include "AT24C64.h"

/* ===== AT24C64 双槽状态接口（掉电写安全） =====
 * 规划：AT24C64 0x0000 / 0x0010 各 16 字节两个状态槽，与 bootloader 用的
 *       0x20 升级状态区（[状态][0x5A][0x6B]）互不冲突。
 * 槽记录格式：
 *   [0]=tag  [1..2]='S''L'  [3..6]=递增序号(小端)  [7..14]=载荷(≤8B)  [15]=CRC8(前15字节)
 * 写入：轮替写"另一槽"，序号 = 当前最大有效序号 + 1；写后读回校验。
 * 读取：两槽都校验，取"序号大且校验通过"者；全部无效返回 SLOT_ERR_EMPTY。
 * 掉电写坏一槽 → 该槽 CRC 不过 → 自动用另一槽兜底。 */

#define SLOT_TAG_BOOKMARK   1       /* datalog 补传书签（rpSec+rpOff，8字节） */

#define SLOT_SIZE           16
#define SLOT_A_ADDR         0x0000
#define SLOT_B_ADDR         0x0010

#define SLOT_ERR_EMPTY      3       /* 无有效槽（首次使用）；区别于 AT24C64_OK/NAK/PARAM */

/* 写入一条状态记录（轮替到另一槽）。len 必须 ≤8。
 * 返回：AT24C64_OK / AT24C64_ERR_NAK / AT24C64_ERR_PARAM */
uint8_t AT24C64_SlotWrite(uint8_t tag, const uint8_t *payload, uint16_t len);

/* 读取当前有效状态记录。len 必须 ≤8，buf 至少 len 字节。
 * 返回：AT24C64_OK=取到 / SLOT_ERR_EMPTY=无有效槽 / AT24C64_ERR_NAK / AT24C64_ERR_PARAM */
uint8_t AT24C64_SlotRead(uint8_t tag, uint8_t *payload, uint16_t len);

#endif /* __AT24C64_SLOT_H */
