#ifndef __DATALOG_H
#define __DATALOG_H

#include "stm32f4xx.h"
#include "w25q64.h"     /* 返回码 W25Q64_OK / W25Q64_ERR_* 复用该驱动的定义 */

/*
 * 阶段9：断网缓存 —— W25Q64 数据缓存区管理（记录式环形 + 掉电安全）
 *
 * 分区表（W25Q64 8MB，阶段9/10 合理规划）：
 *   0x000000-0x07FFFF  Slot A 固件区 512KB（阶段10 OTA）
 *   0x080000-0x0FFFFF  Slot B 固件区 512KB（阶段10 OTA）
 *   0x100000-0x10FFFF  参数/标志区  64KB（阶段10：升级标志等）
 *   0x110000-0x7FFFFF  数据缓存区 7.25MB（本模块）
 *
 * 缓存区结构：32字节定长记录 × 128条/4KB扇区，扇区环形轮转（磨损均衡）。
 * 记录带 CRC16，掉电写一半的记录 CRC 不合法 → 自动跳过（等效"flag最后写"）。
 * 读指针已消费位置存 AT24C64 双槽书签（at24c64_slot.c，tag=SLOT_TAG_BOOKMARK）：
 * 看门狗/断电重启都不丢，避免每条记录都擦参数区（4KB扇区擦写寿命只有10万次）。
 *
 * 掉电特性：AT24C64 掉电保持，补传进度断电也不丢；EEPROM 不在位时书签无效，
 * 退化为从头补传（记录本身带时间戳，服务器可按 ts 去重）。
 */

/* 记录 32 字节定长 */
#define DL_REC_SIZE       32UL                                // 单条离线缓存记录固定32字节
#define DL_REC_PER_SECTOR (W25Q64_SECTOR_SIZE / DL_REC_SIZE)   /* 128 */ // 每个4K扇区最多存放128条记录：4096/32=128
#define DL_SECTORS        1775UL   /* (0x800000-0x110000)/4096，缓存区扇区数 */ // W25Q64数据缓存区总扇区数量
#define DL_BASE_ADDR      0x110000UL                          // 离线数据缓存分区在W25Q64内的起始物理地址

/* 初始化：校验 W25Q64 在位，扫扇区恢复读/写指针（备份寄存器书签优先）。
 * 返回：W25Q64_OK=成功；W25Q64_ERR_ID=Flash 不在位（缓存禁用，可重试 Init） */
uint8_t DL_Init(void);

/* 追加一条记录（离线时由存储任务调用）。缓存满时丢最旧一整个扇区。
 * 返回：W25Q64_OK=成功；W25Q64_ERR_SPI=Flash 异常 */
uint8_t DL_Append(uint32_t ts, int16_t t10, uint16_t h10);

/* 当前缓存条数（补传任务用它决定是否需要补传） */
uint32_t DL_Count(void);

/* 读最旧一条（消费前 Peek，补传发布成功后才 Pop，保证至少一次）
 * 返回：W25Q64_OK=成功；W25Q64_ERR_PARAM=缓存空 */
uint8_t DL_Peek(uint32_t *ts, int16_t *t10, uint16_t *h10);

/* 消费最旧一条（补传成功后调用）。读空一整个扇区时读指针移入下一扇区。
 * 返回：W25Q64_OK=成功；W25Q64_ERR_PARAM=缓存空 */
uint8_t DL_Pop(void);

/* 补传结束强制提交书签：绕过写限流，无条件把当前 rp 落盘一次。
 * 由存储任务 drain 结束后调用，把限流窗口内未落盘的进度补齐。 */
void DL_BookmarkCommit(void);

#endif /* __DATALOG_H */
