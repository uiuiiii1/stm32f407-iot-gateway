#ifndef __W25Q64_H
#define __W25Q64_H

#include "stm32f4xx.h"

/* W25Q64 接线：CS=PB12（本驱动软件控制+初始化），SCK=PB13 / MISO=PB14 / MOSI=PB15（硬件SPI2） */

/* 器件参数 */
#define W25Q64_PAGE_SIZE      256
#define W25Q64_SECTOR_SIZE    4096
#define W25Q64_BLOCK_SIZE     65536
#define W25Q64_TOTAL_SIZE     0x800000UL  /* 8MB */
#define W25Q64_JEDEC_ID       0xEF4017UL

/* 返回值定义 */
#define W25Q64_OK             0    /* 执行成功，无错误 */
#define W25Q64_ERR_SPI        1    /* SPI通信错误，读写Flash通信失败 */
#define W25Q64_ERR_ID         2    /* 芯片ID识别错误，不是支持的W25Q型号 */
#define W25Q64_ERR_PARAM      3    /* 传入参数错误：地址越界、空指针、长度非法 */
#define W25Q64_ERR_TIMEOUT    4    /* 操作超时，Flash一直处于忙状态，等待超时 */


/* CS 片选引脚：PB12 */
#define W25Q64_CS_GPIO        GPIOB
#define W25Q64_CS_PIN         GPIO_PIN_12

/* 初始化：配置CS引脚与SPI2，读JEDEC ID并按W25Q家族自动识别容量（80/16/32/64/128均可）
 * 返回 : W25Q64_OK=成功；W25Q64_ERR_ID=ID不匹配（模块没接好/型号不对）；W25Q64_ERR_SPI=SPI异常 */
uint8_t W25Q64_Init(void);

/* 读JEDEC器件ID
 * Id : 出参，正常应为 0xEF4017
 * 返回 : W25Q64_OK=成功；W25Q64_ERR_PARAM=空指针 */
uint8_t W25Q64_ReadID(uint32_t *Id);

/* 任意长度读（连续读自动越页，无256字节限制）
 * Addr : 首地址；Buf/Len : 接收缓冲区及长度
 * 返回 : W25Q64_OK=成功；W25Q64_ERR_PARAM=地址越界/空指针 */
uint8_t W25Q64_Read(uint32_t Addr, uint8_t *Buf, uint32_t Len);

/* 页编程：单次最多256字节且不得跨页（硬件限制，越界返回参数错）
 * Addr : 目标地址（须事先擦除，NOR只能把1写成0）；Buf/Len : 数据及长度（≤256）
 * 返回 : W25Q64_OK=成功；W25Q64_ERR_PARAM=参数错；W25Q64_ERR_TIMEOUT=写忙超时 */
uint8_t W25Q64_PageProgram(uint32_t Addr, const uint8_t *Buf, uint16_t Len);

/* 任意长度写：内部自动按页拆分（注意：目标区域须事先擦除，否则写入无效）
 * Addr : 首地址；Buf/Len : 数据及长度（自动跨页）
 * 返回 : W25Q64_OK=成功；W25Q64_ERR_PARAM=地址越界；W25Q64_ERR_SPI=页编程失败 */
uint8_t W25Q64_Write(uint32_t Addr, const uint8_t *Buf, uint32_t Len);

/* 扇区擦除（4KB，Addr任意落在目标扇区内即可），典型耗时约60~150ms
 * 返回 : W25Q64_OK=成功；W25Q64_ERR_PARAM=地址越界；W25Q64_ERR_TIMEOUT=忙超时 */
uint8_t W25Q64_EraseSector(uint32_t Addr);

/* 全片擦除（约20~200秒，慎用）
 * 返回 : W25Q64_OK=成功；W25Q64_ERR_TIMEOUT=忙超时 */
uint8_t W25Q64_ChipErase(void);

#endif /* __W25Q64_H */
