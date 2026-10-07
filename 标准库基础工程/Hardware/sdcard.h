#ifndef __SDCARD_H
#define __SDCARD_H

#include "stm32f4xx.h"

/*============================================================================
  MicroSD 卡 SPI 模式驱动（SDv2/SDHC 为主，向下兼容 SDv1/MMC）
  - 【软件 SPI 专用总线】CS=PC6 / SCK=PC7 / MISO=PC8 / MOSI=PC9（GPIO 位拍，模式0）
    ——F407 三条硬件 SPI 已被 W5500/W25Q64/LCD 占满，且 SD 模块的 MISO 缓冲器
    常使能（OE 接地）会抢线，故走独立 GPIO：物理上无人共享，争用问题不存在
  - 速率：Init 期 ~400kHz（SD 规范识别期上限），之后 ~2MHz（DWT 半位延时，
    2s 一条记录的吞吐需求远低于此）；依赖 AT24C64_Init 同款的 DWT 周期计数器，
    驱动内自使能
  - 模块供电：模块带 AMS1117 稳压，VCC 必须接 5V（3.3V 进 AMS1117 输出仅 ~2.1V，
    卡欠压起不来）
  - ⚠️ 所有超时依赖 GetTick()（调度器启动前恒为 0）——本驱动的函数
    只能在调度器启动后的任务上下文里调用（如 STG 任务），勿在 main 初始化段直接调
  - 错误处理原则：卡不在位时应答恒为 0xFF，按"无卡"返回错误码，绝不死等
  ============================================================================*/

/* 错误码 */
#define SD_OK              0x00
#define SD_ERR_PARAM       0x01    /* 参数非法 */
#define SD_ERR_NO_CARD     0x02    /* 无应答：卡未插/接线/供电问题 */
#define SD_ERR_TIMEOUT     0x03    /* 应答/忙等待超时 */
#define SD_ERR_RW          0x04    /* 读/写数据令牌或数据响应错误 */

/* 对外接口 */
uint8_t  SD_Init(void);                                  /* 上电初始化（可重复调用） */
uint8_t  SD_ReadSectors(uint32_t Lba, uint8_t *Buf, uint32_t Cnt);   /* 读 n 个 512B 扇区 */
uint8_t  SD_WriteSectors(uint32_t Lba, const uint8_t *Buf, uint32_t Cnt); /* 写 n 个 512B 扇区 */
uint32_t SD_GetSectorCount(void);                        /* 卡容量（扇区数），0=未知 */

#endif /* __SDCARD_H */
