#ifndef __OTA_H
#define __OTA_H

#include "stm32f4xx.h"

/* ===== 阶段10.3：应用侧 OTA 下载器（MQTT 分块 → W25Q64 Slot A → 校验 → 标志 → 复位）=====
 * 与 bootloader 的约定（App_bootloader.h 一致）：
 *   W25Q64 0x00    : 8 字节元数据 [镜像W25地址(大端)][镜像大小(大端)]
 *   W25Q64 0x1000起: 固件正文（.bin 裸镜像），Slot A 上限 0x080000
 *   AT24C64 0x20   : 升级标志 [0x01=UPDATE][0x5A][0x6B]（bootloader 读取）
 * 下载通道 / 指令：
 *   gateway/gw001/cmd : {"cmd":"ota_begin","size":N,"crc":"0xXXXXXXXX","ver":".."}
 *   gateway/gw001/ota : {"seq":n,"data":"<hex>"}  每块 OTA_CHUNK_BYTES 字节
 */

#define OTA_TOPIC_CMD       "gateway/gw001/cmd"
#define OTA_TOPIC_FW        "gateway/gw001/ota"

/* 本固件版本（OTA 确认用）：改版本号时同步改这两个 */
#define OTA_VER_STR         "0.5"
#define OTA_VER_CODE        0x00000005UL

#define OTA_META_ADDR       0x00UL
#define OTA_META_SIZE       8UL
#define OTA_IMG_BASE        0x1000UL
#define OTA_SLOT_END        0x080000UL
#define OTA_CHUNK_BYTES     512UL        /* 每块明文字节数（hex=1024字符；报文~1060B < W5500 RX 2KB） */

#define OTA_SIZE_MIN        512UL        /* 与 bootloader APP_SIZE_MIN 一致 */
#define OTA_SIZE_MAX        0x78000UL    /* 应用区容量（bootloader APP_SIZE_MAX） */

#define OTA_TIMEOUT_MS      90000UL      /* 分块超时（配合 250ms/块慢速发送，留足余量） */

/* 返回码 */
#define OTA_OK              0
#define OTA_ERR_BUSY        1   /* 正在下载，忽略重复 begin */
#define OTA_ERR_BAD         2   /* size 越界 */
#define OTA_ERR_FLASH       3   /* 擦除/写失败 */
#define OTA_ERR_SEQ         4   /* 分块序号不连续 */
#define OTA_ERR_LEN         5   /* 分块长度越界 */
#define OTA_ERR_CRC         6   /* 整包 CRC 校验失败 */
#define OTA_ERR_TIMEOUT     7   /* 下载超时 */
#define OTA_ERR_IDLE        8   /* 未在下载态收到分块 */

/* 开始下载：擦除 Slot 区 + 写元数据 + 初始化 CRC。size/crc 来自 ota_begin */
uint8_t OTA_Begin(uint32_t size, uint32_t crc);

/* 收一块明文数据（seq 序号，0 起连续）。校验/写入 Slot A，最后一块触发 CRC 校验+写标志+复位 */
uint8_t OTA_Chunk(uint8_t seq, const uint8_t *raw, uint16_t len);

/* 网络任务每轮调用：处理下载超时 */
void    OTA_Poll(void);

/* 是否忙（网络任务用它挂起补传） */
uint8_t OTA_IsBusy(void);

void    OTA_Abort(void);

/* MQTT 回调：解析 cmd/ota 主题的 JSON 并驱动状态机 */
void    OTA_OnMqtt(const uint8_t *topic, uint16_t topicLen,
                   const uint8_t *payload, uint16_t len);

/* 固件运行确认（10.4）：返回 1=本版本尚未确认（需要发确认），0=已确认过。
 * 判定：AT24C64 0x30 存的确认标记 magic+版本码 == 当前 OTA_VER_CODE */
uint8_t OTA_NeedConfirm(void);

/* 写确认标记到 AT24C64（配合 OTA_NeedConfirm 用） */
void    OTA_ConfirmMark(void);

/* 调试/维护用：清除 AT24C64 0x30 确认标记 → 下次上线会重新发一次 ota:ok。
 * 用途：不涨版本号也想强制重确认/重测 OTA 时，在 app 里临时取消注释调用（见 app.c 注释） */
void    OTA_ConfirmClear(void);

/* 调试/维护用：直接把任意版本号写进确认标记（0x30）。ver 用 OTA_VER_CODE 或手写 */
void    OTA_ConfirmSet(uint32_t ver);

/* 固件运行正常通知（A2）：清 bootloader 的"待确认"锁存（AT24 0x23）→ 停止启动计数。
 * 在 MQTT 上线成功时调用（每个版本每次上线都调，幂等）。 */
void    OTA_NotifyAlive(void);

#endif /* __OTA_H */
