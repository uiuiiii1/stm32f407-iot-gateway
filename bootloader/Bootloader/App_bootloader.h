#ifndef __APP_BOOTLOADER_H
#define __APP_BOOTLOADER_H

#include <stdint.h>

/* ===== 升级状态（存 AT24C64，布局与 F1 版保持一致，工具链通用） =====
 * EEPROM 0x20: [0]=状态  [1..2]=校验密钥(大端 0x5A6B)
 * 状态密钥不对/未知值 → 视作无需升级并写回修复
 * 待确认（启动计数器，A2）：
 *   EEPROM 0x23: 0x01=待确认（烧写新固件后置位，应用 MQTT 上线正常后清零）
 *   EEPROM 0x24: 待确认期间的启动计数；≥BL_CONFIRM_BOOTS 未确认 → 串口告警 */
#define APP_UPDATE_ADDR 0x20
#define CHECK_KEY       0x5A6B
#define BL_PENDING_ADDR 0x23
#define BL_COUNT_ADDR   0x24
#define BL_CONFIRM_BOOTS 3      /* 待确认启动多少次未确认 → 告警 */

#define BOOT_UPDATE     0x01   /* 需要执行固件升级 */
#define BOOT_NO_UPDATE  0x02   /* 无需升级，直接运行应用 */
#define BOOT_RESET      0x03   /* 保留（F1 状态布局兼容）；本工程视作 NO_UPDATE 处理 */

/* ===== W25Q64 元数据（Slot A 头 8 字节，大端，由下载端写入） =====
 * [0..3]=镜像在 W25 内的起始地址  [4..7]=镜像总字节数；正文 = .bin 裸镜像 */
#define META_APP_ADDR   0x00
#define META_APP_SIZE   8

/* ===== 片内 Flash 分区（F407VGT6 1MB） =====
 * 0x08000000-0x08007FFF  本 bootloader（32KB，扇区0/1）
 * 0x08008000-0x0807FFFF  应用区（扇区2..11） */
#define APP_FLASH_BASE_ADDR  0x08008000UL
#define APP_END_ADDR         0x08080000UL
#define APP_SIZE_MAX         0x78000UL   /* 应用区容量 480KB */
#define APP_SIZE_MIN         512UL

/* ===== 镜像在 W25 内的合法范围（Slot A: 0x000000-0x07FFFF） ===== */
#define APP_ADDR_MIN         0x00001000UL   /* 头 8KB 留给元数据/参数 */
#define W25_FIRMWARE_END     0x00080000UL   /* Slot A 末尾 */

/* ===== Slot B 备份区（A3 回滚：升级前把当前应用备份到这，新固件未确认则恢复） =====
 * 头 8 字节 = [片内地址 0x08008000(大端)][备份大小(大端)]；正文自 0x081000 起 */
#define SLOT_B_HEAD_ADDR     0x00080000UL
#define SLOT_B_IMG_ADDR      (SLOT_B_HEAD_ADDR + 0x1000UL)
#define SLOT_B_END           0x00100000UL   /* Slot B 末尾（参数区起点） */
#define SLOT_B_MAX_SIZE      (SLOT_B_END - SLOT_B_IMG_ADDR)

void App_bootloader_check_update(void);
void App_bootloader_process(void);
void App_bootloader_jump_app(void);

#endif /* __APP_BOOTLOADER_H */
