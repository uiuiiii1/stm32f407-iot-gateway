#ifndef __INT_BOOTLOADER_H
#define __INT_BOOTLOADER_H

#include <stdint.h>

/* 跳转到 app_base_addr 处的应用（带栈顶/向量/分区校验）。返回 1=校验失败未跳 */
uint8_t bootloader_jump_to_app(uint32_t app_base_addr);

#endif /* __INT_BOOTLOADER_H */
