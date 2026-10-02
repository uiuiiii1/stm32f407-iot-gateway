//*****************************************************************************
//
//! \file w5500.c
//! \brief W5500 硬件抽象层（HAL）：SPI 总线读写 + Socket 收发缓冲搬运
//!
//! 本文件来自 WIZnet 官方 ioLibrary_Driver（github.com/Wiznet/ioLibrary_Driver），
//! 注释已由项目组翻译为中文，代码逻辑未改动。
//!
//! 核心概念（读懂本文件的关键）：
//! 1. W5500 的所有寄存器/缓冲区通过 SPI 访问，每笔 SPI 事务 = 3字节控制段 + 数据：
//!    控制段 = [地址高字节][地址低字节][块选择+读写方向+操作模式]
//! 2. 地址中隐含了"要访问哪个Socket、哪个缓冲区"（BSB位），由宏拼好
//! 3. Common寄存器块是全局配置，Socket n 的寄存器/收发缓冲各有独立地址块
//!
//! 原始版权与许可（BSD风格，保留原文以免违反许可证）：
//! Copyright (c) 2013, WIZnet Co., LTD. All rights reserved.
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the above copyright notice,
//! this conditions and the following disclaimer are retained.
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS", WITHOUT ANY
//! EXPRESS OR IMPLIED WARRANTIES.
//
//*****************************************************************************

//#include <stdio.h>
#include "w5500.h"

/* SPI 操作模式位（拼进控制段的低2位）：VDM=可变数据长度模式（本工程用），FDM=固定长度模式 */
#define _W5500_SPI_VDM_OP_          0x00
#define _W5500_SPI_FDM_OP_LEN1_     0x01
#define _W5500_SPI_FDM_OP_LEN2_     0x02
#define _W5500_SPI_FDM_OP_LEN4_     0x03

#if   (_WIZCHIP_ == 5500)
////////////////////////////////////////////////////

/*
 * 读一个字节的寄存器
 * AddrSel : 寄存器地址（含Socket块选择位，由 w5500.h 的宏拼出）
 * 返回    : 读到的1字节数据
 */
uint8_t  WIZCHIP_READ(uint32_t AddrSel) {
    uint8_t ret;
    uint8_t spi_data[3];

    WIZCHIP_CRITICAL_ENTER();            /* 进入临界区（保护SPI总线不被其他任务打断） */
    WIZCHIP.CS._select();                /* 片选拉低，选中W5500 */

    AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);   /* 地址或上"读+VDM"标志位 */

    if (!WIZCHIP.IF.SPI._read_burst || !WIZCHIP.IF.SPI._write_burst) {	/* 未注册burst回调：逐字节收发 */
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x00FF0000) >> 16);
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x0000FF00) >>  8);
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x000000FF) >>  0);
    } else {															/* 已注册burst回调：3字节控制段一次性发出 */
        spi_data[0] = (AddrSel & 0x00FF0000) >> 16;
        spi_data[1] = (AddrSel & 0x0000FF00) >> 8;
        spi_data[2] = (AddrSel & 0x000000FF) >> 0;
        WIZCHIP.IF.SPI._write_burst(spi_data, 3);
    }
    ret = WIZCHIP.IF.SPI._read_byte();   /* 读回1字节 */

    WIZCHIP.CS._deselect();              /* 释放片选 */
    WIZCHIP_CRITICAL_EXIT();
    return ret;
}

/*
 * 写一个字节到寄存器
 * AddrSel : 寄存器地址；wb : 待写的1字节
 */
void     WIZCHIP_WRITE(uint32_t AddrSel, uint8_t wb) {
    uint8_t spi_data[4];

    WIZCHIP_CRITICAL_ENTER();
    WIZCHIP.CS._select();

    AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);  /* 地址或上"写+VDM"标志位 */

    //if(!WIZCHIP.IF.SPI._read_burst || !WIZCHIP.IF.SPI._write_burst) 	// byte operation
    if (!WIZCHIP.IF.SPI._write_burst) {	/* 逐字节：3字节控制段 + 1字节数据 */
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x00FF0000) >> 16);
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x0000FF00) >>  8);
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x000000FF) >>  0);
        WIZCHIP.IF.SPI._write_byte(wb);
    } else {								/* burst：4字节一次发出 */
        spi_data[0] = (AddrSel & 0x00FF0000) >> 16;
        spi_data[1] = (AddrSel & 0x0000FF00) >> 8;
        spi_data[2] = (AddrSel & 0x000000FF) >> 0;
        spi_data[3] = wb;
        WIZCHIP.IF.SPI._write_burst(spi_data, 4);
    }

    WIZCHIP.CS._deselect();
    WIZCHIP_CRITICAL_EXIT();
}

/*
 * 连续读一段寄存器/缓冲区（burst读，用于从Socket收发缓冲取数据）
 * AddrSel : 起始地址；pBuf/len : 接收缓冲及长度
 */
void     WIZCHIP_READ_BUF(uint32_t AddrSel, uint8_t* pBuf, uint16_t len) {
    uint8_t spi_data[3];
    uint16_t i;

    WIZCHIP_CRITICAL_ENTER();
    WIZCHIP.CS._select();

    AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);

    if (!WIZCHIP.IF.SPI._read_burst || !WIZCHIP.IF.SPI._write_burst) {	/* 逐字节 */
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x00FF0000) >> 16);
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x0000FF00) >>  8);
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x000000FF) >>  0);
        for (i = 0; i < len; i++) {
            pBuf[i] = WIZCHIP.IF.SPI._read_byte();
        }
    } else {															/* burst：控制段+连续读 */
        spi_data[0] = (AddrSel & 0x00FF0000) >> 16;
        spi_data[1] = (AddrSel & 0x0000FF00) >> 8;
        spi_data[2] = (AddrSel & 0x000000FF) >> 0;
        WIZCHIP.IF.SPI._write_burst(spi_data, 3);
        WIZCHIP.IF.SPI._read_burst(pBuf, len);
    }

    WIZCHIP.CS._deselect();
    WIZCHIP_CRITICAL_EXIT();
}

/*
 * 连续写一段寄存器/缓冲区（burst写，用于往Socket收发缓冲放数据）
 * AddrSel : 起始地址；pBuf/len : 数据及长度
 */
void     WIZCHIP_WRITE_BUF(uint32_t AddrSel, uint8_t* pBuf, uint16_t len) {
    uint8_t spi_data[3];
    uint16_t i;

    WIZCHIP_CRITICAL_ENTER();
    WIZCHIP.CS._select();

    AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);

    if (!WIZCHIP.IF.SPI._write_burst) {	/* 逐字节 */
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x00FF0000) >> 16);
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x0000FF00) >>  8);
        WIZCHIP.IF.SPI._write_byte((AddrSel & 0x000000FF) >>  0);
        for (i = 0; i < len; i++) {
            WIZCHIP.IF.SPI._write_byte(pBuf[i]);
        }
    } else {								/* burst */
        spi_data[0] = (AddrSel & 0x00FF0000) >> 16;
        spi_data[1] = (AddrSel & 0x0000FF00) >> 8;
        spi_data[2] = (AddrSel & 0x000000FF) >> 0;
        WIZCHIP.IF.SPI._write_burst(spi_data, 3);
        WIZCHIP.IF.SPI._write_burst(pBuf, len);
    }

    WIZCHIP.CS._deselect();
    WIZCHIP_CRITICAL_EXIT();
}


/*
 * 读Socket发送缓冲剩余空间（Sn_TX_FSR，2字节）
 * 连读两次比较：两次一致才可信——因为该值会被硬件随时更新，
 * 读两次相同说明恰好没赶上硬件刷新，避免拿到撕裂的中间值
 */
uint16_t getSn_TX_FSR(uint8_t sn) {
    uint16_t val = 0, val1 = 0;

    do {
        val1 = WIZCHIP_READ(Sn_TX_FSR(sn));
        val1 = (val1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn), 1));
        if (val1 != 0) {
            val = WIZCHIP_READ(Sn_TX_FSR(sn));
            val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn), 1));
        }
    } while (val != val1);
    return val;
}


/*
 * 读Socket接收缓冲已收数据长度（Sn_RX_RSR，2字节），防撕裂的两次读比较同上
 */
uint16_t getSn_RX_RSR(uint8_t sn) {
    uint16_t val = 0, val1 = 0;

    do {
        val1 = WIZCHIP_READ(Sn_RX_RSR(sn));
        val1 = (val1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn), 1));
        if (val1 != 0) {
            val = WIZCHIP_READ(Sn_RX_RSR(sn));
            val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn), 1));
        }
    } while (val != val1);
    return val;
}

/*
 * 把要发送的数据写入Socket n 的TX缓冲区
 * 原理：TX写指针(TX_WR)决定写到缓冲区哪里，写完指针前移len
 * sn : Socket编号；wizdata/len : 数据及长度
 */
void wiz_send_data(uint8_t sn, uint8_t *wizdata, uint16_t len) {
    uint16_t ptr = 0;
    uint32_t addrsel = 0;

    if (len == 0) {
        return;
    }
    ptr = getSn_TX_WR(sn);
    //M20140501 : implict type casting -> explict type casting
    //addrsel = (ptr << 8) + (WIZCHIP_TXBUF_BLOCK(sn) << 3);
    addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_TXBUF_BLOCK(sn) << 3);   /* 定位到TX缓冲区+写偏移 */
    //
    WIZCHIP_WRITE_BUF(addrsel, wizdata, len);

    ptr += len;
    setSn_TX_WR(sn, ptr);                /* 写指针前移（之后由setSn_CR触发SEND命令才真正发出） */
}

/*
 * 从Socket n 的RX缓冲区读出收到的数据
 * 原理：RX读指针(RX_RD)决定从哪里读，读出后指针前移
 * 调用方读完并前移指针后，还需 setSn_CR(RECV) 通知硬件"已取走"，才会更新RSR释放空间
 */
void wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len) {
    uint16_t ptr = 0;
    uint32_t addrsel = 0;

    if (len == 0) {
        return;
    }
    ptr = getSn_RX_RD(sn);
    //M20140501 : implict type casting -> explict type casting
    //addrsel = ((ptr << 8) + (WIZCHIP_RXBUF_BLOCK(sn) << 3);
    addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(sn) << 3);   /* 定位到RX缓冲区+读偏移 */
    //
    WIZCHIP_READ_BUF(addrsel, wizdata, len);
    ptr += len;

    setSn_RX_RD(sn, ptr);                /* 读指针前移 */
}


/*
 * 丢弃Socket n 的len字节（只前移读指针、不搬运数据）
 * 用途：收到不想要的数据时快速腾空缓冲区
 */
void wiz_recv_ignore(uint8_t sn, uint16_t len) {
    uint16_t ptr = 0;

    ptr = getSn_RX_RD(sn);
    ptr += len;
    setSn_RX_RD(sn, ptr);
}

#endif
