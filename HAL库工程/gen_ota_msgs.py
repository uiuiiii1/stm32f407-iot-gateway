#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成 OTA 分块报文，供 ota_send.py 自动发布 / MQTTX 手动发布
用法：
  python gen_ota_msgs.py                          # 默认 2048B 假固件 → ota_msgs.txt
  python gen_ota_msgs.py app.bin --ver 0.5        # 用 .bin 当固件 → ota_msgs.txt
  python gen_ota_msgs.py app.bin --ver 0.5 --out ota_msgs_hal.txt   # 指定输出文件
产物：第1行 cmd 主题：ota_begin；其余 ota 主题：分块 {seq,data}
CRC32 = zlib.crc32（与固件侧 ota.c 的 IEEE 反射实现一致）。
"""
import sys, os, zlib

CHUNK = 512                 # 与 ota.h OTA_CHUNK_BYTES 一致
SIZE_MIN, SIZE_MAX = 512, 0x78000
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(BASE_DIR, "ota_msgs.txt")
VER = "0.6"                 # 当前固件版本（改版本号时同步改这里 + ota.h；OTA begin 的 ver 字段仅展示用）

def make_fw(n):
    b = bytearray(n)
    for i in range(n):
        b[i] = (0xCD + i) & 0xFF
    return bytes(b)

def main():
    size = 4096
    src = None
    ver = VER
    out = OUT
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--ver" and i + 1 < len(args):
            ver = args[i + 1]; i += 2
        elif a == "--out" and i + 1 < len(args):
            out = os.path.join(BASE_DIR, args[i + 1]); i += 2
        elif a.isdigit():
            size = int(a); i += 1
        else:
            src = a; i += 1
    if size < SIZE_MIN or size > SIZE_MAX:
        print("size %d 越界 [%d,%d]" % (size, SIZE_MIN, SIZE_MAX)); sys.exit(1)

    fw = make_fw(size) if src is None else open(src, "rb").read()
    if len(fw) != size:
        print("提示: %s 实际 %d 字节 != size %d，以实际为准" % (src, len(fw), size))
        size = len(fw)
    crc = zlib.crc32(fw) & 0xFFFFFFFF

    lines = []
    lines.append('{"cmd":"ota_begin","size":%d,"crc":"0x%08X","ver":"%s"}' % (size, crc, ver))
    n = size // CHUNK + (1 if size % CHUNK else 0)
    for s in range(n):
        piece = fw[s * CHUNK:(s + 1) * CHUNK]
        lines.append('{"seq":%d,"data":"%s"}' % (s, piece.hex()))

    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print("# 固件 %d 字节, %d 块(每块 %d), CRC32=0x%08X, ver=%s" % (size, n, CHUNK, crc, ver))
    print("# 已写入 %s（cmd:1行 + ota:%d行）" % (out, n))
    print(lines[0])

if __name__ == "__main__":
    main()
