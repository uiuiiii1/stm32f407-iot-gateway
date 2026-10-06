#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阶段10.4：把 gen_ota_msgs.py 生成的报文单次发布到 MQTT 触发 OTA
用法：
  pip install paho-mqtt          # 首次需要
  python ota_send.py                          # 默认公共 broker.emqx.io / 块间隔100ms / ota_msgs.txt
  python ota_send.py 100 127.0.0.1            # 指定块间隔ms、broker
  python ota_send.py 100 127.0.0.1 ota_msgs_spl.txt   # 指定报文文件（SPL/HAL 各自保留，互不覆盖）
说明：
  报文第1行 = ota_begin（发 cmd 主题），其余 = 分块（发 ota 主题）
  单次发送，发完即退出；板子随后自动复位升级。
"""
import sys, time, os, paho.mqtt.client as mqtt

BROKER = "broker.emqx.io"
PORT = 1883
CMD_TOPIC = "gateway/gw001/cmd"
FW_TOPIC = "gateway/gw001/ota"
INTERVAL_MS = 100
try:
    if len(sys.argv) >= 2:
        INTERVAL_MS = int(sys.argv[1])
    if len(sys.argv) >= 3:
        BROKER = sys.argv[2]
except ValueError:
    print("用法：python ota_send.py [块间隔ms] [broker] [报文文件]"); sys.exit(1)

CLIENT_ID = "gw001-ota-sender-%d" % (os.getpid() & 0xFFFF)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))   # 以脚本所在目录为准，从哪运行都能找到
MSG_FILE = os.path.join(BASE_DIR, "ota_msgs.txt")
if len(sys.argv) >= 4:
    MSG_FILE = os.path.join(BASE_DIR, sys.argv[3])

def main():
    if not os.path.exists(MSG_FILE):
        print("缺少 %s：先运行 gen_ota_msgs.py 生成（可用 --out 指定文件名）" % MSG_FILE)
        sys.exit(1)
    lines = [l.rstrip("\n") for l in open(MSG_FILE, encoding="utf-8") if l.strip()]
    if len(lines) < 2:
        print("%s 内容过少（应 >= 2 行）" % MSG_FILE)
        sys.exit(1)

    c = mqtt.Client(client_id=CLIENT_ID, protocol=mqtt.MQTTv311)
    print("连接 %s:%d ..." % (BROKER, PORT))
    c.connect(BROKER, PORT, 60)
    c.loop_start()
    time.sleep(0.5)

    print("发 ota_begin -> %s" % CMD_TOPIC)
    c.publish(CMD_TOPIC, lines[0], qos=0)
    time.sleep(0.3)

    n = 0
    for line in lines[1:]:
        c.publish(FW_TOPIC, line, qos=0)
        n += 1
        if n % 10 == 0:
            print("已发分块 %d/%d" % (n, len(lines) - 1))
        time.sleep(INTERVAL_MS / 1000.0)

    print("全部 %d 块已发布，等待设备消化 1s..." % n)
    time.sleep(1.0)
    c.loop_stop()
    c.disconnect()
    print("完成。观察设备串口 OTA 流程。")

if __name__ == "__main__":
    main()
