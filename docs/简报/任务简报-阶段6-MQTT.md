# 任务简报：阶段6 —— MQTT 上云（EMQX 公共服务器）

> 本简报自包含。工作目录：`F:\于RTOS的多功能工业网关\标准库基础工程`
> 动手前确认 **Keil μVision 已关闭**（要改 uvprojx）。

## 一、目标

在现有 W5500 socket API 之上实现精简 MQTT 客户端（MQTT 3.1.1，QoS0），连接 EMQX 公共服务器 `broker.emqx.io:1883`，每 5 秒发布一次阶段3 采到的温湿度数据（JSON），电脑用 MQTTX 订阅验证。

## 二、项目背景事实（务必遵守，不要重新发明）

- 工程为 SPL 标准库（非 HAL），**源码一律 UTF-8**；文件组 Start/User/Hardware/Ethernet/Library（.h 在前 .c 在后成对，.h 是 FileType 5）
- **socket API 已可用**：`Ethernet/socket.h`（socket/connect/listen/send/recv/close/getSn_SR，ioLibrary 已接通，echo 服务器验证过）。Socket 编号 0~7
- **main.c 里有一套来之不易的健壮性逻辑，一行都不能动**：
  - 模块在位检测（版本寄存器，连续5次坏读才判OFF）——OFF 时**严禁调用任何 socket API**（读回0xFF会使 close() 死等冻结）
  - 配置自愈（SIPR 校验，连续3次不一致才重下发网络配置）
  - 链路防抖（连续3次 LinkDown 才 close socket）
  - 你的 MQTT 状态机必须嵌在"模块在线 + 链路UP"的区域之内，与 echo 服务器同位置
- **MicroLIB 的 sprintf 不支持 %f**：温湿度是 ×10 有符号整数（temp10=-101 → -10.1℃），JSON 数值用"整数部分+小数部分"分别拼（sprintf %d.%d，负数先取绝对值加负号），或发布原始×10值+字段名标注 _raw
- 硬件：W5500=USR-ES1 已连通路由器（192.168.0.x 网段，静态IP 192.168.0.250）；Modbus 温湿度（modbus.c/h 已验证，0x0000=湿度、0x0001=温度，×10，温度补码）
- 无界面编译：`D:\Keil5\UV4\UV4.exe -b "F:\于RTOS的多功能工业网关\标准库基础工程\基础项目.uvprojx" -j0 -o build_log.txt`，退出码0=0错0警；改 uvprojx 用 Python（re.sub 用 lambda），**必须编译复核**

## 三、网络事实与 DNS

- MQTT broker：`broker.emqx.io:1883`（TCP，无TLS无认证，匿名）。这是**公共服务器**，全世界共用——clientid 和 topic 必须带唯一后缀防冲突（clientid 冲突会被服务器互踢）
- broker 域名解析的两种方案（简报要求按顺序做）：
  1. **快速验证**：电脑上 `nslookup broker.emqx.io` 查当前 IP（如 44.或 121.x 开头），硬编码进代码先跑通 CONNECT/PUBLISH
  2. **正式版**：接 DNS 模块——从 jsDelivr 拉 ioLibrary 的 DNS 文件：
     `https://cdn.jsdelivr.net/gh/Wiznet/ioLibrary_Driver@master/Internet/DNS/dns.c` 和 `dns.h`
     （放 `Ethernet/DNS/`，FilePath 同理，IncludePath 加 .\Ethernet\DNS；`DNS_init` 需要一块缓冲区、`DNS_run(dns_server_ip, domain, out_ip)` 用 UDP 查询——**先读 dns.c 确认它内部占用哪个 Socket 编号，与 MQTT 的 Socket 错开**；DNS 服务器 IP 用网关 192.168.0.1）
- 网络必须经路由器（MQTT 要出公网），电脑走同一路由器的 WiFi

## 四、MQTT 3.1.1 最小协议知识（自写实现够用，不需要移植 paho）

MQTT 报文 = 固定头(1字节) + 剩余长度(1~4字节变长编码) + 载荷。本工程只需要 5 种：

| 报文 | 固定头 | 方向 | 说明 |
|---|---|---|---|
| CONNECT | 0x10 | 客户端→服务器 | 载荷：协议名"MQTT"(4B)+级别0x04+连接标志+keepalive(2B大端，填60)+ClientID |
| CONNACK | 0x20 | 服务器→客户端 | 第2字节=0x00 表示接受；非0=拒绝（打印原因重试） |
| PUBLISH | 0x30 | 客户端→服务器 | QoS0：载荷=主题长度(2B大端)+主题+数据，无报文ID |
| PINGREQ | 0xC0 | 客户端→服务器 | 2字节（C0 00）；keepalive=60s 时**每30秒必须发一次**，否则服务器90s踢人 |
| DISCONNECT | 0xE0 | 客户端→服务器 | 优雅断开（E0 00） |

剩余长度编码（变长）：len<128 直接1字节；更大则每字节低7位+进位标志，先写低位。实现一个 `MQTT_EncodeLength(len, buf)` 返回编码字节数。

## 五、具体任务

### 1. 新建 `Hardware/mqtt.c` / `Hardware/mqtt.h`

建议 API（可微调，但状态机思想不能丢）：

```c
/* 配置区 */
#define MQTT_BROKER_IP   {…, …, …, …}    /* 方案1：nslookup查到的 broker.emqx.io IP */
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENTID    "gw001-lxb407"  /* 公共服务器必须唯一 */
#define MQTT_TOPIC       "gateway/gw001/data"
#define MQTT_KEEPALIVE   60              /* 秒；PINGREQ周期30s */

/* 状态 */
typedef enum { MQ_IDLE, MQ_TCP_OK, MQ_MQTT_OK } MQTT_State;

void     MQTT_Init(void);                                   /* 记录broker IP等，分配Socket编号(建议1) */
uint8_t  MQTT_Process(void);        /* 状态机：每轮调用。返回MQ_MQTT_OK=在线；
                                       内部负责：TCP连接(SOCK_INIT→connect→ESTABLISHED)、
                                       发CONNECT、等CONNACK(带超时)、在线期每30s发PINGREQ */
uint8_t  MQTT_Publish(const char *topic, const uint8_t *payload, uint16_t len);
                                     /* 在线时发PUBLISH(QoS0)；返回MB风格错误码 */
```

实现要点：
- TCP 层：`socket(sock, Sn_MR_TCP, 本地任意端口, 0x00)` → `connect(sock, broker_ip, 1883)` → 轮询 `getSn_SR` 到 `SOCK_ESTABLISHED`（connect 失败/超时要 close 后重试，**不得死等**）
- 所有 recv 带超时或非阻塞处理（模块离线已由 main 的在位检测兜底，但 CONNACK 等待仍要有超时上限）
- 重连规则：TCP 断开（getSn_SR 变回 CLOSED/CLOSE_WAIT）或 CONNACK 拒绝 → 关闭 → 状态机回 MQ_IDLE 自动重来；**重连间隔至少 2 秒**（公共服务器禁止高频重连）
- 主题/JSON 都用 sprintf 的 %d 拼（负温度取绝对值加负号），不用 %f

### 2. 更新 uvprojx

- Hardware 组追加 `mqtt.h`（Type 5）/`mqtt.c`（Type 1）成对条目
- 若接 DNS：Ethernet 组（或新建 DNS 组）追加 dns.h/dns.c，IncludePath 加 .\Ethernet\DNS，Define 无需新增

### 3. 更新 `User/main.c`

- **保留全部**：usart/LCD/delay/W5500 初始化、在位检测（含防抖）、配置自愈、链路防抖、loopback 触发——这些是前几天踩坑换来的，一行都不能少
- **移除**：echo 服务器状态机（case SOCK_CLOSED/ESTABLISHED/CLOSE_WAIT 那段）和 ECHO_SOCK/ECHO_PORT 宏（echo 验收已完成）
- **接回 Modbus**：`#include "modbus.h"`，MB_USART_Init()，每 2 秒 `MODBUS_ReadTempHum` 更新最近一次温湿度（失败保留上次值并计数）
- 主循环逻辑：模块在线+链路UP 时 → `MQTT_Process()`；每 5 秒（计数实现）：Modbus 读温湿度 → 成功则 `MQTT_Publish(MQTT_TOPIC, json, len)` → LCD 显示 `MQTT:OK PUB:n` 或失败原因；串口打印发布的 JSON 原文
- LCD 新增一行：`MQTT:ONLINE` / `MQTT:OFF`（固定坐标局部刷新）

### 4. JSON 载荷格式（发布内容）

```json
{"device":"gw001","temp":-10.1,"hum":60.2}
```

- 温湿度来自 modbus 的 ×10 值：符号+整数部分+`.`+小数部分手工拼（temp10 为负时 "-"，绝对值拆两半）
- 长度可能变化，注意 buf 大小和剩余长度编码正确性

## 六、验收标准（逐项自查+上板验证）

- [ ] 无界面编译 0 错 0 警；mqtt.c/h 为 UTF-8、已入 uvprojx Hardware 组
- [ ] 上电后串口顺序：System Start → version/config 自愈无异常 → `MQTT: ONLINE`
- [ ] 电脑 MQTTX（emqx 官网下载）连 `broker.emqx.io:1883`，订阅 `gateway/gw001/data`，**每 5 秒收到一条 JSON 且温湿度与变送器实际值一致（负温度正确）**
- [ ] 连接保持 5 分钟以上不掉线（keepalive PINGREQ 生效）
- [ ] 拔网线 → 串口打重连日志（LinkDown/config lost），插回 → 自动恢复发布；期间程序不死机不冻结
- [ ] 断开 NetAssist 前先停掉它——避免两个 TCP 客户端测试互相干扰

## 七、禁止事项

- 不要改动 main.c 的在位检测/配置自愈/链路防抖逻辑（含防抖阈值）
- 不要动 modbus.c/h、i2c/ds1307、w25q64/spi2、delay、lcd_* 文件
- 不要用 TLS/8883 端口、不要移植 paho 大库、不要用 %f
- MQTT Socket 编号与 DNS 内部占用的编号不能冲突（接 DNS 前先读 dns.c）
- 公共 broker 是共享资源：禁止高频重连（<2s）、禁止发布大流量数据
