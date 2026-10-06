# STM32F407 多功能工业物联网网关（FreeRTOS + 断网缓存 + OTA）

基于 **STM32F407VGT6 + FreeRTOS + W5500 + Modbus RTU** 的工业物联网网关：RS485 采集温湿度 → W5500 硬件协议栈经 MQTT 上云 → LCD 仪表盘本地可视化；断网期间数据缓存进外部 Flash，联网后自动补传；支持 **MQTT 远程固件升级（OTA）**——分块下载 → 校验 → Bootloader 烧写 → 运行确认 → 失败自动回退旧版。

> **已完成并上板验证**：裸机全功能版（里程碑 A）→ FreeRTOS V11.3.0 多任务版 → 断网缓存/SNTP 对时/联网补传 → OTA 远程升级（v0.4，含 Slot B 旧版备份与自动回滚）。
>
> 包含两个功能完全一致的工程版本：**`标准库基础工程/`**（SPL V1.8.0）与 **`HAL库工程/`**（HAL + CubeMX），共用同一个独立 **`bootloader/`** 工程。两者 90% 驱动同源（Modbus/MQTT/W5500 BSP/LCD/W25Q64/AT24C64/OTA），可对照阅读。

## 系统架构

```
温湿度变送器 ──RS485(Modbus RTU)──┐
                                  │ USART2
                    ┌─────────────▼─────────┐
LCD 仪表盘 ◄──SPI3──┤   STM32F407VGT6       ├──SPI2──W25Q64(断网缓存+OTA固件槽)
(SPI3/1.54寸)       │   (FreeRTOS, 5任务)   │
                    └─────────────┬─────────┘
                                  │ SPI1
                          ┌───────▼───────┐
                          │  W5500 模块   │──网线──> 路由器 ──> MQTT Broker
                          └───────────────┘
AT24C64(软件I2C PB10/PB11)：补传书签 + OTA 升级标志/确认标记/回滚计数（掉电安全）
```

**FreeRTOS 任务**：看门狗巡检（IWDG+全任务心跳）/ Modbus 采集 / 网络+MQTT+OTA / 存储（断网缓存+补传泵）/ LCD 显示；共享状态互斥保护，消息流走队列。

## 存储分区布局

```
片内 Flash  0x08000000-0x08007FFF  Bootloader（32KB，独立工程，两固件通用）
            0x08008000-0x0807FFFF  应用（SPL 或 HAL）
W25Q64(8MB) 0x000000-0x07FFFF     Slot A：OTA 新固件（头8B大端元数据，正文自0x1000）
            0x080000-0x0FFFFF     Slot B：升级前旧版备份（自动回滚用）
            0x100000-0x10FFFF     参数区（备用）
            0x110000-0x7FFFFF     断网缓存（32B/条环形记录 + CRC16，掉电安全）
AT24C64     0x00/0x10             补传书签（双槽乒乓+序号+CRC，掉电撕裂兜底）
            0x20/0x23/0x24/0x30   OTA 升级标志/待确认锁存/启动计数/确认版本
```

## OTA 远程升级（阶段10）

流程：**应用收到 `ota_begin`（含总长+CRC）→ MQTT 分块接收（512B/块，乱序即中止）→ 写 Slot A → 整包 CRC 校验 → 双槽提交升级标志 → 复位 → Bootloader 复核并烧写 0x08008000 → 新固件 MQTT 上线后发 `{"ota":"ok","ver":"x.x"}` 确认**。Bootloader 侧设启动计数器：连续 N 次未确认告警；升级前旧版已备份至 Slot B，校验失败/烧写中断自动回退。

**PC 端测试工具**（两工程目录各一套）：

```powershell
python gen_ota_msgs.py <固件.bin> --ver 0.5      # 生成 ota_msgs.txt（begin+130分块）
start_broker.cmd                                  # 启动本地测试 broker（amqtt）
python ota_send.py 100 192.168.0.106              # 参数：块间隔ms → broker IP → 报文文件
```

> 测试建议用局域网 broker：公共 broker 的跨国链路吞吐受 W5500 接收窗口（2KB）限制，高峰期会丢块；设备端 broker 地址在 `Hardware/mqtt.h` 的 `MQTT_BROKER_IP`/`MQTT_LOCAL_BROKER` 切换。

## 硬件

| 部件 | 型号/说明 |
|---|---|
| 主控 | 鹿小班 LXB407VG-P1 核心板（STM32F407VGT6，HSE 8MHz，板载 RTC 晶振+电池座） |
| 以太网 | USR-ES1（W5500，硬件 TCP/IP 协议栈） |
| 温湿度 | RS485 导轨式变送器（Modbus RTU 从站，DC 5~28V 供电） |
| 显示 | 1.54 寸 TFT 240×240（ST7789，SPI） |
| 存储 | W25Q64 模块（SPI Flash，8MB，断网缓存/OTA 用） |
| EEPROM | AT24C64 模块（软件 I2C：SCL=PB10、SDA=PB11，地址 0xA0） |
| RTC | F407 内部 RTC（LSE 32.768kHz；SNTP 网络对时自动校准） |

### 接线表

**W5500（USR-ES1，SPI1）**

| 模块引脚 | 核心板 | 说明 |
|---|---|---|
| VIN3.3 / GND（×2 都要接） | 3.3V / GND | 3.3V 供电，峰值 >200mA |
| SCLK / MISO / MOSI | PA5 / PA6 / PA7 | SPI1（AF5） |
| nSS | PC4 | 软件片选，低有效 |
| nRST | PC5 | 复位，低有效（拉低>500µs→释放等≥50ms） |
| PWDN | 模块内部接地 | 无需外接 |
| nINT | 不接 | 轮询方式 |

**Modbus 温湿度（USART2 + SP3485）**

| SP3485 | 核心板/变送器 | 说明 |
|---|---|---|
| VCC | 3.3V | 3.3V 版收发器 |
| RX / TX | PA2(TX) / PA3(RX) | 交叉连接 |
| DE+RE（短接） | PA4 | 高=发送，低=接收 |
| A / B | 变送器 A / B | 不通就对调（各厂家命名不统一） |
| 变送器 VCC+ / GND | 5V / GND | 宽压 5~28V |

**1.54 寸 TFT（SPI3，FPC 8P 座）**

| 屏引脚 | 核心板 | | 屏引脚 | 核心板 |
|---|---|---|---|---|
| SCK | PB3 | | CS | PA15 |
| MOSI | PB5 | | DC | PD13 |
| RST | 板载 RC 复位 | | BL | PD12 |

**AT24C64（软件 I2C）**：SCL→PB10、SDA→PB11（开漏+上拉，模块无板上拉则各补 4.7k）、VCC→3.3V、WP→GND、A0/A1/A2→GND。

**调试串口（CH340，USART1）**：CH340 TXD→PA10、RXD→PA9、GND 共地。**CH340 的 3.3V/5V 不要接**（只留三根线）。

### 供电与注意事项（重要）

1. **单电源同源**：所有模块的 3.3V/GND 必须从核心板引脚取（DCDC 同源+短地线）。外部面包板电源与核心板混供时地电位弹跳，会导致 SPI 误码、以太网链路反复断开——本项目实测踩坑
2. **器件电平**：W5500/W25Q64/SP3485/TFT/AT24C64 均 3.3V；变送器 5V 宽压
3. **3.3V 轨容量**：W5500 峰值 >200mA，建议模块 VIN3.3/GND 就近并联 470µF 电解电容
4. **网线接路由器 LAN 口**（MQTT 出公网需要）；静态 IP `192.168.0.250` 需避开路由器 DHCP 池——**换 IP 后先断电 ping 验证地址空闲再上电**（防 IP 冲突）
5. Modbus 通信帧内**温度为 16 位补码**（0xFF9B=-101→-10.1℃），寄存器 0x0000=湿度、0x0001=温度（×10）

## 编译与烧录

### 环境

- Keil MDK 5.x + **ARM Compiler 5**（V5.06）
- 器件包 Keil.STM32F4xx_DFP
- 标准库工程固件库：STM32F4xx_StdPeriph_Driver **V1.8.0**（已随仓库包含，勿用 V1.9）

### 编译（三个工程）

| 工程 | 工程文件 | 说明 |
|---|---|---|
| Bootloader | `bootloader/bootloader.uvprojx` | 先烧，一次即可（占 0x08000000 前 32KB） |
| SPL 应用 | `标准库基础工程/基础项目.uvprojx` | 0x08008000 起 |
| HAL 应用 | `HAL库工程/MDK-ARM/RTOS的多功能工业网关.uvprojx` | 0x08008000 起 |

- 已配置：AC5、MicroLIB、HEX 输出、`_WIZCHIP_=5500`（SPL）、静态 IP `192.168.0.250`
- **烧录注意**：Bootloader 落地后烧应用必须选 **"Erase Sectors"（擦除所需扇区）**，不能全片擦除（会抹掉 Bootloader）

### 上电流程与验证

1. `=== gw001 bootloader v1.0 ===` → 读 EEPROM 升级状态 → `no update` → `jump app 0x08008000`
2. 应用启动（HAL 版有 `LCD: 0x29` 初始化打印可区分固件）→ LCD 仪表盘、`Link: UP`（绿）
3. `MQTT: ONLINE`（绿）→ 自动订阅 `gateway/gw001/cmd` 与 `gateway/gw001/ota`
4. SNTP 自动对时（ntp.aliyun.com）；`Temp/Hum` 每 2 秒刷新；`PUB: n` 每 5 秒 +1
5. MQTTX 订阅 `gateway/gw001/data` 每 5 秒收 JSON；**拔网线**：LCD 变红、数据进 W25Q64 缓存；**插回**：变绿、缓存自动补传到 `gateway/gw001/replay`（带原始时间戳）

## 目录结构

```
├── bootloader/              Bootloader 独立工程（32KB@0x08000000，OTA 烧写/回滚）
├── 标准库基础工程/           SPL V1.8.0 应用工程（0x08008000）
│   ├── Start/ + Library/    启动文件 + CMSIS + STM32F4 标准库 V1.8.0
│   ├── FreeRTOS/            内核 V11.3.0（CM4F port + heap_4）
│   ├── Hardware/            板级驱动
│   │   ├── usart/tick/spi2/w25q64/at24c64/at24c64_slot
│   │   ├── datalog.c/h      断网缓存（环形记录+书签）
│   │   ├── ota.c/h          OTA 下载器（分块/CRC/确认）
│   │   ├── mqtt.c/h         MQTT 3.1.1（CONNECT/PUBLISH/SUBSCRIBE+下行解析）
│   │   ├── sntp.c/h、modbus.c/h、rtc.c/h、lcd_spi_154.c/h、w5500_bsp.c/h
│   ├── User/app.c           FreeRTOS 任务/队列/OTA 接线
│   ├── Ethernet/            WIZnet ioLibrary（socket/W5500）
│   ├── gen_ota_msgs.py      OTA 报文生成（PC 端）
│   ├── ota_send.py          OTA 发送（PC 端，paho-mqtt）
│   └── start_broker.cmd     本地测试 broker 启动脚本
├── HAL库工程/                HAL 应用工程（与 SPL 功能对齐，含同套驱动与工具）
└── docs/                    项目计划等文档
```

## 路线图

- [x] 里程碑 A：裸机全功能网关（Modbus 采集 → MQTT 上云 → LCD 仪表盘 + 全链路健壮性）
- [x] FreeRTOS 多任务（5 任务 + 队列/互斥锁/IWDG+任务心跳）
- [x] 断网缓存（W25Q64 环形记录 + CRC + 掉电安全 + 按时间戳补传 + SNTP 对时）
- [x] OTA 远程升级（Bootloader + 双固件槽 + 运行确认 + Slot B 自动回退）
- [ ] 可选：SD 卡（FatFS）/ CAN / 4G 双链路

## 踩坑记录（节选，完整版见各工程内文档）

1. **IP 地址冲突**：ping 通了但延迟 300~700ms 且时断时续——断电板子后仍能 ping 通 = 地址被局域网其他设备占用。静态 IP 设备上电前必须先断电 ping 验证地址空闲
2. **诊断代码读错寄存器**：W5500 VERSIONR 在 0x0039，误读成 0x0000——"模块离线"结论全是误报。教训：诊断代码的寄存器地址必须先对照数据手册
3. **F4 外设初始化漏配 GPIO_PinAFConfig**：光配 GPIO_Mode_AF 不够，必须写 AFR 路由，否则引脚停在 AF0（SPI2 翻车实例）
4. **硬件等待循环必须有超时**：SPI 等待标志位死等 → 异常电平时整机冻结
5. **模块离线时禁用 socket API**：寄存器读回 0xFF 使 close() 死等 Sn_CR 清零 → 冻结
6. **单次采样不做重大决策**：单次状态毛刺就 close TCP 连接 → 加防抖（连续 N 次才动作）
7. **单电源同源**：模块从不同电源取电 + 长地线 → 地弹跳毁掉 SPI 信号
8. **看门狗巡检者必须给自己打卡**：巡检任务漏了自身心跳 → 每 33 秒（=IWDG 周期）准确复位；任务打卡必须放在循环第一行（continue 会跳过循环尾）
9. **跨介质存储用显式字节打包**：结构体对齐/padding 不可移植，且 CRC 输入不能包含自身字段——否则全部记录校验失败（41 条缓存静默丢弃实例）
10. **Bootloader 跳转后必须尽早恢复全局中断**：跳转前 `__disable_irq()`，依赖中断的延时（如 HAL_Delay）在调度器/中断恢复前会无声挂死
11. **HAL 应用重定位必须同步 VECT_TAB_OFFSET**（或 main 开头设 `SCB->VTOR`），且 CubeMX 重生成会覆盖需重打

## 相关文档

- 开发计划（阶段0~13 全流程）：[docs/工业网关项目开发计划.md](docs/工业网关项目开发计划.md)
- 核心板原理图：[docs/LXB407VG-P1原理图.pdf](docs/LXB407VG-P1原理图.pdf)
- 工程说明：[标准库基础工程/工程说明.md](标准库基础工程/工程说明.md)、[HAL库工程/工程说明.md](HAL库工程/工程说明.md)
