# STM32F407 多功能工业物联网网关（裸机全功能版）

基于 **STM32F407VGT6 + W5500 + Modbus RTU** 的工业物联网网关：RS485 采集温湿度 → W5500 硬件协议栈经 MQTT 上云 → LCD 仪表盘本地可视化。附带全套健壮性机制（模块在位检测、配置自愈、链路防抖、全链路超时保护）。

> 本仓库为**里程碑 A（裸机全功能版）**存档。FreeRTOS 多任务、断网缓存、OTA 升级为里程碑 B 内容，开发中。

## 系统架构

```
温湿度变送器 ──RS485(Modbus RTU)──┐
                                  │ USART2
                    ┌─────────────▼─────────┐
LCD 仪表盘 ◄──SPI3──┤   STM32F407VGT6       │
(SPI3/1.54寸)       │   (SPL 标准库)        ├──SPI2──W25Q64 Flash(阶段9/10用)
                    └─────────────┬─────────┘
                                  │ SPI1
                          ┌───────▼───────┐
                          │  W5500 模块   │──网线──> 路由器 ──> MQTT Broker
                          └───────────────┘         (broker.emqx.io:1883)
```

## 硬件

| 部件 | 型号/说明 |
|---|---|
| 主控 | 鹿小班 LXB407VG-P1 核心板（STM32F407VGT6，HSE 8MHz，板载 RTC 晶振+电池座） |
| 以太网 | USR-ES1（W5500，硬件 TCP/IP 协议栈） |
| 温湿度 | RS485 导轨式变送器（Modbus RTU 从站，DC 5~28V 供电） |
| 显示 | 1.54 寸 TFT 240×240（ST7789，SPI） |
| 存储 | W25Q64 模块（SPI Flash，8MB，断网缓存/OTA 用） |
| RTC | 板载 32.768kHz 晶振 + CR1220 电池座（F407 内部 RTC；外部 DS1307 模块经排查确认故障已弃用） |

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

**调试串口（CH340，USART1）**：CH340 TXD→PA10、RXD→PA9、GND 共地。**CH340 的 3.3V/5V 不要接**（只留三根线）。

### 供电与注意事项（重要）

1. **单电源同源**：所有模块的 3.3V/GND 必须从核心板引脚取（DCDC 同源+短地线）。外部面包板电源与核心板混供时地电位弹跳，会导致 SPI 误码、以太网链路反复断开——本项目实测踩坑
2. **器件电平**：W5500/W25Q64/SP3485/TFT 均 3.3V；**DS1307 已弃用**（若接必须 5V）；变送器 5V 宽压
3. **3.3V 轨容量**：W5500 峰值 >200mA，建议模块 VIN3.3/GND 就近并联 470µF 电解电容
4. **网线接路由器 LAN 口**（MQTT 出公网需要）；静态 IP `192.168.0.250` 需避开路由器 DHCP 池——**换 IP 后先断电 ping 验证地址空闲再上电**（防 IP 冲突）
5. Modbus 通信帧内**温度为 16 位补码**（0xFF9B=-101→-10.1℃），寄存器 0x0000=湿度、0x0001=温度（×10）

## 编译与烧录

### 环境

- Keil MDK 5.x + **ARM Compiler 5**（V5.06）
- 器件包 Keil.STM32F4xx_DFP
- 固件库：STM32F4xx_StdPeriph_Driver **V1.8.0**（已随仓库包含，勿用 V1.9——官方确认其 stm32f4xx.h 有宏定义 bug）

### 编译

打开 `标准库基础工程/基础项目.uvprojx` → F7。

- 已配置：AC5、MicroLIB、HEX 输出、`_WIZCHIP_=5500`、静态 IP `192.168.0.250`
- `stm32f4xx_fmc.c` 已从编译组排除（F427/429 专用，F407 编不过）

### 烧录

- **ST-Link**：SWD（SWDIO=PA13/SWCLK=PA14），Keil 直接 F8 下载，Flash Download 勾选 Reset and Run
- **串口 ISP**：`Objects/基础项目.hex` + FlyMcu（BOOT0 按住进 ISP → 复位 → 下载 → BOOT0 回低 → 复位）

## 上电验证步骤

1. LCD 出现仪表盘框架，`Link: UP`（绿）——网线链路建立
2. `MQTT: ONLINE`（绿）——MQTT 已连接 broker.emqx.io
3. `Temp/Hum` 出现数值并每 2 秒刷新——Modbus 采集正常
4. `PUB: n` 每 5 秒 +1——发布正常
5. 电脑（同一局域网）`ping 192.168.0.250` 应稳定 1~5ms
6. NetAssist/MQTTX 连 `broker.emqx.io:1883` 订阅 `gateway/gw001/data`，每 5 秒收到 JSON：
   `{"device":"gw001","temp":22.4,"hum":35.6}`
7. 串口 `115200-8-N-1` 输出运行日志；发送 `s` 可设置 RTC 基准时间

## 目录结构

```
├── 标准库基础工程/          Keil 工程（本仓库主体）
│   ├── Start/               启动文件 + CMSIS
│   ├── Library/             STM32F4 标准库 V1.8.0（fmc.c 已排除）
│   ├── Hardware/            板级驱动
│   │   ├── usart.c/h        调试串口 + printf 重定向
│   │   ├── delay.c/h        SysTick 毫秒延时 + GetTick()
│   │   ├── lcd_spi_154.c/h  1.54寸 ST7789 驱动（商家例程移植）
│   │   ├── modbus.c/h       Modbus RTU 主站（USART2+RS485）
│   │   ├── spi2.c/h + w25q64.c/h   SPI Flash（断网缓存/OTA用）
│   │   ├── i2c.c/h + ds1307.c/h    I2C/RTC（DS1307 故障已弃用，保留备用）
│   │   ├── rtc.c/h          F407 内部 RTC（当前时间源）
│   │   ├── w5500_bsp.c/h    W5500 BSP（SPI1+复位时序+回调注册+配置自愈）
│   │   └── mqtt.c/h         精简 MQTT 3.1.1 客户端（QoS0）
│   ├── User/main.c          应用层：仪表盘 + 状态机 + 健壮性逻辑
│   ├── Ethernet/            WIZnet ioLibrary（socket/wizchip_conf/W5500）
│   └── 测试程序存档/        DS1307+AT24C32 诊断程序存档
├── docs/
│   ├── 工业网关项目开发计划.md   完整开发计划（阶段0~13）
│   ├── LXB407VG-P1原理图.pdf     核心板原理图
│   └── 简报/                     阶段任务简报（Modbus/MQTT）
```

## 路线图

- [x] **里程碑 A（本仓库）**：Modbus 采集 → W5500 MQTT 上云 → LCD 仪表盘，全链路健壮性机制
- [ ] 阶段8：FreeRTOS 多任务（5任务+队列/互斥锁/看门狗+任务心跳）
- [ ] 阶段9：断网缓存（W25Q64 日志式缓存+CRC+掉电安全+按时间戳补传）
- [ ] 阶段10：OTA（Bootloader+双槽+失败回滚）
- [ ] 可选：SD 卡（FatFS）/ CAN / 4G 双链路

## 踩坑记录（真实排查，面试可讲）

1. **IP 地址冲突**：ping 通了但延迟 300~700ms 且时断时续——断电板子后仍能 ping 通 = 地址被局域网其他设备（DHCP 分配）占用。静态 IP 设备上电前必须先断电 ping 验证地址空闲
2. **诊断代码读错寄存器**：W5500 VERSIONR 在 0x0039，误读成 0x0000（MR，复位值 0x00）——"模块离线"结论全是误报，排查方向被自己的诊断代码带偏。教训：诊断代码的寄存器地址必须先对照数据手册
3. **F4 外设初始化漏配 GPIO_PinAFConfig**：光配 GPIO_Mode_AF 不够，必须写 AFR 路由到外设，否则引脚停在 AF0（SPI2 翻车实例）
4. **硬件等待循环必须有超时**：SPI 等待标志位死等 → 异常电平时整机冻结
5. **模块离线时禁用 ioLibrary socket API**：寄存器读回 0xFF 使 close() 死等 Sn_CR 清零 → 冻结
6. **单次采样不做重大决策**：单次状态读数毛刺就 close TCP 连接 → 加防抖（连续 N 次才动作）
7. **单电源同源**：模块 3.3V/GND 从不同电源取电 + 长地线 → 地弹跳毁掉 SPI 信号、USB 靠近电脑就断链
8. **改配置时显示文本与生效数值是两处**（改了 IP 显示字符串、漏了数字宏 → 屏幕谎报 IP）

## 相关文档

- 开发计划（阶段0~13 全流程）：[docs/工业网关项目开发计划.md](docs/工业网关项目开发计划.md)
- 核心板原理图：[docs/LXB407VG-P1原理图.pdf](docs/LXB407VG-P1原理图.pdf)
- 阶段任务简报：[docs/简报/](docs/简报/)
