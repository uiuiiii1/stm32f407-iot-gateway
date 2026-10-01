# 任务简报：阶段3 —— RS485 Modbus RTU 温湿度采集

> 本简报自包含。工作目录：`F:\于RTOS的多功能工业网关\标准库基础工程`
> 动手前确认 **Keil μVision 已关闭**（要改 uvprojx）。

## 一、目标

实现 Modbus RTU 主站，经 RS485 轮询温湿度变送器，LCD 屏和串口同时显示温湿度值。

## 二、项目背景事实（务必遵守）

- 工程 `基础项目.uvprojx`：SPL 标准库（V1.8.0），不是 HAL；文件组 Start / User / Hardware / Library
- **源码一律 UTF-8 编码**（工程已配 `--locale=english`，UTF-8 中文注释不产生警告）
- Hardware 组现有 5 对文件（lcd_spi_154、lcd_fonts、delay、i2c、ds1307，.h 在前 .c 在后成对）。**i2c.c/ds1307.c 保留在工程里不要删**（RTC 模块退换货后还要用）
- uvprojx 修改规则：Python 字符串替换，re.sub 替换串用 `lambda m: ...`；File 条目仿照 Hardware 组现有 ds1307.h/.c 的格式（.h FileType 5 在前，.c FileType 1 在后）
- 无界面编译验证：`D:\Keil5\UV4\UV4.exe -b "F:\于RTOS的多功能工业网关\标准库基础工程\基础项目.uvprojx" -j0 -o build_log.txt`，退出码 0 = 0错0警
- **MicroLIB 的 sprintf 不支持 %f**：变送器数据是 ×10 存储的整数（如 255 = 25.5℃），LCD 显示用"整数部分 + 手动插小数点"方式，禁止 sprintf 拼浮点

## 三、硬件接线（用户已完成，供核对）

| SP3485 模块 | STM32 | 说明 |
|---|---|---|
| VCC | 3.3V | **必须 3.3V**（3.3V 版收发器） |
| GND | GND | 共地 |
| RX | PA2（USART2_TX） | 交叉 |
| TX | PA3（USART2_RX） | 交叉 |
| DE+RE（短接） | PA4 | 高=发送，低=接收 |

变送器：VCC+ 接 5V、GND 共地、A/B 接 SP3485 的 A/B（A-A/B-B，不通就对调——RS485 的 A/B 命名各厂家不统一）。

## 四、前置验证（PC 端，不写代码就能做）

1. USB转RS485 插电脑（CH340 驱动），A/B 接变送器，变送器接 5V
2. 串口助手 9600-8-N-1，HEX 模式发送：`01 03 00 00 00 02 C4 0B`（读 0x0000 起 2 个寄存器）
3. 正常回复 8 字节：`01 03 04 温度×10 湿度×10 CRC`。无回复就试：起始地址 0x0001、读 4 个寄存器、波特率 4800、从站地址非 01；CRC 用"Modbus CRC16 在线计算器"重算
4. **把实际波特率/从站地址/寄存器地址/换算公式记下来**，填进第五节的配置区。拿不到商家寄存器表就按典型值（0x0000 温、0x0001 湿、×10、9600-8N1、从站 0x01）先跑

## 五、具体任务

### 1. 新建 `Hardware/modbus.c` / `Hardware/modbus.h`

modbus.h 顶部配置区（用户拿到寄存器表后只改这里）：

```c
#define MB_SLAVE_ADDR   0x01      /* 变送器从站地址 */
#define MB_BAUDRATE     9600      /* 串口波特率 */
#define MB_PARITY       0         /* 0=无校验（后续可扩展） */
#define MB_REG_START    0x0000    /* 温度寄存器起始地址 */
#define MB_REG_COUNT    2         /* 温度+湿度共2个寄存器 */
```

modbus.c 实现：

- `MB_USART_Init()`：USART2（PA2/PA3 复用 AF7，MB_BAUDRATE，8数据位，1停止位，按配置区校验）；PA4 推挽输出作 DE
- `MB_CRC16(const uint8_t *data, uint16_t len)`：多项式 0xA001，初值 0xFFFF，**结果低字节在前**附加到帧尾
- `MODBUS_ReadRegs(uint8_t slave, uint16_t regStart, uint16_t count, uint16_t *out)`：
  组帧 `slave 03 regH regL cntH cntL CRC_L CRC_H` → DE 拉高 → 逐字节发送 → **等 USART2 的 TC（Transmission Complete）标志置位后再把 DE 拉低**（防最后1~2字节被截断，这是 RS485 最经典的坑）→ 关闭 DE 接收 → 带超时收 `5+2*count` 字节 → 校验地址/功能码/CRC → 取出寄存器值（高字节在前）
- 超时策略：整体响应超时 200ms（字符间超时可简化不做），失败自动重试 2 次
- `MODBUS_ReadTempHum(int16_t *temp10, uint16_t *hum10)`：调 ReadRegs，按配置区拆分温度/湿度（×10 原始值；温度若为有符号按 int16 处理）
- 所有返回值：0=成功，非0=错误码（超时/CRC错/响应长度不对分开设）

### 2. 更新 uvprojx

Hardware 组按现有成对格式追加 `modbus.h`（FileType 5）+ `modbus.c`（FileType 1）。

### 3. 更新 `User/main.c`

- **移除**：`#include "i2c.h"`、`#include "ds1307.h"`、I2C1_Init、I2C probe、总线扫描、DS1307_Init/GetTime/时间显示等全部 RTC 相关代码（Hardware 里的 i2c/ds1307 文件保留，不删）
- 保留：Delay_Init、USART1 printf 重定向、SPI_LCD_Init 及标题显示（标题文字改成 "Stage3: Modbus"）
- 主循环每 2 秒（Delay_ms）：
  - `MODBUS_ReadTempHum` 成功 → LCD 固定坐标显示两行：`Temp: 25.5 C`、`Hum: 60.2 %`（×10 值拆成整数部分 + 小数部分分别用 LCD_DisplayNumber 显示，中间用 LCD_DisplayString 画小数点；负温度先显示 "-"）+ 串口 printf 同样内容；连续成功时清掉错误提示
  - 失败 → LCD 红字显示 `MODBUS ERR` + 串口打错误码（区分超时/CRC错），**不卡死、下一轮继续**
  - 显示用固定坐标局部覆盖，不要全屏 Clear

## 六、验收标准（逐项自查）

- [ ] modbus.c/h 为 UTF-8 编码，已加入 uvprojx Hardware 组（.h/.c 成对）
- [ ] 无界面编译退出码 0：**0 Error, 0 Warning**
- [ ] 所有等待循环（TC 标志、响应接收）都带超时，拔掉 A/B 线程序不死机、显示 ERR 后继续轮询
- [ ] DE 在 TC 置位后才拉低（注释里能看出为什么）
- [ ] 连续 10 次读取成功（串口日志可见），数值与 PC 端 USB转RS485 读到的一致

## 七、禁止事项

- 不要动 i2c.c/ds1307.c/delay.c/lcd_* 任何文件；不要动时钟配置、MicroLIB、编译器设置
- 不要用 sprintf 拼浮点；不要用 HAL/CubeMX
- 寄存器表与假设不符时：把商家寄存器表内容原样贴出来询问，不要猜着改协议逻辑
