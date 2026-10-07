#ifndef __AIR780E_H
#define __AIR780E_H

#include "main.h"     /* HAL 版：由 CubeMX 生成，含 stm32f4xx_hal.h 与引脚标签宏 */

/*============================================================================
  Air780E 4G 模块驱动（阶段13：以太网断开时的备份上云通道）——HAL 库版
  - 实物：银尔达 YED-M100M-C2（Y100E/Air780E 核心板），跑【合宙原厂 AT 固件】
    （2026-10-07 实机核实：MIPSTART/MCONNECT/MPUB 全通，CONNACK OK）
  - 接线：USART3 PC10(TX)/PC11(RX) AF7，115200-8N1；RDY=PC12 输入
    （模块连上服务器后输出 3.3V 高=在线判据）；RST=PE0 开漏，空闲高（暂未接）
  - 与标准库版差异：USART3 的时钟/GPIO/波特率由 CubeMX 生成（.ioc 已声明，
    main.c 调 MX_USART3_UART_Init），本驱动只做 RDY/RST 引脚、中断优先级、
    单字节中断接收（HAL_UART_Receive_IT + RxCplt/Error 回调，重挂起必须做）
  - MQTT 命令序列（合宙 AT 固件，实机验证）：
      AT+MIPSTART=broker.emqx.io,1883   → OK + "CONNECT OK"（TCP）
      AT+MCONNECT=1,300                 → OK + "CONNACK OK"（MQTT 握手；
        参数1=clean session，300=keepalive 秒。MCONFIG 可省略——空用户名密码
        会 +CME ERROR:765，而默认配置可直接连 emqx 匿名）
      AT+MPUB=topic,0,0,"payload"       → OK（QoS0，retain0）
      AT+MQTTSTATU                      → +MQTTSTATU :1 已连接 / :0 未连接
    ⚠️ <message> 【必须用双引号括住】——合宙 Air780E AT 手册 MPUB 明写，漏了直接 ERROR
       （2026-10-07 实测：PUB FAIL 全是这个原因）。消息内容里的特殊字符按 \xx 转义：
        双引号 → \22、回车 → \0D、换行 → \0A、反斜杠 → \5C（C 源码写 "\\22" 等）
  - 模块上电后协议栈就绪需 ~30s，此前 AT 命令可能 +CME ERROR:4——
    调用方用 AIR780E_WaitReady() 等待
  - 总开关 AIR780E_ENABLE=0：所有运行期动作短路（零开销零输出）
  ============================================================================*/

#define AIR780E_ENABLE      1       /* 0=停用；1=启用（NET 任务 4G 通道） */

/* 以太网断开多长时间后切 4G（联调期 5s：拉掉网线 5 秒就开始试 4G，反复测不用干等；
 * 正式演示建议调回 30000=30s，甚至 300000=5min） */
#define AIR780E_SWITCH_MS   5000UL

/* 4G 启动失败后的重试间隔（联调期同样取 5s，让"失败→再试"的循环转得快） */
#define AIR780E_RETRY_MS    5000UL

/* ---- 模块硬复位（RST）接线：默认 PE0（本工程 GPIOE 一个脚都没用）----
 * 接法：模块 RST/RESET ← PE0，共地；低电平 ≥100ms 即复位。
 *   ⚠️ 别接 PWRKEY/PWR：那是开关机键，拉低约 2s 是"关机"不是"复位"，反而更麻烦。
 * 换脚：改下面两行，同时把 air780e.c 中 AIR780E_Init 里的 GPIOE 时钟使能
 *       改成对应端口（换脚后 .ioc 里也同步改 GPIO_Label 归属）。
 * AIR780E_RST_ENABLE=0：表示还没接线，驱动里所有复位动作短路（函数恒返回 0） */
#define AIR780E_RST_ENABLE     1
#define AIR780E_RST_PORT       GPIOE
#define AIR780E_RST_PIN        GPIO_PIN_0
#define AIR780E_RST_MS         300UL          /* 复位低电平宽度（手册要求 ≥100ms，留余量） */
#define AIR780E_RST_MIN_GAP_MS 60000UL        /* 复位节流：模块重启要十几秒，60s 内不重复按 */

/* 错误码 */
#define AIR780E_OK          0x00
#define AIR780E_ERR_PARAM   0x01
#define AIR780E_ERR_TIMEOUT 0x02
#define AIR780E_ERR_DISABLED 0x03
#define AIR780E_ERR_FAIL    0x04    /* 命令执行失败（ERROR/CME ERROR） */

void    AIR780E_Init(void);              /* RDY/RST 引脚 + USART3 中断接收启动（可重复调用） */
uint8_t AIR780E_IsOnline(void);          /* 读 RDY 引脚：1=模块已连上服务器 */
uint8_t AIR780E_WaitReady(uint32_t timeoutMs);   /* 等 AT 握手通过（上电后调） */
void    AIR780E_Diag(void);              /* 启动失败诊断：RDY/累计收字节/帧错误计数/AT回话原始hex */
/* 硬复位模块（RST 拉低 AIR780E_RST_MS 后释放，内置 60s 节流）。
 * 返回 1=本次真的按了（调用方应再等十几秒让它启动）；0=被节流跳过 或 未接线 */
uint8_t AIR780E_HardReset(void);
/* 注入喂狗心跳回调：AT 长阻塞（最长 ~19s）期间由驱动回调，避免看门狗误判 NET 任务卡死。
 * 传 0 = 取消。只允许 NET 任务调用前注册一次。 */
void    AIR780E_SetHeartbeatCb(void (*cb)(void));

/* MQTT 会话与发布（AT 固件 M 命令，序列见顶部） */
uint8_t AIR780E_MQTTStart(const char *broker, uint16_t port);       /* MIPSTART+MCONNECT */
uint8_t AIR780E_MQTTPublish(const char *topic, const char *payload);/* MPUB，QoS0，payload 原样 */
uint8_t AIR780E_MQTTPublishJson(const char *topic, const char *json);/* JSON 版：自动把双引号转义为合宙规范形式 */
uint8_t AIR780E_MQTTIsConnected(void);  /* MQTTSTATU：1=连接中 */

/* AT 引擎（通用）：发命令等 expect 子串，resp 带回原文（含 URC，注意甄别） */
uint8_t AIR780E_At(const char *cmd, const char *expect,
                   char *resp, uint16_t respLen, uint32_t timeoutMs);

#endif /* __AIR780E_H */
