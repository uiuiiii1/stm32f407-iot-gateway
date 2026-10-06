/* bootloader 主程序（F407VGT6 标准库版，独立工程）
 * 移植自 F1 工程 07-bootloader程序（标准库）；本阶段只做：横幅 + 读 AT24C64
 * 升级状态 + （有标志时）W25→片内Flash 搬运 + 跳应用。MQTT 下载在应用侧实现 */
#include "stm32f4xx.h"
#include "usart.h"
#include "tick.h"
#include "AT24C64.h"
#include "W25Q64.h"
#include "App_bootloader.h"
#include <stdio.h>

int main(void)
{
    /* SystemInit 已由启动文件调用：HSE 8MHz → PLL → 168MHz（AT24C64 的 DWT 延时依赖此时钟） */
    USART1_Init(115200);
    printf("\r\n=== gw001 bootloader v1.0 ===\r\n");

    Tick_Init();            /* SysTick 1ms：W25_WaitBusy 的超时判据 */

    AT24C64_Init();         /* 必须调用：使能 DWT，否则软 I2C 的 IIC_Delay 死循环卡死 */
    if (W25Q64_Init() != W25Q64_OK)
    {
        /* Flash 不在位：驻留 bootloader 不跳应用（没有 Flash 就失去升级/回滚能力，
         * 且无法确认应用区完整性，直接硬跑应用有风险） */
        printf("w25q64 init error, NOT jumping to app\r\n");
        printf("check W25Q64 power/wiring then reset\r\n");
        while (1)
        {
            /* 停在 bootloader，等待复位/检修 */
        }
    }

    App_bootloader_check_update();   /* 读 EEPROM 升级状态（密钥校验+写回修复） */
    App_bootloader_process();        /* UPDATE→搬运后跳应用；NO_UPDATE→直接跳应用 */

    while (1)
    {
        /* 正常流程不会到达（process 内已跳转）；跳转失败时停在这里便于串口排查 */
    }
}
