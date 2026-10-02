/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f4xx_it.h"   /* Fault_Blink：Error_Handler / 异常时的 LED 故障指示 */
#include "delay.h"
#include "usart_app.h"      /* USART1 调试串口 + printf 重定向 */
#include "lcd_spi_154.h"
#include "w5500_bsp.h"
#include "socket.h"
#include "modbus.h"
#include "mqtt.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 仪表盘运行状态（放文件作用域，避免在 MX_xxx_Init 之后声明局部变量的兼容性问题） */
static int16_t  temp10 = 0;      /* 温度×10（补码），Modbus 最近一次成功值 */
static uint16_t hum10 = 0;       /* 湿度×10 */
static uint8_t  mbFailCnt = 0;   /* Modbus 连续失败计数 */
static uint16_t pubCnt = 0;      /* 发布成功计数 */
static uint32_t tRead = 0, tPub = 0;   /* 2秒读 / 5秒发布 节拍 */
static char     json[80];
static int      jsonLen;
static uint8_t  link = 0xFF;     /* 0xFF=初始未知，强制首刷 */
static uint8_t  linkNow;
static uint8_t  modOk = 1;       /* W5500模块在位状态（版本寄存器可读=在位） */
static uint32_t tOff = 0;        /* OFF状态下周期打印用 */
static uint8_t  verFail = 0;     /* 连续版本读失败计数（防抖：满5次才判离线） */
static uint8_t  linkDownCnt = 0; /* 连续LinkDown计数（防抖：满3次才关socket） */
static uint8_t  cfgFail = 0;     /* 连续配置不一致计数（防抖：满3次才重下发） */
static uint32_t tBeat = 0;       /* PC0 心跳灯节拍 */
static uint8_t  beatPh = 1U;     /* 心跳灯相位：0=本拍已点亮，等待熄灭 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * HAL 工程 —— 阶段7：仪表盘整合（与标准库基础工程功能一致）
 * W5500 接线：SCLK=PA5 / MISO=PA6 / MOSI=PA7（SPI1），nSS=PC4，nRST=PC5
 * 网络：静态IP 192.168.0.250（见 w5500_bsp.h 配置区）；Modbus 变送器接 USART2（DE=PA4）
 *
 * 屏幕布局（240x240，全部固定坐标局部刷新，不整屏Clear）：
 *   y=16  Industrial Gateway
 *   y=40  Link: UP/DOWN（绿/红，防抖）
 *   y=64  MQTT: ONLINE/OFF
 *   y=92  Temp: xx.x C    （Modbus 2秒轮询，负温度带负号）
 *   y=116 Hum : xx.x %
 *   y=144 IP: 192.168.0.250
 *   y=168 PUB: n          （发布成功计数）
 */

/* 链路状态行固定坐标刷新（局部覆盖，绿=通 红=断） */
static void LCD_DisplayLink(uint8_t up)
{
    LCD_SetColor(up ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(64, 40, up ? "UP  " : "DOWN");
    LCD_SetColor(LCD_BLACK);
}

/* MQTT 状态行固定坐标刷新（绿=ONLINE 红=OFF）
 * 内部记住上次画的状态，只在变化时重画；返回1=本次发生了切换（供串口日志用） */
static int LCD_DisplayMqtt(uint8_t ok)
{
    static uint8_t shown = 0xFF;
    if (shown == ok)
        return 0;
    shown = ok;
    LCD_SetColor(ok ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(64, 64, ok ? "ONLINE" : "OFF   ");
    LCD_SetColor(LCD_BLACK);
    return 1;
}

/* 温湿度×10值显示：符号(1字符)+3位整数+小数点+1位小数，共48像素，固定坐标局部覆盖 */
static void LCD_DisplayTenths(uint16_t x, uint16_t y, int16_t v10)
{
    uint16_t abs10;

    if (v10 < 0)
    {
        LCD_DisplayString(x, y, "-");
        abs10 = (uint16_t)(-(int32_t)v10);
    }
    else
    {
        LCD_DisplayString(x, y, " ");
        abs10 = (uint16_t)v10;
    }
    LCD_DisplayNumber(x + 8, y, abs10 / 10, 3);
    LCD_DisplayString(x + 32, y, ".");
    LCD_DisplayNumber(x + 40, y, abs10 % 10, 1);
}

/* 温湿度×10值拼 JSON 数值文本："-" + 整数部分 + "." + 小数部分（不用%f，MicroLIB）
 * 返回写入字符数 */
static int FmtX10(char *out, int16_t v10)
{
    if (v10 < 0)
        return sprintf(out, "-%d.%d", (int)(-(int32_t)v10) / 10, (int)(-(int32_t)v10) % 10);
    return sprintf(out, "%d.%d", (int)v10 / 10, (int)v10 % 10);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* MX_GPIO_Init 按 .ioc 里的默认状态把 PD12 拉低了，这里立刻重新点亮背光 */
  LCD_Backlight_ON;

  /* 时钟/GPIO/SPI/USART/RTC 已由上面的 MX_xxx_Init 配好，这里只做应用层初始化 */
  Delay_Init();
  USART1_Init(115200);
  printf("System Start\r\n");

  SPI_LCD_Init();

  LCD_SetAsciiFont(&ASCII_Font16);  LCD_SetBackColor(LCD_WHITE);
  LCD_SetColor(LCD_BLACK);
  LCD_Clear();
  LCD_ShowNumMode(Fill_Space);  /* 数字不足位补空格 */

  /* 仪表盘静态框架 */
  LCD_DisplayString(16, 16,  "Industrial Gateway");
  LCD_DisplayString(16, 40,  "Link:");
  LCD_DisplayString(16, 64,  "MQTT:");
  LCD_DisplayString(16, 92,  "Temp:");
  LCD_DisplayString(16, 116, "Hum :");
  LCD_DisplayString(112, 92,  "C");
  LCD_DisplayString(112, 116, "%");
  LCD_DisplayString(16, 144, "IP: 192.168.0.250");
  LCD_DisplayString(16, 168, "PUB:");

  MB_USART_Init();
  W5500_BSP_Init();
  W5500_NetworkInit();
  MQTT_Init();
  printf("Gateway ready: IP 192.168.0.250, broker.emqx.io:1883, MQTT pub 5s\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        /* 心跳灯（PC0 用户LED，低电平点亮）：每2秒只"点一下"约80ms，其余全灭。
         * 特意做成和故障播报（0.7s长亮 + 连续几下短闪）完全不同的节奏：
         *   看到这个稀疏的"滴——"= 启动全部成功、主循环在跑；
         *   看到"长亮+一组组短闪"循环 = 故障播报，去数两组各几下；
         *   一直不亮 = 没下载进去 / BOOT0 / 供电。 */
        if ((GetTick() - tBeat) >= 2000U)
        {
            tBeat = GetTick();
            beatPh = 0;
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);   /* 点亮 */
        }
        else if (beatPh == 0U && (GetTick() - tBeat) >= 80U)
        {
            beatPh = 1U;
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);     /* 熄灭 */
        }

        /* 模块在位检测：W5500未供电时MISO上拉会把寄存器读成0xFF，链路读数全不可信。
         * 连续5次坏读才判定离线——单次SPI坏读（数据收发时的电源毛刺）不允许杀死TCP连接 */
        if (W5500_ReadVersion() != 0x04)
        {
            verFail++;
            if (verFail < 5)      /* 瞬时坏读：不动socket，快速重试保住TCP */
            {
                Delay_ms(10);
                continue;
            }

            /* 连续5次失败：确认离线 */
            if (modOk)
            {
                modOk = 0;
                tOff = GetTick();
                LCD_SetColor(LCD_RED);
                LCD_DisplayString(64, 40, "OFF ");
                LCD_DisplayMqtt(0);
                LCD_SetColor(LCD_BLACK);
                printf("W5500 module offline (5 consecutive bad version reads)\r\n");
            }
            if ((GetTick() - tOff) >= 2000)   /* 每2秒报一次实测版本值，用于判线 */
            {
                tOff = GetTick();
                printf("module offline, version read: 0x%02X spi1err=%d (0xFF=floating 0x00=stuck low)\r\n",
                       W5500_ReadVersion(), (int)W5500_BSP_SPI1Err());
            }
            /* 注意：确认离线后严禁触碰任何socket API（ioLibrary close 会死等 0xFF 读数导致冻结）；
             * 模块恢复供电后其Socket硬件自动回到CLOSED，无需远程关闭 */
            Delay_ms(500);
            continue;
        }
        verFail = 0;              /* 版本读正常：清零失败计数 */
        if (!modOk)                   /* 模块恢复供电：复位状态，强制刷新链路显示 */
        {
            modOk = 1;
            link = 0xFF;
            printf("W5500 module online\r\n");
        }

        /* 配置自愈（防抖）：芯片被意外复位（干扰/电源抖动）会丢失IP配置但PHY仍显示Link。
         * 连续3次校验不一致才重新下发——单次SPI坏读不触发重配 */
        if (W5500_CheckConfig() == 0)
        {
            if (cfgFail < 3)
                cfgFail++;
            if (cfgFail >= 3)
            {
                cfgFail = 0;
                link = 0xFF;              /* 芯片复位过，强制刷新链路显示 */
                printf("W5500 config lost (chip reset?) - re-applied\r\n");
                W5500_NetworkInit();
            }
        }
        else
            cfgFail = 0;

        /* 链路状态监测（变化才刷屏/打印） */
        linkNow = W5500_LinkStatus();
        if (linkNow != link)
        {
            link = linkNow;
            LCD_DisplayLink(link);
            printf("PHY %s\r\n", link ? "LinkUp" : "LinkDown");
        }

        if (!link)
        {
            /* 防抖：连续3次(~30ms)读都断才关socket——单次PHY读数毛刺不杀TCP连接 */
            if (linkDownCnt < 3)
            {
                linkDownCnt++;
                Delay_ms(10);
                continue;
            }
            linkDownCnt = 0;
            if (getSn_SR(MQTT_SOCK) != SOCK_CLOSED)
                close(MQTT_SOCK);         /* 链路确认断开：关socket，MQTT状态机自会重连 */
            LCD_DisplayMqtt(0);
            Delay_ms(200);
            continue;
        }
        linkDownCnt = 0;

        /* MQTT 状态机（模块在线 + 链路UP 区域内）：连接/保活/自动重连 */
        if (MQTT_Process() == MQ_MQTT_OK)
        {
            if (LCD_DisplayMqtt(1))
                printf("MQTT: ONLINE\r\n");

            /* 每5秒发布 JSON（温湿度用最近一次成功值） */
            if ((GetTick() - tPub) >= 5000)
            {
                tPub = GetTick();
                jsonLen = sprintf(json, "{\"device\":\"gw001\",\"temp\":");
                jsonLen += FmtX10(json + jsonLen, temp10);
                jsonLen += sprintf(json + jsonLen, ",\"hum\":");
                jsonLen += FmtX10(json + jsonLen, (int16_t)hum10);
                jsonLen += sprintf(json + jsonLen, "}");

                if (MQTT_Publish(MQTT_TOPIC, (const uint8_t *)json, (uint16_t)jsonLen) == MQTT_OK)
                {
                    pubCnt++;
                    printf("PUB[%d]: %s\r\n", pubCnt, json);
                    LCD_DisplayNumber(48, 192, pubCnt, 5);
                }
            }
        }
        else
        {
            LCD_DisplayMqtt(0);
            /* 离线期节拍基准前移：避免恢复在线瞬间连发 */
            tPub = GetTick();
        }

        /* 阶段7：每2秒 Modbus 读温湿度并刷新仪表盘（不依赖网络状态，失败保持上次显示） */
        if ((GetTick() - tRead) >= 2000)
        {
            tRead = GetTick();
            if (MODBUS_ReadTempHum(&temp10, &hum10) == MB_OK)
            {
                mbFailCnt = 0;
                LCD_DisplayTenths(64, 92, temp10);
                LCD_DisplayTenths(64, 116, (int16_t)hum10);
            }
            else
            {
                mbFailCnt++;
                if (mbFailCnt <= 2 || mbFailCnt % 10 == 0)
                    printf("Modbus read fail x%d (keep last display)\r\n", mbFailCnt);
            }
        }

        Delay_ms(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  /* 手工修改（CubeMX 重新生成会覆盖，需重打）：
   * LSE(32.768K) 起振失败时不让整板卡死在 Error_Handler —— 标准库工程从来不开 LSE，
   * 所以同一块板子标准库能跑、HAL 版全黑，最大嫌疑就是这里。
   * 失败则降级：关 LSE、开 LSI 做 RTC 时钟（RTC 仍可走时，只是断电不保持）。 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 初始化失败：PC0 LED 快闪报错（寄存器直写，不依赖外设初始化结果），
   * 不静默死循环——避免"程序没跑起来"和"初始化失败"分不清。 */
  Fault_Blink();
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
