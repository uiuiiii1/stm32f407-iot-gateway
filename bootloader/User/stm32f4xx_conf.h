#ifndef __STM32F4xx_CONF_H
#define __STM32F4xx_CONF_H

/* 所有外设头文件全部开启，无需按需注释 */
#include "misc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_cec.h"
#include "stm32f4xx_crc.h"
#include "stm32f4xx_cryp.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_dbgmcu.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dfsdm.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dma2d.h"
#include "stm32f4xx_dsi.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_flash_ramfunc.h"
#include "stm32f4xx_fmc.h"
#include "stm32f4xx_fmpi2c.h"
#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_hash.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_lptim.h"
#include "stm32f4xx_ltdc.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_qspi.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_sai.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_spdifrx.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_wwdg.h"

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F4xx_CONF_H */
