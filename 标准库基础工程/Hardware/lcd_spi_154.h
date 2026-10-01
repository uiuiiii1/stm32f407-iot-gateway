#ifndef 	__LCD_130_H
#define	__LCD_130_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>

#include "lcd_fonts.h"		// 图片和字库文件不是必须，用户可自行删减


/*----------------------------------------------- 参数宏 -------------------------------------------*/

#define LCD_Width     240		// LCD的像素长度
#define LCD_Height    240		// LCD的像素宽度

// 显示方向参数
// 使用示例：LCD_DisplayDirection(Direction_H) 设置屏幕横屏显示
#define	Direction_H				0					//LCD横屏显示
#define	Direction_H_Flip	   1					//LCD横屏显示,上下翻转
#define	Direction_V				2					//LCD竖屏显示 
#define	Direction_V_Flip	   3					//LCD竖屏显示,上下翻转 

// 设置变量显示时多余位补0还是补空格
// 只有 LCD_DisplayNumber() 显示整数 和 LCD_DisplayDecimals()显示小数 这两个函数用到
// 使用示例： LCD_ShowNumMode(Fill_Zero) 设置多余位填充0，例如 123 可以显示为 000123
#define  Fill_Zero  0		//填充0
#define  Fill_Space 1		//填充空格


/*---------------------------------------- 常用颜色 ------------------------------------------------------

 1. 这里为了方便用户使用，定义的是24位 RGB888颜色，然后再通过代码自动转换成 16位 RGB565 的颜色
 2. 24位的颜色中，从高位到低位分别对应 R、G、B  3个颜色通道
 3. 用户可以在电脑用调色板获取24位RGB颜色，再将颜色输入LCD_SetColor()或LCD_SetBackColor()就可以显示出相应的颜色 
 */                                                  						
#define 	LCD_WHITE       0xFFFFFF	 // 纯白色
#define 	LCD_BLACK       0x000000    // 纯黑色
                        
#define 	LCD_BLUE        0x0000FF	 //	纯蓝色
#define 	LCD_GREEN       0x00FF00    //	纯绿色
#define 	LCD_RED         0xFF0000    //	纯红色
#define 	LCD_CYAN        0x00FFFF    //	蓝绿色
#define 	LCD_MAGENTA     0xFF00FF    //	紫红色
#define 	LCD_YELLOW      0xFFFF00    //	黄色
#define 	LCD_GREY        0x2C2C2C    //	灰色
												
#define 	LIGHT_BLUE      0x8080FF    //	亮蓝色
#define 	LIGHT_GREEN     0x80FF80    //	亮绿色
#define 	LIGHT_RED       0xFF8080    //	亮红色
#define 	LIGHT_CYAN      0x80FFFF    //	亮蓝绿色
#define 	LIGHT_MAGENTA   0xFF80FF    //	亮紫红色
#define 	LIGHT_YELLOW    0xFFFF80    //	亮黄色
#define 	LIGHT_GREY      0xA3A3A3    //	亮灰色
												
#define 	DARK_BLUE       0x000080    //	暗蓝色
#define 	DARK_GREEN      0x008000    //	暗绿色
#define 	DARK_RED        0x800000    //	暗红色
#define 	DARK_CYAN       0x008080    //	暗蓝绿色
#define 	DARK_MAGENTA    0x800080    //	暗紫红色
#define 	DARK_YELLOW     0x808000    //	暗黄色
#define 	DARK_GREY       0x404040    //	暗灰色


/*------------------------------------------------ 函数声明 ----------------------------------------------*/

void  SPI_LCD_Init(void);      // 液晶屏以及SPI初始化   
void  LCD_Clear(void);			 // 清屏函数
void  LCD_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);	// 局部清屏函数

void  LCD_SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);	// 设置坐标		
void  LCD_SetColor(uint32_t Color); 				   //	设置画笔颜色
void  LCD_SetBackColor(uint32_t Color);  				//	设置背景颜色
void  LCD_SetDirection(uint8_t direction);  	      //	设置显示方向

//>>>>>	显示ASCII字符
void  LCD_SetAsciiFont(pFONT *fonts);										//	设置ASCII字体
void 	LCD_DisplayChar(uint16_t x, uint16_t y,uint8_t c);				//	显示单个ASCII字符
void 	LCD_DisplayString( uint16_t x, uint16_t y, char *p);	 		//	显示ASCII字符串

//>>>>>	显示中文字符，包括ASCII码
void 	LCD_SetTextFont(pFONT *fonts);										// 设置文本字体，包括中文和ASCII字体
void 	LCD_DisplayChinese(uint16_t x, uint16_t y, char *pText);		// 显示单个汉字
void 	LCD_DisplayText(uint16_t x, uint16_t y, char *pText) ;		// 显示字符串，包括中文和ASCII字符

//>>>>>	显示整数或小数
void  LCD_ShowNumMode(uint8_t mode);		// 设置变量显示模式，多余位填充空格还是填充0
void  LCD_DisplayNumber( uint16_t x, uint16_t y, int32_t number,uint8_t len) ;					// 显示整数
void  LCD_DisplayDecimals( uint16_t x, uint16_t y, double number,uint8_t len,uint8_t decs);	// 显示小数

//>>>>>	2D图形函数
void  LCD_DrawPoint(uint16_t x,uint16_t y,uint32_t color);   	//画点

void  LCD_DrawLine_V(uint16_t x, uint16_t y, uint16_t height);          // 画垂直线
void  LCD_DrawLine_H(uint16_t x, uint16_t y, uint16_t width);           // 画水平线
void  LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);	// 两点之间画线

void  LCD_DrawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);			//画矩形
void  LCD_DrawCircle(uint16_t x, uint16_t y, uint16_t r);									//画圆
void  LCD_DrawEllipse(int x, int y, int r1, int r2);											//画椭圆

//>>>>>	区域填充函数
void  LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);			//填充矩形
void  LCD_FillCircle(uint16_t x, uint16_t y, uint16_t r);									//填充圆

//>>>>>	绘制单色图片
void 	LCD_DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage)  ;

/*------------------------------------------------------  引脚配置宏 -------------------------------------------------*/	

#define 	LCD_SPI      SPI3				// LCD用到的是SPI3
#define	LCD_SPI_APBClock_Enable 	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_SPI3,ENABLE ) 	// 使能外设时钟

#define 	LCD_SCK_PIN      		 			GPIO_Pin_3						// SCK引脚， 需要重定义SPI1的IO口复用
#define 	LCD_SCK_PORT     		 			GPIOB                 		// SCK引脚用到的端口  
#define 	GPIO_LCD_SCK_CLK      			RCC_AHB1Periph_GPIOB  		// SCK引脚IO口时钟
#define 	GPIO_LCD_SCK_PinSource       	GPIO_PinSource3
#define 	GPIO_LCD_SCK_AF              	GPIO_AF_SPI3

#define 	LCD_SDA_PIN      		 			GPIO_Pin_5						// SDA引脚， 需要重定义SPI1的IO口复用
#define 	LCD_SDA_PORT    		 			GPIOB                 		// SDA引脚用到的端口  
#define 	GPIO_LCD_SDA_CLK      			RCC_AHB1Periph_GPIOB  		// SDA引脚IO口时钟
#define 	GPIO_LCD_SDA_PinSource       	GPIO_PinSource5
#define 	GPIO_LCD_SDA_AF              	GPIO_AF_SPI3

#define 	LCD_CS_PIN       				GPIO_Pin_15							// CS片选引脚，低电平有效
#define 	LCD_CS_PORT      				GPIOA                 			// CS引脚用到的端口 
#define 	GPIO_LCD_CS_CLK     			RCC_AHB1Periph_GPIOA				// CS引脚IO口时钟

#define  LCD_DC_PIN						GPIO_Pin_13				         // 数据指令选择  引脚				
#define	LCD_DC_PORT						GPIOD									// 数据指令选择  GPIO端口
#define 	GPIO_LCD_DC_CLK     			RCC_AHB1Periph_GPIOD				// 数据指令选择  GPIO时钟 	

#define  LCD_Backlight_PIN				GPIO_Pin_12				         // 背光  引脚				
#define	LCD_Backlight_PORT			GPIOD									// 背光 GPIO端口
#define 	GPIO_LCD_Backlight_CLK     RCC_AHB1Periph_GPIOD				// 背光 GPIO时钟 	


/*--------------------------------------------------------- 控制宏 ---------------------------------------------------*/

// 因为片选引脚需要频繁操作，使用寄存器效率会高些
#define 	LCD_CS_H    		 	LCD_CS_PORT->BSRRL  = LCD_CS_PIN		// 片选拉高
#define 	LCD_CS_L     			LCD_CS_PORT->BSRRH  = LCD_CS_PIN		// 片选拉低

#define	LCD_DC_Command		   GPIO_ResetBits(LCD_DC_PORT, LCD_DC_PIN)	  		// 低电平，指令传输 
#define 	LCD_DC_Data		      GPIO_SetBits(LCD_DC_PORT, LCD_DC_PIN)				// 高电平，数据传输

#define 	LCD_Backlight_ON     GPIO_SetBits(LCD_Backlight_PORT,LCD_Backlight_PIN)		// 高电平，开启背光
#define 	LCD_Backlight_OFF  	GPIO_ResetBits(LCD_Backlight_PORT,LCD_Backlight_PIN)	// 低电平，关闭背光


#endif  // __LCD_130_H


