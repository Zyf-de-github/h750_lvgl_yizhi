/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__
#include "lv_color.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "stm32h7xx_hal.h"
#include "usart.h"

#include "lcd_fonts.h"	// Í¼Æ¬ºÍ×Ö¿âÎÄ¼þ²»ÊÇ±ØÐë£¬ÓÃ»§¿É×ÔÐÐÉ¾¼õ
#include	"lcd_image.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi4;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


/*----------------------------------------------- ²ÎÊýºê -------------------------------------------*/
#define LCD_Width     240		// LCDµÄÏñËØ³¤¶È
#define LCD_Height    280		// LCDµÄÏñËØ¿í¶È

#define RADIAN(angle)  ((angle==0)?0:(3.14159*angle/180))   //½Ç¶È×ª»»
#define MAX(x,y)  		((x)>(y)? (x):(y))
#define MIN(x,y)  		((x)<(y)? (x):(y))
#define SWAP(x, y) \
	(y) = (x) + (y); \
	(x) = (y) - (x); \
	(y) = (y) - (x);
#define ABS(X)  ((X) > 0 ? (X) : -(X))   //¼ÆËãÊäÈëÊýÖµµÄ¾ø¶ÔÖµ

//¶þÎ¬×ø±ê½á¹¹Ìå£¬°üº¬ºá×ø±ê x ºÍ×Ý×ø±ê y
typedef struct COORDINATE
{
    int x;  // ºá×ø±ê
    int y;  // ×Ý×ø±ê
} TypeXY;

#define point TypeXY

//Ðý×ª½á¹¹Ìå£¬°üº¬Ðý×ªÖÐÐÄ center¡¢Ðý×ª½Ç¶È angle ºÍÐý×ª·½Ïò direct
typedef struct ROATE
{
    TypeXY center;   // Ðý×ªÖÐÐÄ×ø±ê
    float angle;     // Ðý×ª½Ç¶È
    int direct;      // Ðý×ª·½Ïò
} TypeRoate;

// ÏÔÊ¾·½Ïò²ÎÊý
// Ê¹ÓÃÊ¾Àý£ºLCD_DisplayDirection(Direction_H) ÉèÖÃÆÁÄ»ºáÆÁÏÔÊ¾
#define	Direction_H				0					//LCDºáÆÁÏÔÊ¾
#define	Direction_H_Flip	   1					//LCDºáÆÁÏÔÊ¾,ÉÏÏÂ·­×ª
#define	Direction_V				2					//LCDÊúÆÁÏÔÊ¾
#define	Direction_V_Flip	   3					//LCDÊúÆÁÏÔÊ¾,ÉÏÏÂ·­×ª

// ÉèÖÃ±äÁ¿ÏÔÊ¾Ê±¶àÓàÎ»²¹0»¹ÊÇ²¹¿Õ¸ñ
// Ö»ÓÐ LCD_DisplayNumber() ÏÔÊ¾ÕûÊý ºÍ LCD_DisplayDecimals()ÏÔÊ¾Ð¡Êý ÕâÁ½¸öº¯ÊýÓÃµ½
// Ê¹ÓÃÊ¾Àý£º LCD_ShowNumMode(Fill_Zero) ÉèÖÃ¶àÓàÎ»Ìî³ä0£¬ÀýÈç 123 ¿ÉÒÔÏÔÊ¾Îª 000123
#define  Fill_Zero  0		//Ìî³ä0
#define  Fill_Space 1		//Ìî³ä¿Õ¸ñ


/*---------------------------------------- ³£ÓÃÑÕÉ« ------------------------------------------------------

 1. ÕâÀïÎªÁË·½±ãÓÃ»§Ê¹ÓÃ£¬¶¨ÒåµÄÊÇ24Î» RGB888ÑÕÉ«£¬È»ºóÔÙÍ¨¹ý´úÂë×Ô¶¯×ª»»³É 16Î» RGB565 µÄÑÕÉ«
 2. 24Î»µÄÑÕÉ«ÖÐ£¬´Ó¸ßÎ»µ½µÍÎ»·Ö±ð¶ÔÓ¦ R¡¢G¡¢B  3¸öÑÕÉ«Í¨µÀ
 3. ÓÃ»§¿ÉÒÔÔÚµçÄÔÓÃµ÷É«°å»ñÈ¡24Î»RGBÑÕÉ«£¬ÔÙ½«ÑÕÉ«ÊäÈëLCD_SetColor()»òLCD_SetBackColor()¾Í¿ÉÒÔÏÔÊ¾³öÏàÓ¦µÄÑÕÉ«
 */
#define 	LCD_WHITE       0xFFFFFF	 // ´¿°×É«
#define 	LCD_BLACK       0x000000    // ´¿ºÚÉ«

#define 	LCD_BLUE        0x0000FF	 //	´¿À¶É«
#define 	LCD_GREEN       0x00FF00    //	´¿ÂÌÉ«
#define 	LCD_RED         0xFF0000    //	´¿ºìÉ«
#define 	LCD_CYAN        0x00FFFF    //	À¶ÂÌÉ«
#define 	LCD_MAGENTA     0xFF00FF    //	×ÏºìÉ«
#define 	LCD_YELLOW      0xFFFF00    //	»ÆÉ«
#define 	LCD_GREY        0x2C2C2C    //	»ÒÉ«

#define 	LIGHT_BLUE      0x8080FF    //	ÁÁÀ¶É«
#define 	LIGHT_GREEN     0x80FF80    //	ÁÁÂÌÉ«
#define 	LIGHT_RED       0xFF8080    //	ÁÁºìÉ«
#define 	LIGHT_CYAN      0x80FFFF    //	ÁÁÀ¶ÂÌÉ«
#define 	LIGHT_MAGENTA   0xFF80FF    //	ÁÁ×ÏºìÉ«
#define 	LIGHT_YELLOW    0xFFFF80    //	ÁÁ»ÆÉ«
#define 	LIGHT_GREY      0xA3A3A3    //	ÁÁ»ÒÉ«

#define 	DARK_BLUE       0x000080    //	°µÀ¶É«
#define 	DARK_GREEN      0x008000    //	°µÂÌÉ«
#define 	DARK_RED        0x800000    //	°µºìÉ«
#define 	DARK_CYAN       0x008080    //	°µÀ¶ÂÌÉ«
#define 	DARK_MAGENTA    0x800080    //	°µ×ÏºìÉ«
#define 	DARK_YELLOW     0x808000    //	°µ»ÆÉ«
#define 	DARK_GREY       0x404040    //	°µ»ÒÉ«

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */


void MX_SPI4_Init(void);
void  SPI_LCD_Init(void);      // Òº¾§ÆÁÒÔ¼°SPI³õÊ¼»¯
void  LCD_Clear(void);			 // ÇåÆÁº¯Êý
void  LCD_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);	// ¾Ö²¿ÇåÆÁº¯Êý

// void  LCD_color_fill(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint32_t Color);
void LCD_color_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, lv_color_t *color_p);

void  LCD_SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);	// ÉèÖÃ×ø±ê
void  LCD_SetColor(uint32_t Color); 				   //	ÉèÖÃ»­±ÊÑÕÉ«
void  LCD_SetBackColor(uint32_t Color);  				//	ÉèÖÃ±³¾°ÑÕÉ«
void  LCD_SetDirection(uint8_t direction);  	      //	ÉèÖÃÏÔÊ¾·½Ïò

//>>>>>	ÏÔÊ¾ASCII×Ö·û
void  LCD_SetAsciiFont(pFONT *fonts);										//	ÉèÖÃASCII×ÖÌå
void 	LCD_DisplayChar(uint16_t x, uint16_t y,uint8_t c);				//	ÏÔÊ¾µ¥¸öASCII×Ö·û
void 	LCD_DisplayString( uint16_t x, uint16_t y, char *p);	 		//	ÏÔÊ¾ASCII×Ö·û´®

//>>>>>	ÏÔÊ¾ÖÐÎÄ×Ö·û£¬°üÀ¨ASCIIÂë
void 	LCD_SetTextFont(pFONT *fonts);										// ÉèÖÃÎÄ±¾×ÖÌå£¬°üÀ¨ÖÐÎÄºÍASCII×ÖÌå
void 	LCD_DisplayChinese(uint16_t x, uint16_t y, char *pText);		// ÏÔÊ¾µ¥¸öºº×Ö
void 	LCD_DisplayText(uint16_t x, uint16_t y, char *pText) ;		// ÏÔÊ¾×Ö·û´®£¬°üÀ¨ÖÐÎÄºÍASCII×Ö·û

//>>>>>	ÏÔÊ¾ÕûÊý»òÐ¡Êý
void  LCD_ShowNumMode(uint8_t mode);		// ÉèÖÃ±äÁ¿ÏÔÊ¾Ä£Ê½£¬¶àÓàÎ»Ìî³ä¿Õ¸ñ»¹ÊÇÌî³ä0
void  LCD_DisplayNumber( uint16_t x, uint16_t y, int32_t number,uint8_t len) ;					// ÏÔÊ¾ÕûÊý
void  LCD_DisplayDecimals( uint16_t x, uint16_t y, double number,uint8_t len,uint8_t decs);	// ÏÔÊ¾Ð¡Êý

//>>>>>	2DÍ¼ÐÎº¯Êý
void  LCD_DrawPoint(uint16_t x,uint16_t y,uint32_t color);   	//»­µã

void  LCD_DrawLine_V(uint16_t x, uint16_t y, uint16_t height);          // »­´¹Ö±Ïß
void  LCD_DrawLine_H(uint16_t x, uint16_t y, uint16_t width);           // »­Ë®Æ½Ïß
void  LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);	// Á½µãÖ®¼ä»­Ïß

void  LCD_DrawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);			//»­¾ØÐÎ
void  LCD_DrawCircle(uint16_t x, uint16_t y, uint16_t r);									//»­Ô²
void  LCD_DrawEllipse(int x, int y, int r1, int r2);											//»­ÍÖÔ²

//>>>>>	ÇøÓòÌî³äº¯Êý
void  LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);			//Ìî³ä¾ØÐÎ
void  LCD_FillCircle(uint16_t x, uint16_t y, uint16_t r);									//Ìî³äÔ²
void LCD_DrawImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *pImage); // »­Í¼Ïñ
void DrawRoundRect(int x, int y, unsigned char w, unsigned char h, unsigned char r); // »­Ô²½Ç¾ØÐÎ±ß¿ò
void DrawfillRoundRect(int x, int y, unsigned char w, unsigned char h, unsigned char r); // Ìî³äÔ²½Ç¾ØÐÎ
void DrawCircleHelper(int x0, int y0, unsigned char r, unsigned char cornername); // »­Ô²½Ç¸¨Öúº¯Êý
void DrawFillCircleHelper(int x0, int y0, unsigned char r, unsigned char cornername, int delta); // Ìî³äÔ²½Ç¸¨Öúº¯Êý
void DrawFillEllipse(int x0, int y0, int rx, int ry);         // Ìî³äÍÖÔ²

void DrawTriangle(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2); // »­Èý½ÇÐÎ
void DrawFillTriangle(int x0, int y0, int x1, int y1, int x2, int y2); // Ìî³äÈý½ÇÐÎ
void DrawArc(int x, int y, unsigned char r, int angle_start, int angle_end); // »­Ô²»¡
TypeXY GetXY(void);   // »ñÈ¡µ±Ç°×ø±ê
void SetRotateCenter(int x0, int y0);  // ÉèÖÃÐý×ªÖÐÐÄ
void SetAngleDir(int direction);       // ÉèÖÃÐý×ª·½Ïò
void SetAngle(float angle);             // ÉèÖÃÐý×ª½Ç¶È
static void Rotate(int x0, int y0, int *x, int *y, double angle, int direction); // Ðý×ª×ø±ê
float mySqrt(float x);  // ¼ÆËãÆ½·½¸ù
TypeXY GetRotateXY(int x, int y);  // »ñÈ¡Ðý×ªºóµÄ×ø±ê
void MoveTo(int x, int y);  // ÒÆ¶¯µ½Ö¸¶¨×ø±ê
void LineTo(int x, int y);  // »­Ïßµ½Ö¸¶¨×ø±ê
void SetRotateValue(int x, int y, float angle, int direct); // ÉèÖÃÐý×ªÖµ

//>>>>>	»æÖÆµ¥É«Í¼Æ¬
void 	LCD_DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage)  ;

//>>>>>	»æÖÆµ¥É«Í¼Æ¬
void 	LCD_DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage)  ;

//>>>>>	ÅúÁ¿¸´ÖÆº¯Êý£¬Ö±½Ó½«Êý¾Ý¸´ÖÆµ½ÆÁÄ»µÄÏÔ´æ
void	LCD_CopyBuffer(uint16_t x, uint16_t y,uint16_t width,uint16_t height,uint16_t *DataBuff);

 /*--------------------------------------------- LCDÆäËüÒý½Å -----------------------------------------------*/

#define  LCD_Backlight_PIN								GPIO_PIN_15				         // ±³¹â  Òý½Å
#define	LCD_Backlight_PORT							GPIOD									// ±³¹â GPIO¶Ë¿Ú
#define 	GPIO_LDC_Backlight_CLK_ENABLE        	__HAL_RCC_GPIOD_CLK_ENABLE()	// ±³¹â GPIOÊ±ÖÓ

#define	LCD_Backlight_OFF		HAL_GPIO_WritePin(LCD_Backlight_PORT, LCD_Backlight_PIN, GPIO_PIN_RESET);	// µÍµçÆ½£¬¹Ø±Õ±³¹â
#define 	LCD_Backlight_ON		HAL_GPIO_WritePin(LCD_Backlight_PORT, LCD_Backlight_PIN, GPIO_PIN_SET);		// ¸ßµçÆ½£¬¿ªÆô±³¹â

#define  LCD_DC_PIN						GPIO_PIN_15				         // Êý¾ÝÖ¸ÁîÑ¡Ôñ  Òý½Å
#define	LCD_DC_PORT						GPIOE									// Êý¾ÝÖ¸ÁîÑ¡Ôñ  GPIO¶Ë¿Ú
#define 	GPIO_LDC_DC_CLK_ENABLE     __HAL_RCC_GPIOE_CLK_ENABLE()	// Êý¾ÝÖ¸ÁîÑ¡Ôñ  GPIOÊ±ÖÓ

#define	LCD_DC_Command		   HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET);	   // µÍµçÆ½£¬Ö¸Áî´«Êä
#define 	LCD_DC_Data		      HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);		// ¸ßµçÆ½£¬Êý¾Ý´«Êä

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
