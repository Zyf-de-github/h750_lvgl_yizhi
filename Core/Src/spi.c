/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usart.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
	* @note:     ÔÚÊ¹ÓÃ´ËÍ·ÎÄ¼þÊ±£¬ÐèÈ·±£Ïà¹ØµÄÒÀÀµÏîÒÑÕýÈ·ÅäÖÃ£¬
	*
	* @contact:  Î¢ÐÅ¹«ÖÚºÅ - [GenBotter]£¬·ÖÏí¼¼ÊõÄÚÈÝ
  *            QQÈººÅ - [366182133]£¬ÔÚÏß¼¼ÊõÖ§³Ö
  *            ÌÔ±¦µêÆÌ - [https://genbotter.taobao.com]£¬Ìá¹©ÅäÌ×Ó²¼þ²úÆ·
	*
>>>>> ÖØÒªËµÃ÷£º
	*
	*  1.ÆÁÄ»ÅäÖÃÎª16Î»RGB565¸ñÊ½
	*  2.SPIÍ¨ÐÅËÙ¶ÈÎª60M£¬ÇåÆÁ LCD_Clear() ËùÐèÊ±¼äÎª18ms£¬Ô¼Îª55.5Ö¡
   *
>>>>> ÆäËûËµÃ÷£º
	*
	*	1. ÖÐÎÄ×Ö¿âÊ¹ÓÃµÄÊÇÐ¡×Ö¿â£¬¼´ÓÃµ½ÁË¶ÔÓ¦µÄºº×ÖÔÙÈ¥È¡Ä££¬ÓÃ»§¿ÉÒÔ¸ù¾ÝÐèÇó×ÔÐÐÔöÌí»òÉ¾¼õ
	*	2. ¸÷¸öº¯ÊýµÄ¹¦ÄÜºÍÊ¹ÓÃ¿ÉÒÔ²Î¿¼º¯ÊýµÄËµÃ÷
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
/* USER CODE END 0 */

SPI_HandleTypeDef hspi4;

#define  LCD_SPI hspi4           // SPI¾Ö²¿ºê£¬·½±ãÐÞ¸ÄºÍÒÆÖ²

static pFONT *LCD_AsciiFonts;		// Ó¢ÎÄ×ÖÌå£¬ASCII×Ö·û¼¯
static pFONT *LCD_CHFonts;		   // ÖÐÎÄ×ÖÌå£¨Í¬Ê±Ò²°üº¬Ó¢ÎÄ×ÖÌå£©

static int  _pointx=0;
static int 	_pointy=0;
unsigned char ScreenBuffer[8][240]={0};   //8  240
TypeRoate _RoateValue={{0,0},0,1}; //±£´æÐý×ª²Ù×÷µÄÏà¹ØÐÅÏ¢

// ÒòÎªÕâÀàSPIµÄÆÁÄ»£¬Ã¿´Î¸üÐÂÏÔÊ¾Ê±£¬ÐèÒªÏÈÅäÖÃ×ø±êÇøÓò¡¢ÔÙÐ´ÏÔ´æ£¬
// ÔÚÏÔÊ¾×Ö·ûÊ±£¬Èç¹ûÊÇÒ»¸ö¸öµãÈ¥Ð´×ø±êÐ´ÏÔ´æ£¬»á·Ç³£Âý£¬
// Òò´Ë¿ª±ÙÒ»Æ¬»º³åÇø£¬ÏÈ½«ÐèÒªÏÔÊ¾µÄÊý¾ÝÐ´½ø»º³åÇø£¬×îºóÔÙÅúÁ¿Ð´ÈëÏÔ´æ¡£
// ÓÃ»§¿ÉÒÔ¸ù¾ÝÊµ¼ÊÇé¿öÈ¥ÐÞ¸Ä´Ë´¦»º³åÇøµÄ´óÐ¡£¬
// ÀýÈç£¬ÓÃ»§ÐèÒªÏÔÊ¾32*32µÄºº×ÖÊ±£¬ÐèÒªµÄ´óÐ¡Îª 32*32*2 = 2048 ×Ö½Ú£¨Ã¿¸öÏñËØµãÕ¼2×Ö½Ú£©
uint16_t  LCD_Buff[1024];        // LCD»º³åÇø£¬16Î»¿í£¨Ã¿¸öÏñËØµãÕ¼2×Ö½Ú£©

struct	//LCDÏà¹Ø²ÎÊý½á¹¹Ìå
{
	uint32_t Color;  				//	LCDµ±Ç°»­±ÊÑÕÉ«
	uint32_t BackColor;			//	±³¾°É«
   uint8_t  ShowNum_Mode;		// Êý×ÖÏÔÊ¾Ä£Ê½
	uint8_t  Direction;			//	ÏÔÊ¾·½Ïò
   uint16_t Width;            // ÆÁÄ»ÏñËØ³¤¶È
   uint16_t Height;           // ÆÁÄ»ÏñËØ¿í¶È
   uint8_t  X_Offset;         // X×ø±êÆ«ÒÆ£¬ÓÃÓÚÉèÖÃÆÁÄ»¿ØÖÆÆ÷µÄÏÔ´æÐ´Èë·½Ê½
   uint8_t  Y_Offset;         // Y×ø±êÆ«ÒÆ£¬ÓÃÓÚÉèÖÃÆÁÄ»¿ØÖÆÆ÷µÄÏÔ´æÐ´Èë·½Ê½
}LCD;

// ¸Ãº¯ÊýÐÞ¸ÄÓÚHALµÄSPI¿âº¯Êý£¬×¨Îª LCD_Clear() ÇåÆÁº¯ÊýÐÞ¸Ä£¬
// Ä¿µÄÊÇÎªÁËSPI´«ÊäÊý¾Ý²»ÏÞÊý¾Ý³¤¶ÈµÄÐ´Èë
HAL_StatusTypeDef LCD_SPI_Transmit(SPI_HandleTypeDef *hspi, uint16_t pData, uint32_t Size);
HAL_StatusTypeDef LCD_SPI_TransmitBuffer (SPI_HandleTypeDef *hspi, uint16_t *pData, uint32_t Size);


/* SPI4 init function */
void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;

  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;

  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_02DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(spiHandle->Instance==SPI4)
  {
  /* USER CODE BEGIN SPI4_MspInit 0 */

  /* USER CODE END SPI4_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
    PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI4 clock enable */
    __HAL_RCC_SPI4_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SPI4 GPIO Configuration
    PE11     ------> SPI4_NSS
    PE12     ------> SPI4_SCK
    PE14     ------> SPI4_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

// ³õÊ¼»¯ ±³¹â Òý½Å
		GPIO_InitStruct.Pin 		= LCD_Backlight_PIN;				// ±³¹â Òý½Å
		GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;			// ÍÆÍìÊä³öÄ£Ê½
		GPIO_InitStruct.Pull 	= GPIO_NOPULL;						// ÎÞÉÏÏÂÀ­
		GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;			// ËÙ¶ÈµÈ¼¶µÍ
		HAL_GPIO_Init(LCD_Backlight_PORT, &GPIO_InitStruct);	// ³õÊ¼»¯

// ³õÊ¼»¯ Êý¾ÝÖ¸ÁîÑ¡Ôñ Òý½Å
		GPIO_InitStruct.Pin 		= LCD_DC_PIN;				      // Êý¾ÝÖ¸ÁîÑ¡Ôñ Òý½Å
		GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;			// ÍÆÍìÊä³öÄ£Ê½
		GPIO_InitStruct.Pull 	= GPIO_NOPULL;						// ÎÞÉÏÏÂÀ­
		GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;			// ËÙ¶ÈµÈ¼¶µÍ
		HAL_GPIO_Init(LCD_DC_PORT, &GPIO_InitStruct);	      // ³õÊ¼»¯

  /* USER CODE BEGIN SPI4_MspInit 1 */

  /* USER CODE END SPI4_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI4)
  {
  /* USER CODE BEGIN SPI4_MspDeInit 0 */

  /* USER CODE END SPI4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI4_CLK_DISABLE();

    /**SPI4 GPIO Configuration
    PE11     ------> SPI4_NSS
    PE12     ------> SPI4_SCK
    PE14     ------> SPI4_MOSI
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14);

  /* USER CODE BEGIN SPI4_MspDeInit 1 */

  /* USER CODE END SPI4_MspDeInit 1 */
  }
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_WriteCommand
*
*	Èë¿Ú²ÎÊý: lcd_command - ÐèÒªÐ´ÈëµÄ¿ØÖÆÖ¸Áî
*
*	º¯Êý¹¦ÄÜ: ÓÃÓÚÏòÆÁÄ»¿ØÖÆÆ÷Ð´ÈëÖ¸Áî
*
****************************************************************************************************************************************/

void  LCD_WriteCommand(uint8_t lcd_command)
{
   LCD_DC_Command;     // Êý¾ÝÖ¸ÁîÑ¡Ôñ Òý½ÅÊä³öµÍµçÆ½£¬´ú±í±¾´Î´«Êä Ö¸Áî

   HAL_SPI_Transmit(&LCD_SPI, &lcd_command, 1, 1000); // Æô¶¯SPI´«Êä
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_WriteData_8bit
*
*	Èë¿Ú²ÎÊý: lcd_data - ÐèÒªÐ´ÈëµÄÊý¾Ý£¬8Î»
*
*	º¯Êý¹¦ÄÜ: Ð´Èë8Î»Êý¾Ý
*
****************************************************************************************************************************************/

void  LCD_WriteData_8bit(uint8_t lcd_data)
{
   LCD_DC_Data;     // Êý¾ÝÖ¸ÁîÑ¡Ôñ Òý½ÅÊä³ö¸ßµçÆ½£¬´ú±í±¾´Î´«Êä Êý¾Ý

   HAL_SPI_Transmit(&LCD_SPI, &lcd_data, 1, 1000) ; // Æô¶¯SPI´«Êä
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_WriteData_16bit
*
*	Èë¿Ú²ÎÊý: lcd_data - ÐèÒªÐ´ÈëµÄÊý¾Ý£¬16Î»
*
*	º¯Êý¹¦ÄÜ: Ð´Èë16Î»Êý¾Ý
*
****************************************************************************************************************************************/

void  LCD_WriteData_16bit(uint16_t lcd_data)
{
   uint8_t lcd_data_buff[2];    // Êý¾Ý·¢ËÍÇø
   LCD_DC_Data;      // Êý¾ÝÖ¸ÁîÑ¡Ôñ Òý½ÅÊä³ö¸ßµçÆ½£¬´ú±í±¾´Î´«Êä Êý¾Ý

   lcd_data_buff[0] = lcd_data>>8;  // ½«Êý¾Ý²ð·Ö
   lcd_data_buff[1] = lcd_data;

	HAL_SPI_Transmit(&LCD_SPI, lcd_data_buff, 2, 1000) ;   // Æô¶¯SPI´«Êä
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_WriteBuff
*
*	Èë¿Ú²ÎÊý: DataBuff - Êý¾ÝÇø£¬DataSize - Êý¾Ý³¤¶È
*
*	º¯Êý¹¦ÄÜ: ÅúÁ¿Ð´ÈëÊý¾Ýµ½ÆÁÄ»
*
****************************************************************************************************************************************/

void  LCD_WriteBuff(uint16_t *DataBuff, uint16_t DataSize)
{
	LCD_DC_Data;     // Êý¾ÝÖ¸ÁîÑ¡Ôñ Òý½ÅÊä³ö¸ßµçÆ½£¬´ú±í±¾´Î´«Êä Êý¾Ý

// ÐÞ¸ÄÎª16Î»Êý¾Ý¿í¶È£¬Ð´ÈëÊý¾Ý¸ü¼ÓÐ§ÂÊ£¬²»ÐèÒª²ð·Ö
   LCD_SPI.Init.DataSize 	= SPI_DATASIZE_16BIT;   //	16Î»Êý¾Ý¿í¶È
   HAL_SPI_Init(&LCD_SPI);

	HAL_SPI_Transmit(&LCD_SPI, (uint8_t *)DataBuff, DataSize, 1000) ; // Æô¶¯SPI´«Êä

// ¸Ä»Ø8Î»Êý¾Ý¿í¶È£¬ÒòÎªÖ¸ÁîºÍ²¿·ÖÊý¾Ý¶¼ÊÇ°´ÕÕ8Î»´«ÊäµÄ
	LCD_SPI.Init.DataSize 	= SPI_DATASIZE_8BIT;    //	8Î»Êý¾Ý¿í¶È
   HAL_SPI_Init(&LCD_SPI);
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû: SPI_LCD_Init
*
*	º¯Êý¹¦ÄÜ: ³õÊ¼»¯SPIÒÔ¼°ÆÁÄ»¿ØÖÆÆ÷µÄ¸÷ÖÖ²ÎÊý
*
****************************************************************************************************************************************/

void SPI_LCD_Init(void)
{
   MX_SPI4_Init();               // ³õÊ¼»¯SPIºÍ¿ØÖÆÒý½Å

   HAL_Delay(10);               	// ÆÁÄ»¸ÕÍê³É¸´Î»Ê±£¨°üÀ¨ÉÏµç¸´Î»£©£¬ÐèÒªµÈ´ýÖÁÉÙ5ms²ÅÄÜ·¢ËÍÖ¸Áî

 	LCD_WriteCommand(0x36);       // ÏÔ´æ·ÃÎÊ¿ØÖÆ Ö¸Áî£¬ÓÃÓÚÉèÖÃ·ÃÎÊÏÔ´æµÄ·½Ê½
	LCD_WriteData_8bit(0x00);     // ÅäÖÃ³É ´ÓÉÏµ½ÏÂ¡¢´Ó×óµ½ÓÒ£¬RGBÏñËØ¸ñÊ½

	LCD_WriteCommand(0x3A);			// ½Ó¿ÚÏñËØ¸ñÊ½ Ö¸Áî£¬ÓÃÓÚÉèÖÃÊ¹ÓÃ 12Î»¡¢16Î»»¹ÊÇ18Î»É«
	LCD_WriteData_8bit(0x05);     // ´Ë´¦ÅäÖÃ³É 16Î» ÏñËØ¸ñÊ½

// ½ÓÏÂÀ´ºÜ¶à¶¼ÊÇµçÑ¹ÉèÖÃÖ¸Áî£¬Ö±½ÓÊ¹ÓÃ³§¼Ò¸øÉè¶¨Öµ
 	LCD_WriteCommand(0xB2);
	LCD_WriteData_8bit(0x0C);
	LCD_WriteData_8bit(0x0C);
	LCD_WriteData_8bit(0x00);
	LCD_WriteData_8bit(0x33);
	LCD_WriteData_8bit(0x33);

	LCD_WriteCommand(0xB7);		   // Õ¤¼«µçÑ¹ÉèÖÃÖ¸Áî
	LCD_WriteData_8bit(0x35);     // VGH = 13.26V£¬VGL = -10.43V

	LCD_WriteCommand(0xBB);			// ¹«¹²µçÑ¹ÉèÖÃÖ¸Áî
	LCD_WriteData_8bit(0x19);     // VCOM = 1.35V

	LCD_WriteCommand(0xC0);
	LCD_WriteData_8bit(0x2C);

	LCD_WriteCommand(0xC2);       // VDV ºÍ VRH À´Ô´ÉèÖÃ
	LCD_WriteData_8bit(0x01);     // VDV ºÍ VRH ÓÉÓÃ»§×ÔÓÉÅäÖÃ

	LCD_WriteCommand(0xC3);			// VRHµçÑ¹ ÉèÖÃÖ¸Áî
	LCD_WriteData_8bit(0x12);     // VRHµçÑ¹ = 4.6+( vcom+vcom offset+vdv)

	LCD_WriteCommand(0xC4);		   // VDVµçÑ¹ ÉèÖÃÖ¸Áî
	LCD_WriteData_8bit(0x20);     // VDVµçÑ¹ = 0v

	LCD_WriteCommand(0xC6); 		// Õý³£Ä£Ê½µÄÖ¡ÂÊ¿ØÖÆÖ¸Áî
	LCD_WriteData_8bit(0x0F);   	// ÉèÖÃÆÁÄ»¿ØÖÆÆ÷µÄË¢ÐÂÖ¡ÂÊÎª60Ö¡

	LCD_WriteCommand(0xD0);			// µçÔ´¿ØÖÆÖ¸Áî
	LCD_WriteData_8bit(0xA4);     // ÎÞÐ§Êý¾Ý£¬¹Ì¶¨Ð´Èë0xA4
	LCD_WriteData_8bit(0xA1);     // AVDD = 6.8V £¬AVDD = -4.8V £¬VDS = 2.3V

	LCD_WriteCommand(0xE0);       // Õý¼«µçÑ¹Ù¤ÂíÖµÉè¶¨
	LCD_WriteData_8bit(0xD0);
	LCD_WriteData_8bit(0x04);
	LCD_WriteData_8bit(0x0D);
	LCD_WriteData_8bit(0x11);
	LCD_WriteData_8bit(0x13);
	LCD_WriteData_8bit(0x2B);
	LCD_WriteData_8bit(0x3F);
	LCD_WriteData_8bit(0x54);
	LCD_WriteData_8bit(0x4C);
	LCD_WriteData_8bit(0x18);
	LCD_WriteData_8bit(0x0D);
	LCD_WriteData_8bit(0x0B);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x23);

	LCD_WriteCommand(0xE1);      // ¸º¼«µçÑ¹Ù¤ÂíÖµÉè¶¨
	LCD_WriteData_8bit(0xD0);
	LCD_WriteData_8bit(0x04);
	LCD_WriteData_8bit(0x0C);
	LCD_WriteData_8bit(0x11);
	LCD_WriteData_8bit(0x13);
	LCD_WriteData_8bit(0x2C);
	LCD_WriteData_8bit(0x3F);
	LCD_WriteData_8bit(0x44);
	LCD_WriteData_8bit(0x51);
	LCD_WriteData_8bit(0x2F);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x20);
	LCD_WriteData_8bit(0x23);

	LCD_WriteCommand(0x21);       // ´ò¿ª·´ÏÔ£¬ÒòÎªÃæ°åÊÇ³£ºÚÐÍ£¬²Ù×÷ÐèÒª·´¹ýÀ´

 // ÍË³öÐÝÃßÖ¸Áî£¬LCD¿ØÖÆÆ÷ÔÚ¸ÕÉÏµç¡¢¸´Î»Ê±£¬»á×Ô¶¯½øÈëÐÝÃßÄ£Ê½ £¬Òò´Ë²Ù×÷ÆÁÄ»Ö®Ç°£¬ÐèÒªÍË³öÐÝÃß
	LCD_WriteCommand(0x11);       // ÍË³öÐÝÃß Ö¸Áî
   HAL_Delay(120);               // ÐèÒªµÈ´ý120ms£¬ÈÃµçÔ´µçÑ¹ºÍÊ±ÖÓµçÂ·ÎÈ¶¨ÏÂÀ´

 // ´ò¿ªÏÔÊ¾Ö¸Áî£¬LCD¿ØÖÆÆ÷ÔÚ¸ÕÉÏµç¡¢¸´Î»Ê±£¬»á×Ô¶¯¹Ø±ÕÏÔÊ¾
	LCD_WriteCommand(0x29);       // ´ò¿ªÏÔÊ¾

// ÒÔÏÂ½øÐÐÒ»Ð©Çý¶¯µÄÄ¬ÈÏÉèÖÃ
   LCD_SetDirection(Direction_V);  	      //	ÉèÖÃÏÔÊ¾·½Ïò
	LCD_SetBackColor(LCD_BLACK);           // ÉèÖÃ±³¾°É«
 	LCD_SetColor(LCD_WHITE);               // ÉèÖÃ»­±ÊÉ«
	LCD_Clear();                           // ÇåÆÁ

   LCD_SetAsciiFont(&ASCII_Font12);       // ÉèÖÃÄ¬ÈÏ×ÖÌå
   LCD_ShowNumMode(Fill_Zero);	      	// ÉèÖÃ±äÁ¿ÏÔÊ¾Ä£Ê½£¬¶àÓàÎ»Ìî³ä¿Õ¸ñ»¹ÊÇÌî³ä0

// È«²¿ÉèÖÃÍê±ÏÖ®ºó£¬´ò¿ª±³¹â
   LCD_Backlight_ON;  // Òý½ÅÊä³ö¸ßµçÆ½µãÁÁ±³¹â
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	 LCD_SetAddress
*
*	Èë¿Ú²ÎÊý:	 x1 - ÆðÊ¼Ë®Æ½×ø±ê   y1 - ÆðÊ¼´¹Ö±×ø±ê
*              x2 - ÖÕµãË®Æ½×ø±ê   y2 - ÖÕµã´¹Ö±×ø±ê
*
*	º¯Êý¹¦ÄÜ:   ÉèÖÃÐèÒªÏÔÊ¾µÄ×ø±êÇøÓò
*****************************************************************************************************************************************/
void LCD_color_fill(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint32_t Color)
{
	LCD_WriteCommand(0x2a);			//	ÁÐµØÖ·ÉèÖÃ£¬¼´X×ø±ê
	LCD_WriteData_16bit(x1+LCD.X_Offset);
	LCD_WriteData_16bit(x2+LCD.X_Offset);

	LCD_WriteCommand(0x2b);			//	ÐÐµØÖ·ÉèÖÃ£¬¼´Y×ø±ê
	LCD_WriteData_16bit(y1+LCD.Y_Offset);
	LCD_WriteData_16bit(y2+LCD.Y_Offset);

	LCD_WriteCommand(0x2c);			//	¿ªÊ¼Ð´ÈëÏÔ´æ£¬¼´ÒªÏÔÊ¾µÄÑÕÉ«Êý¾Ý

	uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0; //¸÷¸öÑÕÉ«Í¨µÀµÄÖµ

	Red_Value   = (uint16_t)((Color&0x00F80000)>>8);   // ×ª»»³É 16Î» µÄRGB565ÑÕÉ«
	Green_Value = (uint16_t)((Color&0x0000FC00)>>5);
	Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);

	LCD.Color = (uint16_t)(Red_Value | Green_Value | Blue_Value);  // ½«ÑÕÉ«Ð´ÈëÈ«¾ÖLCD²ÎÊý
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_SetColor
*
*	Èë¿Ú²ÎÊý:	Color - ÒªÏÔÊ¾µÄÑÕÉ«£¬Ê¾Àý£º0x0000FF ±íÊ¾À¶É«
*
*	º¯Êý¹¦ÄÜ:	´Ëº¯ÊýÓÃÓÚÉèÖÃ»­±ÊµÄÑÕÉ«£¬ÀýÈçÏÔÊ¾×Ö·û¡¢»­µã»­Ïß¡¢»æÍ¼µÄÑÕÉ«
*
*	Ëµ    Ã÷:	1. ÎªÁË·½±ãÓÃ»§Ê¹ÓÃ×Ô¶¨ÒåÑÕÉ«£¬Èë¿Ú²ÎÊý Color Ê¹ÓÃ24Î» RGB888µÄÑÕÉ«¸ñÊ½£¬ÓÃ»§ÎÞÐè¹ØÐÄÑÕÉ«¸ñÊ½µÄ×ª»»
*					2. 24Î»µÄÑÕÉ«ÖÐ£¬´Ó¸ßÎ»µ½µÍÎ»·Ö±ð¶ÔÓ¦ R¡¢G¡¢B  3¸öÑÕÉ«Í¨µÀ
*
*****************************************************************************************************************************************/
void LCD_SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	LCD_WriteCommand(0x2a);			//	ÁÐµØÖ·ÉèÖÃ£¬¼´X×ø±ê
	LCD_WriteData_16bit(x1+LCD.X_Offset);
	LCD_WriteData_16bit(x2+LCD.X_Offset);

	LCD_WriteCommand(0x2b);			//	ÐÐµØÖ·ÉèÖÃ£¬¼´Y×ø±ê
	LCD_WriteData_16bit(y1+LCD.Y_Offset);
	LCD_WriteData_16bit(y2+LCD.Y_Offset);

	LCD_WriteCommand(0x2c);			//	¿ªÊ¼Ð´ÈëÏÔ´æ£¬¼´ÒªÏÔÊ¾µÄÑÕÉ«Êý¾Ý
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_SetColor
*
*	Èë¿Ú²ÎÊý:	Color - ÒªÏÔÊ¾µÄÑÕÉ«£¬Ê¾Àý£º0x0000FF ±íÊ¾À¶É«
*
*	º¯Êý¹¦ÄÜ:	´Ëº¯ÊýÓÃÓÚÉèÖÃ»­±ÊµÄÑÕÉ«£¬ÀýÈçÏÔÊ¾×Ö·û¡¢»­µã»­Ïß¡¢»æÍ¼µÄÑÕÉ«
*
*	Ëµ    Ã÷:	1. ÎªÁË·½±ãÓÃ»§Ê¹ÓÃ×Ô¶¨ÒåÑÕÉ«£¬Èë¿Ú²ÎÊý Color Ê¹ÓÃ24Î» RGB888µÄÑÕÉ«¸ñÊ½£¬ÓÃ»§ÎÞÐè¹ØÐÄÑÕÉ«¸ñÊ½µÄ×ª»»
*					2. 24Î»µÄÑÕÉ«ÖÐ£¬´Ó¸ßÎ»µ½µÍÎ»·Ö±ð¶ÔÓ¦ R¡¢G¡¢B  3¸öÑÕÉ«Í¨µÀ
*
*****************************************************************************************************************************************/

void LCD_SetColor(uint32_t Color)
{
	uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0; //¸÷¸öÑÕÉ«Í¨µÀµÄÖµ

	Red_Value   = (uint16_t)((Color&0x00F80000)>>8);   // ×ª»»³É 16Î» µÄRGB565ÑÕÉ«
	Green_Value = (uint16_t)((Color&0x0000FC00)>>5);
	Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);

	LCD.Color = (uint16_t)(Red_Value | Green_Value | Blue_Value);  // ½«ÑÕÉ«Ð´ÈëÈ«¾ÖLCD²ÎÊý
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_SetBackColor
*
*	Èë¿Ú²ÎÊý:	Color - ÒªÏÔÊ¾µÄÑÕÉ«£¬Ê¾Àý£º0x0000FF ±íÊ¾À¶É«
*
*	º¯Êý¹¦ÄÜ:	ÉèÖÃ±³¾°É«,´Ëº¯ÊýÓÃÓÚÇåÆÁÒÔ¼°ÏÔÊ¾×Ö·ûµÄ±³¾°É«
*
*	Ëµ    Ã÷:	1. ÎªÁË·½±ãÓÃ»§Ê¹ÓÃ×Ô¶¨ÒåÑÕÉ«£¬Èë¿Ú²ÎÊý Color Ê¹ÓÃ24Î» RGB888µÄÑÕÉ«¸ñÊ½£¬ÓÃ»§ÎÞÐè¹ØÐÄÑÕÉ«¸ñÊ½µÄ×ª»»
*					2. 24Î»µÄÑÕÉ«ÖÐ£¬´Ó¸ßÎ»µ½µÍÎ»·Ö±ð¶ÔÓ¦ R¡¢G¡¢B  3¸öÑÕÉ«Í¨µÀ
*
*****************************************************************************************************************************************/

void LCD_SetBackColor(uint32_t Color)
{
	uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0; //¸÷¸öÑÕÉ«Í¨µÀµÄÖµ

	Red_Value   = (uint16_t)((Color&0x00F80000)>>8);   // ×ª»»³É 16Î» µÄRGB565ÑÕÉ«
	Green_Value = (uint16_t)((Color&0x0000FC00)>>5);
	Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);

	LCD.BackColor = (uint16_t)(Red_Value | Green_Value | Blue_Value);	// ½«ÑÕÉ«Ð´ÈëÈ«¾ÖLCD²ÎÊý
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_SetDirection
*
*	Èë¿Ú²ÎÊý:	direction - ÒªÏÔÊ¾µÄ·½Ïò
*
*	º¯Êý¹¦ÄÜ:	ÉèÖÃÒªÏÔÊ¾µÄ·½Ïò
*
*	Ëµ    Ã÷:   1. ¿ÉÊäÈë²ÎÊý Direction_H ¡¢Direction_V ¡¢Direction_H_Flip ¡¢Direction_V_Flip
*              2. Ê¹ÓÃÊ¾Àý LCD_DisplayDirection(Direction_H) £¬¼´ÉèÖÃÆÁÄ»ºáÆÁÏÔÊ¾
*
*****************************************************************************************************************************************/

void LCD_SetDirection(uint8_t direction)
{
	LCD.Direction = direction;    // Ð´ÈëÈ«¾ÖLCD²ÎÊý

   if( direction == Direction_H )   // ºáÆÁÏÔÊ¾
   {
      LCD_WriteCommand(0x36);    		// ÏÔ´æ·ÃÎÊ¿ØÖÆ Ö¸Áî£¬ÓÃÓÚÉèÖÃ·ÃÎÊÏÔ´æµÄ·½Ê½
      LCD_WriteData_8bit(0x70);        // ºáÆÁÏÔÊ¾
      LCD.X_Offset   = 20;             // ÉèÖÃ¿ØÖÆÆ÷×ø±êÆ«ÒÆÁ¿
      LCD.Y_Offset   = 0;
      LCD.Width      = LCD_Height;		// ÖØÐÂ¸³Öµ³¤¡¢¿í
      LCD.Height     = LCD_Width;
   }
   else if( direction == Direction_V )
   {
      LCD_WriteCommand(0x36);    		// ÏÔ´æ·ÃÎÊ¿ØÖÆ Ö¸Áî£¬ÓÃÓÚÉèÖÃ·ÃÎÊÏÔ´æµÄ·½Ê½
      LCD_WriteData_8bit(0x00);        // ´¹Ö±ÏÔÊ¾
      LCD.X_Offset   = 0;              // ÉèÖÃ¿ØÖÆÆ÷×ø±êÆ«ÒÆÁ¿
      LCD.Y_Offset   = 20;
      LCD.Width      = LCD_Width;		// ÖØÐÂ¸³Öµ³¤¡¢¿í
      LCD.Height     = LCD_Height;
   }
   else if( direction == Direction_H_Flip )
   {
      LCD_WriteCommand(0x36);   			 // ÏÔ´æ·ÃÎÊ¿ØÖÆ Ö¸Áî£¬ÓÃÓÚÉèÖÃ·ÃÎÊÏÔ´æµÄ·½Ê½
      LCD_WriteData_8bit(0xA0);         // ºáÆÁÏÔÊ¾£¬²¢ÉÏÏÂ·­×ª£¬RGBÏñËØ¸ñÊ½
      LCD.X_Offset   = 20;              // ÉèÖÃ¿ØÖÆÆ÷×ø±êÆ«ÒÆÁ¿
      LCD.Y_Offset   = 0;
      LCD.Width      = LCD_Height;		 // ÖØÐÂ¸³Öµ³¤¡¢¿í
      LCD.Height     = LCD_Width;
   }
   else if( direction == Direction_V_Flip )
   {
      LCD_WriteCommand(0x36);    		// ÏÔ´æ·ÃÎÊ¿ØÖÆ Ö¸Áî£¬ÓÃÓÚÉèÖÃ·ÃÎÊÏÔ´æµÄ·½Ê½
      LCD_WriteData_8bit(0xC0);        // ´¹Ö±ÏÔÊ¾ £¬²¢ÉÏÏÂ·­×ª£¬RGBÏñËØ¸ñÊ½
      LCD.X_Offset   = 0;              // ÉèÖÃ¿ØÖÆÆ÷×ø±êÆ«ÒÆÁ¿
      LCD.Y_Offset   = 20;
      LCD.Width      = LCD_Width;		// ÖØÐÂ¸³Öµ³¤¡¢¿í
      LCD.Height     = LCD_Height;
   }
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_SetAsciiFont
*
*	Èë¿Ú²ÎÊý:	*fonts - ÒªÉèÖÃµÄASCII×ÖÌå
*
*	º¯Êý¹¦ÄÜ:	ÉèÖÃASCII×ÖÌå£¬¿ÉÑ¡ÔñÊ¹ÓÃ 3216/2412/2010/1608/1206 ÎåÖÖ´óÐ¡µÄ×ÖÌå
*
*	Ëµ    Ã÷:	1. Ê¹ÓÃÊ¾Àý LCD_SetAsciiFont(&ASCII_Font24) £¬¼´ÉèÖÃ 2412µÄ ASCII×ÖÌå
*					2. Ïà¹Ø×ÖÄ£´æ·ÅÔÚ lcd_fonts.c
*
*****************************************************************************************************************************************/

void LCD_SetAsciiFont(pFONT *Asciifonts)
{
  LCD_AsciiFonts = Asciifonts;
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_Clear
*
*	º¯Êý¹¦ÄÜ:	ÇåÆÁº¯Êý£¬½«LCDÇå³ýÎª LCD.BackColor µÄÑÕÉ«
*
*	Ëµ    Ã÷:	ÏÈÓÃ LCD_SetBackColor() ÉèÖÃÒªÇå³ýµÄ±³¾°É«£¬ÔÙµ÷ÓÃ¸Ãº¯ÊýÇåÆÁ¼´¿É
*
*****************************************************************************************************************************************/

void LCD_Clear(void)
{
   LCD_SetAddress(0,0,LCD.Width-1,LCD.Height-1);	// ÉèÖÃ×ø±ê

	LCD_DC_Data;     // Êý¾ÝÖ¸ÁîÑ¡Ôñ Òý½ÅÊä³ö¸ßµçÆ½£¬´ú±í±¾´Î´«Êä Êý¾Ý

// ÐÞ¸ÄÎª16Î»Êý¾Ý¿í¶È£¬Ð´ÈëÊý¾Ý¸ü¼ÓÐ§ÂÊ£¬²»ÐèÒª²ð·Ö
   LCD_SPI.Init.DataSize 	= SPI_DATASIZE_16BIT;   //	16Î»Êý¾Ý¿í¶È
   HAL_SPI_Init(&LCD_SPI);

   LCD_SPI_Transmit(&LCD_SPI, LCD.BackColor, LCD.Width * LCD.Height) ;   // Æô¶¯´«Êä

// ¸Ä»Ø8Î»Êý¾Ý¿í¶È£¬ÒòÎªÖ¸ÁîºÍ²¿·ÖÊý¾Ý¶¼ÊÇ°´ÕÕ8Î»´«ÊäµÄ
	LCD_SPI.Init.DataSize 	= SPI_DATASIZE_8BIT;    //	8Î»Êý¾Ý¿í¶È
   HAL_SPI_Init(&LCD_SPI);
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_ClearRect
*
*	Èë¿Ú²ÎÊý:	x - ÆðÊ¼Ë®Æ½×ø±ê
*					y - ÆðÊ¼´¹Ö±×ø±ê
*					width  - ÒªÇå³ýÇøÓòµÄºáÏò³¤¶È
*					height - ÒªÇå³ýÇøÓòµÄ×ÝÏò¿í¶È
*
*	º¯Êý¹¦ÄÜ:	¾Ö²¿ÇåÆÁº¯Êý£¬½«Ö¸¶¨Î»ÖÃ¶ÔÓ¦µÄÇøÓòÇå³ýÎª LCD.BackColor µÄÑÕÉ«
*
*	Ëµ    Ã÷:	1. ÏÈÓÃ LCD_SetBackColor() ÉèÖÃÒªÇå³ýµÄ±³¾°É«£¬ÔÙµ÷ÓÃ¸Ãº¯ÊýÇåÆÁ¼´¿É
*				   2. Ê¹ÓÃÊ¾Àý LCD_ClearRect( 10, 10, 100, 50) £¬Çå³ý×ø±ê(10,10)¿ªÊ¼µÄ³¤100¿í50µÄÇøÓò
*
*****************************************************************************************************************************************/

void LCD_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
   LCD_SetAddress( x, y, x+width-1, y+height-1);	// ÉèÖÃ×ø±ê

	LCD_DC_Data;     // Êý¾ÝÖ¸ÁîÑ¡Ôñ Òý½ÅÊä³ö¸ßµçÆ½£¬´ú±í±¾´Î´«Êä Êý¾Ý

// ÐÞ¸ÄÎª16Î»Êý¾Ý¿í¶È£¬Ð´ÈëÊý¾Ý¸ü¼ÓÐ§ÂÊ£¬²»ÐèÒª²ð·Ö
   LCD_SPI.Init.DataSize 	= SPI_DATASIZE_16BIT;   //	16Î»Êý¾Ý¿í¶È
   HAL_SPI_Init(&LCD_SPI);

   LCD_SPI_Transmit(&LCD_SPI, LCD.BackColor, width*height) ;  // Æô¶¯´«Êä

// ¸Ä»Ø8Î»Êý¾Ý¿í¶È£¬ÒòÎªÖ¸ÁîºÍ²¿·ÖÊý¾Ý¶¼ÊÇ°´ÕÕ8Î»´«ÊäµÄ
	LCD_SPI.Init.DataSize 	= SPI_DATASIZE_8BIT;    //	8Î»Êý¾Ý¿í¶È
   HAL_SPI_Init(&LCD_SPI);

}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_DrawPoint
*
*	Èë¿Ú²ÎÊý:	x - ÆðÊ¼Ë®Æ½×ø±ê
*					y - ÆðÊ¼´¹Ö±×ø±ê
*					color  - Òª»æÖÆµÄÑÕÉ«£¬Ê¹ÓÃ 24Î» RGB888 µÄÑÕÉ«¸ñÊ½£¬ÓÃ»§ÎÞÐè¹ØÐÄÑÕÉ«¸ñÊ½µÄ×ª»»
*
*	º¯Êý¹¦ÄÜ:	ÔÚÖ¸¶¨×ø±ê»æÖÆÖ¸¶¨ÑÕÉ«µÄµã
*
*	Ëµ    Ã÷:	Ê¹ÓÃÊ¾Àý LCD_DrawPoint( 10, 10, 0x0000FF) £¬ÔÚ×ø±ê(10,10)»æÖÆÀ¶É«µÄµã
*
*****************************************************************************************************************************************/

void LCD_DrawPoint(uint16_t x,uint16_t y,uint32_t color)
{
	LCD_SetAddress(x,y,x,y);	//	ÉèÖÃ×ø±ê

	LCD_WriteData_16bit(color)	;
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_DisplayChar
*
*	Èë¿Ú²ÎÊý:	x - ÆðÊ¼Ë®Æ½×ø±ê
*					y - ÆðÊ¼´¹Ö±×ø±ê
*					c  - ASCII×Ö·û
*
*	º¯Êý¹¦ÄÜ:	ÔÚÖ¸¶¨×ø±êÏÔÊ¾Ö¸¶¨µÄ×Ö·û
*
*	Ëµ    Ã÷:	1. ¿ÉÉèÖÃÒªÏÔÊ¾µÄ×ÖÌå£¬ÀýÈçÊ¹ÓÃ LCD_SetAsciiFont(&ASCII_Font24) ÉèÖÃÎª 2412µÄASCII×ÖÌå
*					2.	¿ÉÉèÖÃÒªÏÔÊ¾µÄÑÕÉ«£¬ÀýÈçÊ¹ÓÃ LCD_SetColor(0xff0000FF) ÉèÖÃÎªÀ¶É«
*					3. ¿ÉÉèÖÃ¶ÔÓ¦µÄ±³¾°É«£¬ÀýÈçÊ¹ÓÃ LCD_SetBackColor(0x000000) ÉèÖÃÎªºÚÉ«µÄ±³¾°É«
*					4. Ê¹ÓÃÊ¾Àý LCD_DisplayChar( 10, 10, 'a') £¬ÔÚ×ø±ê(10,10)ÏÔÊ¾×Ö·û 'a'
*
*****************************************************************************************************************************************/

void LCD_DisplayChar(uint16_t x, uint16_t y,uint8_t c)
{
	uint16_t  index = 0, counter = 0 ,i = 0, w = 0;		// ¼ÆÊý±äÁ¿
   uint8_t   disChar;		//´æ´¢×Ö·ûµÄµØÖ·

	c = c - 32; 	// ¼ÆËãASCII×Ö·ûµÄÆ«ÒÆ

	for(index = 0; index < LCD_AsciiFonts->Sizes; index++)
	{
		disChar = LCD_AsciiFonts->pTable[c*LCD_AsciiFonts->Sizes + index]; //»ñÈ¡×Ö·ûµÄÄ£Öµ
		for(counter = 0; counter < 8; counter++)
		{
			if(disChar & 0x01)
			{
            LCD_Buff[i] =  LCD.Color;			// µ±Ç°Ä£Öµ²»Îª0Ê±£¬Ê¹ÓÃ»­±ÊÉ«»æµã
			}
			else
			{
            LCD_Buff[i] = LCD.BackColor;		//·ñÔòÊ¹ÓÃ±³¾°É«»æÖÆµã
			}
			disChar >>= 1;
			i++;
         w++;
 			if( w == LCD_AsciiFonts->Width ) // Èç¹ûÐ´ÈëµÄÊý¾Ý´ïµ½ÁË×Ö·û¿í¶È£¬ÔòÍË³öµ±Ç°Ñ­»·
			{								   // ½øÈëÏÂÒ»×Ö·ûµÄÐ´ÈëµÄ»æÖÆ
				w = 0;
				break;
			}
		}
	}
   LCD_SetAddress( x, y, x+LCD_AsciiFonts->Width-1, y+LCD_AsciiFonts->Height-1);	   // ÉèÖÃ×ø±ê
   LCD_WriteBuff(LCD_Buff,LCD_AsciiFonts->Width*LCD_AsciiFonts->Height);          // Ð´ÈëÏÔ´æ
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_DisplayString
*
*	Èë¿Ú²ÎÊý:	x - ÆðÊ¼Ë®Æ½×ø±ê
*					y - ÆðÊ¼´¹Ö±×ø±ê
*					p - ASCII×Ö·û´®µÄÊ×µØÖ·
*
*	º¯Êý¹¦ÄÜ:	ÔÚÖ¸¶¨×ø±êÏÔÊ¾Ö¸¶¨µÄ×Ö·û´®
*
*	Ëµ    Ã÷:	1. ¿ÉÉèÖÃÒªÏÔÊ¾µÄ×ÖÌå£¬ÀýÈçÊ¹ÓÃ LCD_SetAsciiFont(&ASCII_Font24) ÉèÖÃÎª 2412µÄASCII×ÖÌå
*					2.	¿ÉÉèÖÃÒªÏÔÊ¾µÄÑÕÉ«£¬ÀýÈçÊ¹ÓÃ LCD_SetColor(0x0000FF) ÉèÖÃÎªÀ¶É«
*					3. ¿ÉÉèÖÃ¶ÔÓ¦µÄ±³¾°É«£¬ÀýÈçÊ¹ÓÃ LCD_SetBackColor(0x000000) ÉèÖÃÎªºÚÉ«µÄ±³¾°É«
*					4. Ê¹ÓÃÊ¾Àý LCD_DisplayString( 10, 10, "FANKE") £¬ÔÚÆðÊ¼×ø±êÎª(10,10)µÄµØ·½ÏÔÊ¾×Ö·û´®"FANKE"
*
*****************************************************************************************************************************************/

void LCD_DisplayString( uint16_t x, uint16_t y, char *p)
{
	while ((x < LCD.Width) && (*p != 0))	//ÅÐ¶ÏÏÔÊ¾×ø±êÊÇ·ñ³¬³öÏÔÊ¾ÇøÓò²¢ÇÒ×Ö·ûÊÇ·ñÎª¿Õ×Ö·û
	{
		 LCD_DisplayChar( x,y,*p);
		 x += LCD_AsciiFonts->Width; //ÏÔÊ¾ÏÂÒ»¸ö×Ö·û
		 p++;	//È¡ÏÂÒ»¸ö×Ö·ûµØÖ·
	}
}

/****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_SetTextFont
*
*	Èë¿Ú²ÎÊý:	*fonts - ÒªÉèÖÃµÄÎÄ±¾×ÖÌå
*
*	º¯Êý¹¦ÄÜ:	ÉèÖÃÎÄ±¾×ÖÌå£¬°üÀ¨ÖÐÎÄºÍASCII×Ö·û£¬
*
*	Ëµ    Ã÷:	1. ¿ÉÑ¡ÔñÊ¹ÓÃ 3232/2424/2020/1616/1212 ÎåÖÖ´óÐ¡µÄÖÐÎÄ×ÖÌå£¬
*						²¢ÇÒ¶ÔÓ¦µÄÉèÖÃASCII×ÖÌåÎª 3216/2412/2010/1608/1206
*					2. Ïà¹Ø×ÖÄ£´æ·ÅÔÚ lcd_fonts.c
*					3. ÖÐÎÄ×Ö¿âÊ¹ÓÃµÄÊÇÐ¡×Ö¿â£¬¼´ÓÃµ½ÁË¶ÔÓ¦µÄºº×ÖÔÙÈ¥È¡Ä£
*					4. Ê¹ÓÃÊ¾Àý LCD_SetTextFont(&CH_Font24) £¬¼´ÉèÖÃ 2424µÄÖÐÎÄ×ÖÌåÒÔ¼°2412µÄASCII×Ö·û×ÖÌå
*
*****************************************************************************************************************************************/

void LCD_SetTextFont(pFONT *fonts)
{
	LCD_CHFonts = fonts;		// ÉèÖÃÖÐÎÄ×ÖÌå
	switch(fonts->Width )
	{
		case 12:	LCD_AsciiFonts = &ASCII_Font12;	break;	// ÉèÖÃASCII×Ö·ûµÄ×ÖÌåÎª 1206
		// case 16:	LCD_AsciiFonts = &ASCII_Font16;	break;	// ÉèÖÃASCII×Ö·ûµÄ×ÖÌåÎª 1608
		// case 20:	LCD_AsciiFonts = &ASCII_Font20;	break;	// ÉèÖÃASCII×Ö·ûµÄ×ÖÌåÎª 2010
		// case 24:	LCD_AsciiFonts = &ASCII_Font24;	break;	// ÉèÖÃASCII×Ö·ûµÄ×ÖÌåÎª 2412
		// case 32:	LCD_AsciiFonts = &ASCII_Font32;	break;	// ÉèÖÃASCII×Ö·ûµÄ×ÖÌåÎª 3216
		default: break;
	}
}
/******************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_DisplayChinese
*
*	Èë¿Ú²ÎÊý:	x - ÆðÊ¼Ë®Æ½×ø±ê
*					y - ÆðÊ¼´¹Ö±×ø±ê
*					pText - ÖÐÎÄ×Ö·û
*
*	º¯Êý¹¦ÄÜ:	ÔÚÖ¸¶¨×ø±êÏÔÊ¾Ö¸¶¨µÄµ¥¸öÖÐÎÄ×Ö·û
*
*	Ëµ    Ã÷:	1. ¿ÉÉèÖÃÒªÏÔÊ¾µÄ×ÖÌå£¬ÀýÈçÊ¹ÓÃ LCD_SetTextFont(&CH_Font24) ÉèÖÃÎª 2424µÄÖÐÎÄ×ÖÌåÒÔ¼°2412µÄASCII×Ö·û×ÖÌå
*					2.	¿ÉÉèÖÃÒªÏÔÊ¾µÄÑÕÉ«£¬ÀýÈçÊ¹ÓÃ LCD_SetColor(0xff0000FF) ÉèÖÃÎªÀ¶É«
*					3. ¿ÉÉèÖÃ¶ÔÓ¦µÄ±³¾°É«£¬ÀýÈçÊ¹ÓÃ LCD_SetBackColor(0xff000000) ÉèÖÃÎªºÚÉ«µÄ±³¾°É«
*					4. Ê¹ÓÃÊ¾Àý LCD_DisplayChinese( 10, 10, "·´") £¬ÔÚ×ø±ê(10,10)ÏÔÊ¾ÖÐÎÄ×Ö·û"·´"
*
*****************************************************************************************************************************************/

void LCD_DisplayChinese(uint16_t x, uint16_t y, char *pText)
{
	uint16_t  i=0,index = 0, counter = 0;	// ¼ÆÊý±äÁ¿
	uint16_t  addr;	// ×ÖÄ£µØÖ·
   uint8_t   disChar;	//×ÖÄ£µÄÖµ
	uint16_t  Xaddress = 0; //Ë®Æ½×ø±ê

	while(1)
	{
		// ¶Ô±ÈÊý×éÖÐµÄºº×Ö±àÂë£¬ÓÃÒÔ¶¨Î»¸Ãºº×Ö×ÖÄ£µÄµØÖ·
		if ( *(LCD_CHFonts->pTable + (i+1)*LCD_CHFonts->Sizes + 0)==*pText && *(LCD_CHFonts->pTable + (i+1)*LCD_CHFonts->Sizes + 1)==*(pText+1) )
		{
			addr=i;	// ×ÖÄ£µØÖ·Æ«ÒÆ
			break;
		}
		i+=2;	// Ã¿¸öÖÐÎÄ×Ö·û±àÂëÕ¼Á½×Ö½Ú

		if(i >= LCD_CHFonts->Table_Rows)	break;	// ×ÖÄ£ÁÐ±íÖÐÎÞÏàÓ¦µÄºº×Ö
	}
	i=0;
	for(index = 0; index <LCD_CHFonts->Sizes; index++)
	{
		disChar = *(LCD_CHFonts->pTable + (addr)*LCD_CHFonts->Sizes + index);	// »ñÈ¡ÏàÓ¦µÄ×ÖÄ£µØÖ·

		for(counter = 0; counter < 8; counter++)
		{
			if(disChar & 0x01)
			{
            LCD_Buff[i] =  LCD.Color;			// µ±Ç°Ä£Öµ²»Îª0Ê±£¬Ê¹ÓÃ»­±ÊÉ«»æµã
			}
			else
			{
            LCD_Buff[i] = LCD.BackColor;		// ·ñÔòÊ¹ÓÃ±³¾°É«»æÖÆµã
			}
         i++;
			disChar >>= 1;
			Xaddress++;  //Ë®Æ½×ø±ê×Ô¼Ó

			if( Xaddress == LCD_CHFonts->Width ) 	//	Èç¹ûË®Æ½×ø±ê´ïµ½ÁË×Ö·û¿í¶È£¬ÔòÍË³öµ±Ç°Ñ­»·
			{														//	½øÈëÏÂÒ»ÐÐµÄ»æÖÆ
				Xaddress = 0;
				break;
			}
		}
	}
   LCD_SetAddress( x, y, x+LCD_CHFonts->Width-1, y+LCD_CHFonts->Height-1);	   // ÉèÖÃ×ø±ê
   LCD_WriteBuff(LCD_Buff,LCD_CHFonts->Width*LCD_CHFonts->Height);            // Ð´ÈëÏÔ´æ
}

/*****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_DisplayText
*
*	Èë¿Ú²ÎÊý:	x - ÆðÊ¼Ë®Æ½×ø±ê
*					y - ÆðÊ¼´¹Ö±×ø±ê
*					pText - ×Ö·û´®£¬¿ÉÒÔÏÔÊ¾ÖÐÎÄ»òÕßASCII×Ö·û
*
*	º¯Êý¹¦ÄÜ:	ÔÚÖ¸¶¨×ø±êÏÔÊ¾Ö¸¶¨µÄ×Ö·û´®
*
*	Ëµ    Ã÷:	1. ¿ÉÉèÖÃÒªÏÔÊ¾µÄ×ÖÌå£¬ÀýÈçÊ¹ÓÃ LCD_SetTextFont(&CH_Font24) ÉèÖÃÎª 2424µÄÖÐÎÄ×ÖÌåÒÔ¼°2412µÄASCII×Ö·û×ÖÌå
*					2.	¿ÉÉèÖÃÒªÏÔÊ¾µÄÑÕÉ«£¬ÀýÈçÊ¹ÓÃ LCD_SetColor(0xff0000FF) ÉèÖÃÎªÀ¶É«
*					3. ¿ÉÉèÖÃ¶ÔÓ¦µÄ±³¾°É«£¬ÀýÈçÊ¹ÓÃ LCD_SetBackColor(0xff000000) ÉèÖÃÎªºÚÉ«µÄ±³¾°É«
*					4. Ê¹ÓÃÊ¾Àý LCD_DisplayChinese( 10, 10, "·´¿Í¿Æ¼¼STM32") £¬ÔÚ×ø±ê(10,10)ÏÔÊ¾×Ö·û´®"·´¿Í¿Æ¼¼STM32"
*
**********************************************************************************************************************************fanke*******/

void LCD_DisplayText(uint16_t x, uint16_t y, char *pText)
{

	while(*pText != 0)	// ÅÐ¶ÏÊÇ·ñÎª¿Õ×Ö·û
	{
		if(*pText<=0x7F)	// ÅÐ¶ÏÊÇ·ñÎªASCIIÂë
		{
			LCD_DisplayChar(x,y,*pText);	// ÏÔÊ¾ASCII
			x+=LCD_AsciiFonts->Width;				// Ë®Æ½×ø±êµ÷µ½ÏÂÒ»¸ö×Ö·û´¦
			pText++;								// ×Ö·û´®µØÖ·+1
		}
		else					// Èô×Ö·ûÎªºº×Ö
		{
			LCD_DisplayChinese(x,y,pText);	// ÏÔÊ¾ºº×Ö
			x+=LCD_CHFonts->Width;				// Ë®Æ½×ø±êµ÷µ½ÏÂÒ»¸ö×Ö·û´¦
			pText+=2;								// ×Ö·û´®µØÖ·+2£¬ºº×ÖµÄ±àÂëÒª2×Ö½Ú
		}
	}
}

/*****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_ShowNumMode
*
*	Èë¿Ú²ÎÊý:	mode - ÉèÖÃ±äÁ¿µÄÏÔÊ¾Ä£Ê½
*
*	º¯Êý¹¦ÄÜ:	ÉèÖÃ±äÁ¿ÏÔÊ¾Ê±¶àÓàÎ»²¹0»¹ÊÇ²¹¿Õ¸ñ£¬¿ÉÊäÈë²ÎÊý Fill_Space Ìî³ä¿Õ¸ñ£¬Fill_Zero Ìî³äÁã
*
*	Ëµ    Ã÷:   1. Ö»ÓÐ LCD_DisplayNumber() ÏÔÊ¾ÕûÊý ºÍ LCD_DisplayDecimals()ÏÔÊ¾Ð¡Êý ÕâÁ½¸öº¯ÊýÓÃµ½
*					2. Ê¹ÓÃÊ¾Àý LCD_ShowNumMode(Fill_Zero) ÉèÖÃ¶àÓàÎ»Ìî³ä0£¬ÀýÈç 123 ¿ÉÒÔÏÔÊ¾Îª 000123
*
*****************************************************************************************************************************************/

void LCD_ShowNumMode(uint8_t mode)
{
	LCD.ShowNum_Mode = mode;
}

/*****************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_DisplayNumber
*
*	Èë¿Ú²ÎÊý:	x - ÆðÊ¼Ë®Æ½×ø±ê
*					y - ÆðÊ¼´¹Ö±×ø±ê
*					number - ÒªÏÔÊ¾µÄÊý×Ö,·¶Î§ÔÚ -2147483648~2147483647 Ö®¼ä
*					len - Êý×ÖµÄÎ»Êý£¬Èç¹ûÎ»Êý³¬¹ýlen£¬½«°´ÆäÊµ¼Ê³¤¶ÈÊä³ö£¬Èç¹ûÐèÒªÏÔÊ¾¸ºÊý£¬ÇëÔ¤ÁôÒ»¸öÎ»µÄ·ûºÅÏÔÊ¾¿Õ¼ä
*
*	º¯Êý¹¦ÄÜ:	ÔÚÖ¸¶¨×ø±êÏÔÊ¾Ö¸¶¨µÄÕûÊý±äÁ¿
*
*	Ëµ    Ã÷:	1. ¿ÉÉèÖÃÒªÏÔÊ¾µÄ×ÖÌå£¬ÀýÈçÊ¹ÓÃ LCD_SetAsciiFont(&ASCII_Font24) ÉèÖÃÎªµÄASCII×Ö·û×ÖÌå
*					2.	¿ÉÉèÖÃÒªÏÔÊ¾µÄÑÕÉ«£¬ÀýÈçÊ¹ÓÃ LCD_SetColor(0x0000FF) ÉèÖÃÎªÀ¶É«
*					3. ¿ÉÉèÖÃ¶ÔÓ¦µÄ±³¾°É«£¬ÀýÈçÊ¹ÓÃ LCD_SetBackColor(0x000000) ÉèÖÃÎªºÚÉ«µÄ±³¾°É«
*					4. Ê¹ÓÃÊ¾Àý LCD_DisplayNumber( 10, 10, a, 5) £¬ÔÚ×ø±ê(10,10)ÏÔÊ¾Ö¸¶¨±äÁ¿a,×Ü¹²5Î»£¬¶àÓàÎ»²¹0»ò¿Õ¸ñ£¬
*						ÀýÈç a=123 Ê±£¬»á¸ù¾Ý LCD_ShowNumMode()µÄÉèÖÃÀ´ÏÔÊ¾  123(Ç°ÃæÁ½¸ö¿Õ¸ñÎ») »òÕß00123
*
*****************************************************************************************************************************************/

void  LCD_DisplayNumber( uint16_t x, uint16_t y, int32_t number, uint8_t len)
{
	char   Number_Buffer[15];				// ÓÃÓÚ´æ´¢×ª»»ºóµÄ×Ö·û´®

	if( LCD.ShowNum_Mode == Fill_Zero)	// ¶àÓàÎ»²¹0
	{
		sprintf( Number_Buffer , "%0.*d",len, number );	// ½« number ×ª»»³É×Ö·û´®£¬±ãÓÚÏÔÊ¾
	}
	else			// ¶àÓàÎ»²¹¿Õ¸ñ
	{
		sprintf( Number_Buffer , "%*d",len, number );	// ½« number ×ª»»³É×Ö·û´®£¬±ãÓÚÏÔÊ¾
	}

	LCD_DisplayString( x, y,(char *)Number_Buffer) ;  // ½«×ª»»µÃµ½µÄ×Ö·û´®ÏÔÊ¾³öÀ´

}

/***************************************************************************************************************************************
*	º¯ Êý Ãû:	LCD_DisplayDecimals
*
*	Èë¿Ú²ÎÊý:	x - ÆðÊ¼Ë®Æ½×ø±ê
*					y - ÆðÊ¼´¹Ö±×ø±ê
*					decimals - ÒªÏÔÊ¾µÄÊý×Ö, doubleÐÍÈ¡Öµ1.7 x 10^£¨-308£©~ 1.7 x 10^£¨+308£©£¬µ«ÊÇÄÜÈ·±£×¼È·µÄÓÐÐ§Î»ÊýÎª15~16Î»
*
*       			len - Õû¸ö±äÁ¿µÄ×ÜÎ»Êý£¨°üÀ¨Ð¡ÊýµãºÍ¸ººÅ£©£¬ÈôÊµ¼ÊµÄ×ÜÎ»Êý³¬¹ýÁËÖ¸¶¨µÄ×ÜÎ»Êý£¬½«°´Êµ¼ÊµÄ×Ü³¤¶ÈÎ»Êä³ö£¬
*							Ê¾Àý1£ºÐ¡Êý -123.123 £¬Ö¸¶¨ len <=8 µÄ»°£¬ÔòÊµ¼ÊÕÕ³£Êä³ö -123.123
*							Ê¾Àý2£ºÐ¡Êý -123.123 £¬Ö¸¶¨ len =10 µÄ»°£¬ÔòÊµ¼ÊÊä³ö   -123.123(¸ººÅÇ°Ãæ»áÓÐÁ½¸ö¿Õ¸ñÎ»)
*							Ê¾Àý3£ºÐ¡Êý -123.123 £¬Ö¸¶¨ len =10 µÄ»°£¬µ±µ÷ÓÃº¯Êý LCD_ShowNumMode() ÉèÖÃÎªÌî³ä0Ä£Ê½Ê±£¬Êµ¼ÊÊä³ö -00123.123
*
*					decs - Òª±£ÁôµÄÐ¡ÊýÎ»Êý£¬ÈôÐ¡ÊýµÄÊµ¼ÊÎ»Êý³¬¹ýÁËÖ¸¶¨µÄÐ¡ÊýÎ»£¬Ôò°´Ö¸¶¨µÄ¿í¶ÈËÄÉáÎåÈëÊä³ö
*							 Ê¾Àý£º1.12345 £¬Ö¸¶¨ decs Îª4Î»µÄ»°£¬ÔòÊä³ö½á¹ûÎª1.1235
*
*	º¯Êý¹¦ÄÜ:	ÔÚÖ¸¶¨×ø±êÏÔÊ¾Ö¸¶¨µÄ±äÁ¿£¬°üÀ¨Ð¡Êý
*
*	Ëµ    Ã÷:	1. ¿ÉÉèÖÃÒªÏÔÊ¾µÄ×ÖÌå£¬ÀýÈçÊ¹ÓÃ LCD_SetAsciiFont(&ASCII_Font24) ÉèÖÃÎªµÄASCII×Ö·û×ÖÌå
*					2.	¿ÉÉèÖÃÒªÏÔÊ¾µÄÑÕÉ«£¬ÀýÈçÊ¹ÓÃ LCD_SetColor(0x0000FF) ÉèÖÃÎªÀ¶É«
*					3. ¿ÉÉèÖÃ¶ÔÓ¦µÄ±³¾°É«£¬ÀýÈçÊ¹ÓÃ LCD_SetBackColor(0x000000) ÉèÖÃÎªºÚÉ«µÄ±³¾°É«
*					4. Ê¹ÓÃÊ¾Àý LCD_DisplayDecimals( 10, 10, a, 5, 3) £¬ÔÚ×ø±ê(10,10)ÏÔÊ¾×Ö±äÁ¿a,×Ü³¤¶ÈÎª5Î»£¬ÆäÖÐ±£Áô3Î»Ð¡Êý
*
*****************************************************************************************************************************************/

void  LCD_DisplayDecimals( uint16_t x, uint16_t y, double decimals, uint8_t len, uint8_t decs)
{
	char  Number_Buffer[20];				// ÓÃÓÚ´æ´¢×ª»»ºóµÄ×Ö·û´®

	if( LCD.ShowNum_Mode == Fill_Zero)	// ¶àÓàÎ»Ìî³ä0Ä£Ê½
	{
		sprintf( Number_Buffer , "%0*.*lf",len,decs, decimals );	// ½« number ×ª»»³É×Ö·û´®£¬±ãÓÚÏÔÊ¾
	}
	else		// ¶àÓàÎ»Ìî³ä¿Õ¸ñ
	{
		sprintf( Number_Buffer , "%*.*lf",len,decs, decimals );	// ½« number ×ª»»³É×Ö·û´®£¬±ãÓÚÏÔÊ¾
	}

	LCD_DisplayString( x, y,(char *)Number_Buffer) ;	// ½«×ª»»µÃµ½µÄ×Ö·û´®ÏÔÊ¾³öÀ´
}


/***************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_DrawLine
*
*	Èë¿Ú²ÎÊý: x1 - Æðµã Ë®Æ½×ø±ê
*			 	 y1 - Æðµã ´¹Ö±×ø±ê
*
*				 x2 - ÖÕµã Ë®Æ½×ø±ê
*            y2 - ÖÕµã ´¹Ö±×ø±ê
*
*	º¯Êý¹¦ÄÜ: ÔÚÁ½µãÖ®¼ä»­Ïß
*
*	Ëµ    Ã÷: ¸Ãº¯ÊýÒÆÖ²ÓÚST¹Ù·½ÆÀ¹À°åµÄÀý³Ì
*
*****************************************************************************************************************************************/

#define ABS(X)  ((X) > 0 ? (X) : -(X))

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	curpixel = 0;

	deltax = ABS(x2 - x1);        /* The difference between the x's */
	deltay = ABS(y2 - y1);        /* The difference between the y's */
	x = x1;                       /* Start x off at the first pixel */
	y = y1;                       /* Start y off at the first pixel */

	if (x2 >= x1)                 /* The x-values are increasing */
	{
	 xinc1 = 1;
	 xinc2 = 1;
	}
	else                          /* The x-values are decreasing */
	{
	 xinc1 = -1;
	 xinc2 = -1;
	}

	if (y2 >= y1)                 /* The y-values are increasing */
	{
	 yinc1 = 1;
	 yinc2 = 1;
	}
	else                          /* The y-values are decreasing */
	{
	 yinc1 = -1;
	 yinc2 = -1;
	}

	if (deltax >= deltay)         /* There is at least one x-value for every y-value */
	{
	 xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
	 yinc2 = 0;                  /* Don't change the y for every iteration */
	 den = deltax;
	 num = deltax / 2;
	 numadd = deltay;
	 numpixels = deltax;         /* There are more x-values than y-values */
	}
	else                          /* There is at least one y-value for every x-value */
	{
	 xinc2 = 0;                  /* Don't change the x for every iteration */
	 yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
	 den = deltay;
	 num = deltay / 2;
	 numadd = deltax;
	 numpixels = deltay;         /* There are more y-values than x-values */
	}
	for (curpixel = 0; curpixel <= numpixels; curpixel++)
	{
	 LCD_DrawPoint(x,y,LCD.Color);             /* Draw the current pixel */
	 num += numadd;              /* Increase the numerator by the top of the fraction */
	 if (num >= den)             /* Check if numerator >= denominator */
	 {
		num -= den;               /* Calculate the new numerator value */
		x += xinc1;               /* Change the x as appropriate */
		y += yinc1;               /* Change the y as appropriate */
	 }
	 x += xinc2;                 /* Change the x as appropriate */
	 y += yinc2;                 /* Change the y as appropriate */
	}
}

/***************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_DrawLine_V
*
*	Èë¿Ú²ÎÊý: x - Ë®Æ½×ø±ê
*			 	 y - ´¹Ö±×ø±ê
*				 height - ´¹Ö±¿í¶È
*
*	º¯Êý¹¦ÄÜ: ÔÚÖ¸µãÎ»ÖÃ»æÖÆÖ¸¶¨³¤¿íµÄ ´¹Ö± Ïß
*
*	Ëµ    Ã÷: 1. ¸Ãº¯ÊýÒÆÖ²ÓÚST¹Ù·½ÆÀ¹À°åµÄÀý³Ì
*				 2. Òª»æÖÆµÄÇøÓò²»ÄÜ³¬¹ýÆÁÄ»µÄÏÔÊ¾ÇøÓò
*            3. Èç¹ûÖ»ÊÇ»­´¹Ö±µÄÏß£¬ÓÅÏÈÊ¹ÓÃ´Ëº¯Êý£¬ËÙ¶È±È LCD_DrawLine ¿ìºÜ¶à
*  ÐÔÄÜ²âÊÔ£º
*****************************************************************************************************************************************/

void LCD_DrawLine_V(uint16_t x, uint16_t y, uint16_t height)
{
   uint16_t i ; // ¼ÆÊý±äÁ¿

	for (i = 0; i < height; i++)
	{
       LCD_Buff[i] =  LCD.Color;  // Ð´Èë»º³åÇø
   }
   LCD_SetAddress( x, y, x, y+height-1);	     // ÉèÖÃ×ø±ê

   LCD_WriteBuff(LCD_Buff,height);          // Ð´ÈëÏÔ´æ
}

/***************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_DrawLine_H
*
*	Èë¿Ú²ÎÊý: x - Ë®Æ½×ø±ê
*			 	 y - ´¹Ö±×ø±ê
*				 width  - Ë®Æ½¿í¶È
*
*	º¯Êý¹¦ÄÜ: ÔÚÖ¸µãÎ»ÖÃ»æÖÆÖ¸¶¨³¤¿íµÄ Ë®Æ½ Ïß
*
*	Ëµ    Ã÷: 1. ¸Ãº¯ÊýÒÆÖ²ÓÚST¹Ù·½ÆÀ¹À°åµÄÀý³Ì
*				 2. Òª»æÖÆµÄÇøÓò²»ÄÜ³¬¹ýÆÁÄ»µÄÏÔÊ¾ÇøÓò
*            3. Èç¹ûÖ»ÊÇ»­ Ë®Æ½ µÄÏß£¬ÓÅÏÈÊ¹ÓÃ´Ëº¯Êý£¬ËÙ¶È±È LCD_DrawLine ¿ìºÜ¶à
*  ÐÔÄÜ²âÊÔ£º
**********************************************************************************************************************************fanke*******/

void LCD_DrawLine_H(uint16_t x, uint16_t y, uint16_t width)
{
   uint16_t i ; // ¼ÆÊý±äÁ¿

	for (i = 0; i < width; i++)
	{
       LCD_Buff[i] =  LCD.Color;  // Ð´Èë»º³åÇø
   }
   LCD_SetAddress( x, y, x+width-1, y);	     // ÉèÖÃ×ø±ê

   LCD_WriteBuff(LCD_Buff,width);          // Ð´ÈëÏÔ´æ
}
/***************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_DrawRect
*
*	Èë¿Ú²ÎÊý: x - Ë®Æ½×ø±ê
*			 	 y - ´¹Ö±×ø±ê
*			 	 width  - Ë®Æ½¿í¶È
*				 height - ´¹Ö±¿í¶È
*
*	º¯Êý¹¦ÄÜ: ÔÚÖ¸µãÎ»ÖÃ»æÖÆÖ¸¶¨³¤¿íµÄ¾ØÐÎÏßÌõ
*
*	Ëµ    Ã÷: 1. ¸Ãº¯ÊýÒÆÖ²ÓÚST¹Ù·½ÆÀ¹À°åµÄÀý³Ì
*				 2. Òª»æÖÆµÄÇøÓò²»ÄÜ³¬¹ýÆÁÄ»µÄÏÔÊ¾ÇøÓò
*
*****************************************************************************************************************************************/

void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
   // »æÖÆË®Æ½Ïß
   LCD_DrawLine_H( x,  y,  width);
   LCD_DrawLine_H( x,  y+height-1,  width);

   // »æÖÆ´¹Ö±Ïß
   LCD_DrawLine_V( x,  y,  height);
   LCD_DrawLine_V( x+width-1,  y,  height);
}


/***************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_DrawCircle
*
*	Èë¿Ú²ÎÊý: x - Ô²ÐÄ Ë®Æ½×ø±ê
*			 	 y - Ô²ÐÄ ´¹Ö±×ø±ê
*			 	 r  - °ë¾¶
*
*	º¯Êý¹¦ÄÜ: ÔÚ×ø±ê (x,y) »æÖÆ°ë¾¶Îª r µÄÔ²ÐÎÏßÌõ
*
*	Ëµ    Ã÷: 1. ¸Ãº¯ÊýÒÆÖ²ÓÚST¹Ù·½ÆÀ¹À°åµÄÀý³Ì
*				 2. Òª»æÖÆµÄÇøÓò²»ÄÜ³¬¹ýÆÁÄ»µÄÏÔÊ¾ÇøÓò
*
*****************************************************************************************************************************************/

void LCD_DrawCircle(uint16_t x, uint16_t y, uint16_t r)
{
	int Xadd = -r, Yadd = 0, err = 2-2*r, e2;
	do {

		LCD_DrawPoint(x-Xadd,y+Yadd,LCD.Color);
		LCD_DrawPoint(x+Xadd,y+Yadd,LCD.Color);
		LCD_DrawPoint(x+Xadd,y-Yadd,LCD.Color);
		LCD_DrawPoint(x-Xadd,y-Yadd,LCD.Color);

		e2 = err;
		if (e2 <= Yadd) {
			err += ++Yadd*2+1;
			if (-Xadd == Yadd && e2 <= Xadd) e2 = 0;
		}
		if (e2 > Xadd) err += ++Xadd*2+1;
    }
    while (Xadd <= 0);
}


/***************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_DrawEllipse
*
*	Èë¿Ú²ÎÊý: x - Ô²ÐÄ Ë®Æ½×ø±ê
*			 	 y - Ô²ÐÄ ´¹Ö±×ø±ê
*			 	 r1  - Ë®Æ½°ëÖáµÄ³¤¶È
*				 r2  - ´¹Ö±°ëÖáµÄ³¤¶È
*
*	º¯Êý¹¦ÄÜ: ÔÚ×ø±ê (x,y) »æÖÆË®Æ½°ëÖáÎª r1 ´¹Ö±°ëÖáÎª r2 µÄÍÖÔ²ÏßÌõ
*
*	Ëµ    Ã÷: 1. ¸Ãº¯ÊýÒÆÖ²ÓÚST¹Ù·½ÆÀ¹À°åµÄÀý³Ì
*				 2. Òª»æÖÆµÄÇøÓò²»ÄÜ³¬¹ýÆÁÄ»µÄÏÔÊ¾ÇøÓò
*
*****************************************************************************************************************************************/

void LCD_DrawEllipse(int x, int y, int r1, int r2)
{
  int Xadd = -r1, Yadd = 0, err = 2-2*r1, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = r1;
  rad2 = r2;

  if (r1 > r2)
  {
    do {
      K = (float)(rad1/rad2);

		LCD_DrawPoint(x-Xadd,y+(uint16_t)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x+Xadd,y+(uint16_t)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x+Xadd,y-(uint16_t)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x-Xadd,y-(uint16_t)(Yadd/K),LCD.Color);

      e2 = err;
      if (e2 <= Yadd) {
        err += ++Yadd*2+1;
        if (-Xadd == Yadd && e2 <= Xadd) e2 = 0;
      }
      if (e2 > Xadd) err += ++Xadd*2+1;
    }
    while (Xadd <= 0);
  }
  else
  {
    Yadd = -r2;
    Xadd = 0;
    do {
      K = (float)(rad2/rad1);

		LCD_DrawPoint(x-(uint16_t)(Xadd/K),y+Yadd,LCD.Color);
		LCD_DrawPoint(x+(uint16_t)(Xadd/K),y+Yadd,LCD.Color);
		LCD_DrawPoint(x+(uint16_t)(Xadd/K),y-Yadd,LCD.Color);
		LCD_DrawPoint(x-(uint16_t)(Xadd/K),y-Yadd,LCD.Color);

      e2 = err;
      if (e2 <= Xadd) {
        err += ++Xadd*3+1;
        if (-Yadd == Xadd && e2 <= Yadd) e2 = 0;
      }
      if (e2 > Yadd) err += ++Yadd*3+1;
    }
    while (Yadd <= 0);
  }
}

/***************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_FillCircle
*
*	Èë¿Ú²ÎÊý: x - Ô²ÐÄ Ë®Æ½×ø±ê
*			 	 y - Ô²ÐÄ ´¹Ö±×ø±ê
*			 	 r  - °ë¾¶
*
*	º¯Êý¹¦ÄÜ: ÔÚ×ø±ê (x,y) Ìî³ä°ë¾¶Îª r µÄÔ²ÐÎÇøÓò
*
*	Ëµ    Ã÷: 1. ¸Ãº¯ÊýÒÆÖ²ÓÚST¹Ù·½ÆÀ¹À°åµÄÀý³Ì
*				 2. Òª»æÖÆµÄÇøÓò²»ÄÜ³¬¹ýÆÁÄ»µÄÏÔÊ¾ÇøÓò
*
*****************************************************************************************************************************************/

void LCD_FillCircle(uint16_t x, uint16_t y, uint16_t r)
{
  int32_t  D;    /* Decision Variable */
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */

  D = 3 - (r << 1);

  CurX = 0;
  CurY = r;

  while (CurX <= CurY)
  {
    if(CurY > 0)
    {
      LCD_DrawLine_V(x - CurX, y - CurY,2*CurY);
      LCD_DrawLine_V(x + CurX, y - CurY,2*CurY);
    }

    if(CurX > 0)
    {
		// LCD_DrawLine(x - CurY, y - CurX,x - CurY,y - CurX + 2*CurX);
		// LCD_DrawLine(x + CurY, y - CurX,x + CurY,y - CurX + 2*CurX);

      LCD_DrawLine_V(x - CurY, y - CurX,2*CurX);
      LCD_DrawLine_V(x + CurY, y - CurX,2*CurX);
    }
    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
  LCD_DrawCircle(x, y, r);
}

/***************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_FillRect
*
*	Èë¿Ú²ÎÊý: x - Ë®Æ½×ø±ê
*			 	 y - ´¹Ö±×ø±ê
*			 	 width  - Ë®Æ½¿í¶È
*				 height -´¹Ö±¿í¶È
*
*	º¯Êý¹¦ÄÜ: ÔÚ×ø±ê (x,y) Ìî³äÖ¸¶¨³¤¿íµÄÊµÐÄ¾ØÐÎ
*
*	Ëµ    Ã÷: Òª»æÖÆµÄÇøÓò²»ÄÜ³¬¹ýÆÁÄ»µÄÏÔÊ¾ÇøÓò
*
*****************************************************************************************************************************************/

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
   LCD_SetAddress( x, y, x+width-1, y+height-1);	// ÉèÖÃ×ø±ê

	LCD_DC_Data;     // Êý¾ÝÖ¸ÁîÑ¡Ôñ Òý½ÅÊä³ö¸ßµçÆ½£¬´ú±í±¾´Î´«Êä Êý¾Ý

// ÐÞ¸ÄÎª16Î»Êý¾Ý¿í¶È£¬Ð´ÈëÊý¾Ý¸ü¼ÓÐ§ÂÊ£¬²»ÐèÒª²ð·Ö
   LCD_SPI.Init.DataSize 	= SPI_DATASIZE_16BIT;   //	16Î»Êý¾Ý¿í¶È
   HAL_SPI_Init(&LCD_SPI);

   LCD_SPI_Transmit(&LCD_SPI, LCD.Color, width*height) ;

// ¸Ä»Ø8Î»Êý¾Ý¿í¶È£¬ÒòÎªÖ¸ÁîºÍ²¿·ÖÊý¾Ý¶¼ÊÇ°´ÕÕ8Î»´«ÊäµÄ
	LCD_SPI.Init.DataSize 	= SPI_DATASIZE_8BIT;    //	8Î»Êý¾Ý¿í¶È
   HAL_SPI_Init(&LCD_SPI);
}


/***************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_DrawImage
*
*	Èë¿Ú²ÎÊý: x - ÆðÊ¼Ë®Æ½×ø±ê
*				 y - ÆðÊ¼´¹Ö±×ø±ê
*			 	 width  - Í¼Æ¬µÄË®Æ½¿í¶È
*				 height - Í¼Æ¬µÄ´¹Ö±¿í¶È
*				*pImage - Í¼Æ¬Êý¾Ý´æ´¢ÇøµÄÊ×µØÖ·
*
*	º¯Êý¹¦ÄÜ: ÔÚÖ¸¶¨×ø±ê´¦ÏÔÊ¾Í¼Æ¬
*
*	Ëµ    Ã÷: 1.ÒªÏÔÊ¾µÄÍ¼Æ¬ÐèÒªÊÂÏÈ½øÐÐÈ¡Ä£¡¢»ñÏ¤Í¼Æ¬µÄ³¤¶ÈºÍ¿í¶È
*            2.Ê¹ÓÃ LCD_SetColor() º¯ÊýÉèÖÃ»­±ÊÉ«£¬LCD_SetBackColor() ÉèÖÃ±³¾°É«
*
*****************************************************************************************************************************************/

void 	LCD_DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage)
{
   uint8_t   disChar;	         // ×ÖÄ£µÄÖµ
	uint16_t  Xaddress = x;       // Ë®Æ½×ø±ê
 	uint16_t  Yaddress = y;       // ´¹Ö±×ø±ê
	uint16_t  i=0,j=0,m=0;        // ¼ÆÊý±äÁ¿
	uint16_t  BuffCount = 0;      // »º³åÇø¼ÆÊý
   uint16_t  Buff_Height = 0;    // »º³åÇøµÄÐÐÊý

// ÒòÎª»º³åÇø´óÐ¡ÓÐÏÞ£¬ÐèÒª·Ö¶à´ÎÐ´Èë
   Buff_Height = (sizeof(LCD_Buff)/2) / height;    // ¼ÆËã»º³åÇøÄÜ¹»Ð´ÈëÍ¼Æ¬µÄ¶àÉÙÐÐ

	for(i = 0; i <height; i++)             // Ñ­»·°´ÐÐÐ´Èë
	{
		for(j = 0; j <(float)width/8; j++)
		{
			disChar = *pImage;

			for(m = 0; m < 8; m++)
			{
				if(disChar & 0x01)
				{
               LCD_Buff[BuffCount] =  LCD.Color;			// µ±Ç°Ä£Öµ²»Îª0Ê±£¬Ê¹ÓÃ»­±ÊÉ«»æµã
				}
				else
				{
				   LCD_Buff[BuffCount] = LCD.BackColor;		//·ñÔòÊ¹ÓÃ±³¾°É«»æÖÆµã
				}
				disChar >>= 1;     // Ä£ÖµÒÆÎ»
				Xaddress++;        // Ë®Æ½×ø±ê×Ô¼Ó
				BuffCount++;       // »º³åÇø¼ÆÊý
				if( (Xaddress - x)==width ) // Èç¹ûË®Æ½×ø±ê´ïµ½ÁË×Ö·û¿í¶È£¬ÔòÍË³öµ±Ç°Ñ­»·,½øÈëÏÂÒ»ÐÐµÄ»æÖÆ
				{
					Xaddress = x;
					break;
				}
			}
			pImage++;
		}
      if( BuffCount == Buff_Height*width  )  // ´ïµ½»º³åÇøËùÄÜÈÝÄÉµÄ×î´óÐÐÊýÊ±
      {
         BuffCount = 0; // »º³åÇø¼ÆÊýÇå0

         LCD_SetAddress( x, Yaddress , x+width-1, Yaddress+Buff_Height-1);	// ÉèÖÃ×ø±ê
         LCD_WriteBuff(LCD_Buff,width*Buff_Height);          // Ð´ÈëÏÔ´æ

         Yaddress = Yaddress+Buff_Height;    // ¼ÆËãÐÐÆ«ÒÆ£¬¿ªÊ¼Ð´ÈëÏÂÒ»²¿·ÖÊý¾Ý
      }
      if( (i+1)== height ) // µ½ÁË×îºóÒ»ÐÐÊ±
      {
         LCD_SetAddress( x, Yaddress , x+width-1,i+y);	   // ÉèÖÃ×ø±ê
         LCD_WriteBuff(LCD_Buff,width*(i+1+y-Yaddress));    // Ð´ÈëÏÔ´æ
      }
	}
}


/***************************************************************************************************************************************
*	º¯ Êý Ãû: LCD_CopyBuffer
*
*	Èë¿Ú²ÎÊý: x - ÆðÊ¼Ë®Æ½×ø±ê
*				 y - ÆðÊ¼´¹Ö±×ø±ê
*			 	 width  - Ä¿±êÇøÓòµÄË®Æ½¿í¶È
*				 height - Ä¿±êÇøÓòµÄ´¹Ö±¿í¶È
*				*pImage - Êý¾Ý´æ´¢ÇøµÄÊ×µØÖ·
*
*	º¯Êý¹¦ÄÜ: ÔÚÖ¸¶¨×ø±ê´¦£¬Ö±½Ó½«Êý¾Ý¸´ÖÆµ½ÆÁÄ»µÄÏÔ´æ
*
*	Ëµ    Ã÷: ÅúÁ¿¸´ÖÆº¯Êý£¬¿ÉÓÃÓÚÒÆÖ² LVGL »òÕß½«ÉãÏñÍ·²É¼¯µÄÍ¼ÏñÏÔÊ¾³öÀ´
*
*****************************************************************************************************************************************/

void	LCD_CopyBuffer(uint16_t x, uint16_t y,uint16_t width,uint16_t height,uint16_t *DataBuff)
{

	LCD_SetAddress(x,y,x+width-1,y+height-1);

	LCD_DC_Data;     // Êý¾ÝÖ¸ÁîÑ¡Ôñ Òý½ÅÊä³ö¸ßµçÆ½£¬´ú±í±¾´Î´«Êä Êý¾Ý

// ÐÞ¸ÄÎª16Î»Êý¾Ý¿í¶È£¬Ð´ÈëÊý¾Ý¸ü¼ÓÐ§ÂÊ£¬²»ÐèÒª²ð·Ö
   LCD_SPI.Init.DataSize 	= SPI_DATASIZE_16BIT;   //	16Î»Êý¾Ý¿í¶È
   HAL_SPI_Init(&LCD_SPI);

	LCD_SPI_TransmitBuffer(&LCD_SPI, DataBuff,width * height) ;

//	HAL_SPI_Transmit(&hspi5, (uint8_t *)DataBuff, (x2-x1+1) * (y2-y1+1), 1000) ;

// ¸Ä»Ø8Î»Êý¾Ý¿í¶È£¬ÒòÎªÖ¸ÁîºÍ²¿·ÖÊý¾Ý¶¼ÊÇ°´ÕÕ8Î»´«ÊäµÄ
	LCD_SPI.Init.DataSize 	= SPI_DATASIZE_8BIT;    //	8Î»Êý¾Ý¿í¶È
   HAL_SPI_Init(&LCD_SPI);

}

/*****************************************************************************************
* º¯ Êý Ãû: DrawRoundRect
* Èë¿Ú²ÎÊý: int x - Ô²½Ç¾ØÐÎ×óÉÏ½Çºá×ø±ê
*           int y - Ô²½Ç¾ØÐÎ×óÉÏ½Ç×Ý×ø±ê
*           unsigned char w - Ô²½Ç¾ØÐÎ¿í¶È
*           unsigned char h - Ô²½Ç¾ØÐÎ¸ß¶È
*           unsigned char r - Ô²½Ç°ë¾¶
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÔÚLCDÉÏ»æÖÆÔ²½Ç¾ØÐÎ
* Ëµ    Ã÷: Í¨¹ý»æÖÆË®Æ½ºÍ´¹Ö±Ïß£¬ÒÔ¼°ÔÚËÄ¸ö½ÇÉÏ»æÖÆÔ²»¡£¬ÊµÏÖÔ²½Ç¾ØÐÎµÄ»æÖÆ¡£º¯ÊýÖÐÊ¹
*           ÓÃ DrawCircleHelper º¯Êý»æÖÆÔ²½Ç¾ØÐÎµÄËÄ¸öÔ²½Ç¡£
******************************************************************************************/
void DrawRoundRect(int x, int y, unsigned char w, unsigned char h, unsigned char r)
{
    // »æÖÆÉÏÏÂÁ½ÌõË®Æ½Ïß
    LCD_DrawLine_H(x + r, y, w - 2 * r);        // ÉÏ±ß
    LCD_DrawLine_H(x + r, y + h - 1, w - 2 * r); // ÏÂ±ß

    // »æÖÆ×óÓÒÁ½Ìõ´¹Ö±Ïß
    LCD_DrawLine_V(x, y + r, h - 2 * r);        // ×ó±ß
    LCD_DrawLine_V(x + w - 1, y + r, h - 2 * r); // ÓÒ±ß

    // »æÖÆËÄ¸öÔ²½Ç
    DrawCircleHelper(x + r, y + r, r, 1);                 // ×óÉÏ½Ç
    DrawCircleHelper(x + w - r - 1, y + r, r, 2);         // ÓÒÉÏ½Ç
    DrawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4); // ÓÒÏÂ½Ç
    DrawCircleHelper(x + r, y + h - r - 1, r, 8);         // ×óÏÂ½Ç
}

/*****************************************************************************************
* º¯ Êý Ãû: DrawfillRoundRect
* Èë¿Ú²ÎÊý: int x - Ô²½Ç¾ØÐÎ×óÉÏ½Çºá×ø±ê
*           int y - Ô²½Ç¾ØÐÎ×óÉÏ½Ç×Ý×ø±ê
*           unsigned char w - Ô²½Ç¾ØÐÎ¿í¶È
*           unsigned char h - Ô²½Ç¾ØÐÎ¸ß¶È
*           unsigned char r - Ô²½Ç°ë¾¶
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÔÚLCDÉÏ»æÖÆÌî³äµÄÔ²½Ç¾ØÐÎ
* Ëµ    Ã÷: Í¨¹ý»æÖÆÒ»ÌõË®Æ½ÏßºÍÔÚÁ½¸ö½ÇÉÏ»æÖÆÌî³äµÄÔ²»¡£¬ÊµÏÖÌî³äµÄÔ²½Ç¾ØÐÎµÄ»æÖÆ¡£º¯Êý
*           ÖÐÊ¹ÓÃ DrawFillCircleHelper º¯Êý»æÖÆÁ½¸ö½ÇµÄÌî³äÔ²»¡£¬Í¬Ê±Ê¹ÓÃ LCD_FillRect
*           º¯Êý»æÖÆ¾ØÐÎµÄÖÐ¼ä²¿·Ö¡£
******************************************************************************************/
void DrawfillRoundRect(int x, int y, unsigned char w, unsigned char h, unsigned char r)
{
    // »æÖÆ¾ØÐÎµÄÖÐ¼ä²¿·Ö
    LCD_FillRect(x + r, y, w - 2 * r, h);

    // »æÖÆÁ½¸ö½ÇµÄÌî³äÔ²»¡
    DrawFillCircleHelper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1); // ÓÒÉÏ½Ç
    DrawFillCircleHelper(x + r, y + r, r, 2, h - 2 * r - 1);         // ×óÉÏ½Ç
}


/*****************************************************************************************
* º¯ Êý Ãû: DrawCircleHelper
* Èë¿Ú²ÎÊý: int x0 - Ô²ÐÄºá×ø±ê
*           int y0 - Ô²ÐÄ×Ý×ø±ê
*           unsigned char r - Ô²°ë¾¶
*           unsigned char cornername - Ô²µÄËÄ¸ö½ÇµÄ×éºÏÖµ
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÔÚLCDÉÏ»æÖÆÔ²µÄÒ»²¿·Ö£¬¿ÉÑ¡Ôñ»æÖÆµÄÔ²½Ç
* Ëµ    Ã÷: Ê¹ÓÃ Bresenham Ëã·¨»æÖÆÔ²µÄÒ»²¿·Ö£¬¿ÉÒÔÑ¡ÔñÔÚËÄ¸ö½ÇÖÐµÄÒ»¸ö»ò¶à¸ö»æÖÆÔ²½Ç¡£
*           Ô²½ÇµÄ×éºÏÖµÓÉ cornername ¾ö¶¨£¬Ã¿¸öÎ»´ú±íÒ»¸ö½Ç£¬1 ´ú±í»æÖÆ£¬0 ´ú±í²»»æÖÆ¡£
*           º¯ÊýÖÐÊ¹ÓÃ LCD_DrawPoint º¯Êý»æÖÆÃ¿¸öÏñËØµã¡£
******************************************************************************************/
void DrawCircleHelper(int x0, int y0, unsigned char r, unsigned char cornername)
{
    int f = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x = 0;
    int y = r;

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }

        x++;
        ddF_x += 2;
        f += ddF_x;

        if (cornername & 0x4)
        {
            LCD_DrawPoint(x0 + x, y0 + y, LIGHT_GREEN);
            LCD_DrawPoint(x0 + y, y0 + x, LIGHT_GREEN);
        }
        if (cornername & 0x2)
        {
            LCD_DrawPoint(x0 + x, y0 - y, LIGHT_GREEN);
            LCD_DrawPoint(x0 + y, y0 - x, LIGHT_GREEN);
        }
        if (cornername & 0x8)
        {
            LCD_DrawPoint(x0 - y, y0 + x, LIGHT_GREEN);
            LCD_DrawPoint(x0 - x, y0 + y, LIGHT_GREEN);
        }
        if (cornername & 0x1)
        {
            LCD_DrawPoint(x0 - y, y0 - x, LIGHT_GREEN);
            LCD_DrawPoint(x0 - x, y0 - y, LIGHT_GREEN);
        }
    }
}

/*****************************************************************************************
* º¯ Êý Ãû: DrawFillCircleHelper
* Èë¿Ú²ÎÊý: int x0 - Ô²ÐÄºá×ø±ê
*           int y0 - Ô²ÐÄ×Ý×ø±ê
*           unsigned char r - Ô²°ë¾¶
*           unsigned char cornername - Ô²µÄËÄ¸ö½ÇµÄ×éºÏÖµ
*           int delta - ÐÞÕýÖµ£¬ÓÃÓÚÎ¢µ÷»æÖÆÐ§¹û
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÔÚLCDÉÏ»æÖÆÌî³äÔ²µÄÒ»²¿·Ö£¬¿ÉÑ¡Ôñ»æÖÆµÄÔ²½Ç
* Ëµ    Ã÷: Ê¹ÓÃ Bresenham Ëã·¨»æÖÆÌî³äÔ²µÄÒ»²¿·Ö£¬¿ÉÒÔÑ¡ÔñÔÚÁ½¸ö½ÇÖÐµÄÒ»¸ö»òÁ½¸ö»æÖÆÔ²½Ç¡£
*           Ô²½ÇµÄ×éºÏÖµÓÉ cornername ¾ö¶¨£¬Ã¿¸öÎ»´ú±íÒ»¸ö½Ç£¬1 ´ú±í»æÖÆ£¬0 ´ú±í²»»æÖÆ¡£
*           º¯ÊýÖÐÊ¹ÓÃ LCD_DrawLine_V º¯Êý»æÖÆÃ¿ÁÐÏñËØµã¡£
******************************************************************************************/
void DrawFillCircleHelper(int x0, int y0, unsigned char r, unsigned char cornername, int delta)
{
    int f = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x = 0;
    int y = r;

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }

        x++;
        ddF_x += 2;
        f += ddF_x;

        if (cornername & 0x1)
        {
            LCD_DrawLine_V(x0 + x, y0 - y, 2 * y + 1 + delta);
            LCD_DrawLine_V(x0 + y, y0 - x, 2 * x + 1 + delta);
        }

        if (cornername & 0x2)
        {
            LCD_DrawLine_V(x0 - x, y0 - y, 2 * y + 1 + delta);
            LCD_DrawLine_V(x0 - y, y0 - x, 2 * x + 1 + delta);
        }
    }
}

/*****************************************************************************************
* º¯ Êý Ãû: DrawFillEllipse
* Èë¿Ú²ÎÊý: int x0 - ÍÖÔ²ÖÐÐÄºá×ø±ê
*           int y0 - ÍÖÔ²ÖÐÐÄ×Ý×ø±ê
*           int rx - ÍÖÔ² x Öá°ë¾¶
*           int ry - ÍÖÔ² y Öá°ë¾¶
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÔÚLCDÉÏ»æÖÆÌî³äÍÖÔ²
* Ëµ    Ã÷: Ê¹ÓÃ Bresenham Ëã·¨»æÖÆÌî³äÍÖÔ²¡£º¯ÊýÖÐÊ¹ÓÃ LCD_DrawLine_V º¯Êý»æÖÆÃ¿ÁÐÏñËØµã¡£
******************************************************************************************/
void DrawFillEllipse(int x0, int y0, int rx, int ry)
{
    int x, y;
    int xchg, ychg;
    int err;
    int rxrx2;
    int ryry2;
    int stopx, stopy;

    rxrx2 = rx;
    rxrx2 *= rx;
    rxrx2 *= 2;

    ryry2 = ry;
    ryry2 *= ry;
    ryry2 *= 2;

    x = rx;
    y = 0;

    xchg = 1;
    xchg -= rx;
    xchg -= rx;
    xchg *= ry;
    xchg *= ry;

    ychg = rx;
    ychg *= rx;

    err = 0;

    stopx = ryry2;
    stopx *= rx;
    stopy = 0;

    while (stopx >= stopy)
    {
        // »æÖÆÍÖÔ²µÄËÄ¸öÏóÏÞµÄÒ»ÁÐÏñËØµã
        LCD_DrawLine_V(x0 + x, y0 - y, y + 1);
        LCD_DrawLine_V(x0 - x, y0 - y, y + 1);
        LCD_DrawLine_V(x0 + x, y0, y + 1);
        LCD_DrawLine_V(x0 - x, y0, y + 1);

        y++;
        stopy += rxrx2;
        err += ychg;
        ychg += rxrx2;

        if (2 * err + xchg > 0)
        {
            x--;
            stopx -= ryry2;
            err += xchg;
            xchg += ryry2;
        }
    }

    x = 0;
    y = ry;

    xchg = ry;
    xchg *= ry;

    ychg = 1;
    ychg -= ry;
    ychg -= ry;
    ychg *= rx;
    ychg *= rx;

    err = 0;

    stopx = 0;
    stopy = rxrx2;
    stopy *= ry;

    while (stopx <= stopy)
    {
        // »æÖÆÍÖÔ²µÄËÄ¸öÏóÏÞµÄÒ»ÁÐÏñËØµã
        LCD_DrawLine_V(x0 + x, y0 - y, y + 1);
        LCD_DrawLine_V(x0 - x, y0 - y, y + 1);
        LCD_DrawLine_V(x0 + x, y0, y + 1);
        LCD_DrawLine_V(x0 - x, y0, y + 1);

        x++;
        stopx += ryry2;
        err += xchg;
        xchg += ryry2;

        if (2 * err + ychg > 0)
        {
            y--;
            stopy -= rxrx2;
            err += ychg;
            ychg += rxrx2;
        }
    }
}

/*****************************************************************************************
* º¯ Êý Ãû: DrawTriangle
* Èë¿Ú²ÎÊý: unsigned char x0 - Èý½ÇÐÎµÚÒ»¸ö¶¥µãºá×ø±ê
*           unsigned char y0 - Èý½ÇÐÎµÚÒ»¸ö¶¥µã×Ý×ø±ê
*           unsigned char x1 - Èý½ÇÐÎµÚ¶þ¸ö¶¥µãºá×ø±ê
*           unsigned char y1 - Èý½ÇÐÎµÚ¶þ¸ö¶¥µã×Ý×ø±ê
*           unsigned char x2 - Èý½ÇÐÎµÚÈý¸ö¶¥µãºá×ø±ê
*           unsigned char y2 - Èý½ÇÐÎµÚÈý¸ö¶¥µã×Ý×ø±ê
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÔÚLCDÉÏ»æÖÆÈý½ÇÐÎ
* Ëµ    Ã÷: Ê¹ÓÃ LCD_DrawLine º¯Êý»æÖÆÈý½ÇÐÎµÄÈýÌõ±ß¡£
******************************************************************************************/
void DrawTriangle(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2)
{
    LCD_DrawLine(x0, y0, x1, y1);
    LCD_DrawLine(x1, y1, x2, y2);
    LCD_DrawLine(x2, y2, x0, y0);
}

/*****************************************************************************************
* º¯ Êý Ãû: DrawFillTriangle
* Èë¿Ú²ÎÊý: int x0 - Èý½ÇÐÎµÚÒ»¸ö¶¥µãºá×ø±ê
*           int y0 - Èý½ÇÐÎµÚÒ»¸ö¶¥µã×Ý×ø±ê
*           int x1 - Èý½ÇÐÎµÚ¶þ¸ö¶¥µãºá×ø±ê
*           int y1 - Èý½ÇÐÎµÚ¶þ¸ö¶¥µã×Ý×ø±ê
*           int x2 - Èý½ÇÐÎµÚÈý¸ö¶¥µãºá×ø±ê
*           int y2 - Èý½ÇÐÎµÚÈý¸ö¶¥µã×Ý×ø±ê
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÔÚLCDÉÏ»æÖÆÌî³äµÄÈý½ÇÐÎ
* Ëµ    Ã÷: Ê¹ÓÃ LCD_DrawLine_H º¯Êý»æÖÆÈý½ÇÐÎµÄ¸÷¸öË®Æ½É¨ÃèÏß¡£
******************************************************************************************/
void DrawFillTriangle(int x0, int y0, int x1, int y1, int x2, int y2)
{
    int a, b, y, last;
    int dx01, dy01, dx02, dy02, dx12, dy12, sa = 0, sb = 0;

    // ±£Ö¤y0<=y1<=y2
    if (y0 > y1) { SWAP(y0, y1); SWAP(x0, x1); }
    if (y1 > y2) { SWAP(y2, y1); SWAP(x2, x1); }
    if (y0 > y1) { SWAP(y0, y1); SWAP(x0, x1); }

    // Èý½ÇÐÎÍË»¯³ÉÏß¶Î»òÎªµ¥µã
    if (y0 == y2)
    {
        a = b = x0;
        if (x1 < a) a = x1;
        else if (x1 > b) b = x1;
        if (x2 < a) a = x2;
        else if (x2 > b) b = x2;

        LCD_DrawLine_H(a, y0, b - a + 1);
        return;
    }

    dx01 = x1 - x0;
    dy01 = y1 - y0;
    dx02 = x2 - x0;
    dy02 = y2 - y0;
    dx12 = x2 - x1;
    dy12 = y2 - y1;
    sa = 0;
    sb = 0;

    if (y1 == y2)
    {
        last = y1;   // Include y1 scanline
    }
    else
    {
        last = y1 - 1; // Skip it
    }

    // »æÖÆÉÏ°ë²¿·ÖÈý½ÇÐÎ
    for (y = y0; y <= last; y++)
    {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;

        if (a > b) {SWAP(a, b);}

        LCD_DrawLine_H(a, y, b - a + 1);
    }

    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);

    // »æÖÆÏÂ°ë²¿·ÖÈý½ÇÐÎ
    for (; y <= y2; y++)
    {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;

        if (a > b) {SWAP(a, b);}

        LCD_DrawLine_H(a, y, b - a + 1);
    }

}


/*****************************************************************************************
* º¯ Êý Ãû: DrawArc
* Èë¿Ú²ÎÊý: int x - Ô²ÐÄºá×ø±ê
*           int y - Ô²ÐÄ×Ý×ø±ê
*           unsigned char r - »¡ÐÎ°ë¾¶
*           int angle_start - »¡ÐÎÆðÊ¼½Ç¶È
*           int angle_end - »¡ÐÎÖÕÖ¹½Ç¶È
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: »æÖÆÖ¸¶¨Ô²ÐÄ¡¢°ë¾¶µÄ»¡ÐÎ¡£
* Ëµ    Ã÷: ¸Ãº¯Êý´ÓÆðÊ¼½Ç¶Èµ½ÖÕÖ¹½Ç¶ÈÒÔ5¶È²½½ø»æÖÆ»¡ÐÎ£¬Ê¹ÓÃÁËÐý×ªº¯ÊýºÍÖ±Ïß»æÖÆº¯Êý¡£
******************************************************************************************/
void DrawArc(int x, int y, unsigned char r, int angle_start, int angle_end)
{
    float i = 0;
    TypeXY m, temp;
    temp = GetXY();
    SetRotateCenter(x, y);
    SetAngleDir(0);

    if (angle_end > 360)
        angle_end = 360;

    SetAngle(0);
    m = GetRotateXY(x, y + r);
    MoveTo(m.x, m.y);

    for (i = angle_start; i < angle_end; i += 5)
    {
        SetAngle(i);
        m = GetRotateXY(x, y + r);
        LineTo(m.x, m.y);
    }

    MoveTo(temp.x, temp.y);
}

/*****************************************************************************************
* º¯ Êý Ãû: GetXY
* ·µ »Ø Öµ: TypeXY - °üº¬ºá×ø±êºÍ×Ý×ø±êµÄ¶þÎ¬×ø±ê½á¹¹Ìå
* º¯Êý¹¦ÄÜ: »ñÈ¡×ø±êÖµµÄº¯Êý
* Ëµ    Ã÷: ·µ»ØÒ»¸ö TypeXY ½á¹¹Ìå£¬ÆäÖÐ°üº¬ºá×ø±êºÍ×Ý×ø±êµÄÖµ¡£
******************************************************************************************/
TypeXY GetXY(void)
{
    TypeXY m;
    m.x = _pointx;  // »ñÈ¡ºá×ø±êÖµ
    m.y = _pointy;  // »ñÈ¡×Ý×ø±êÖµ
    return m;       // ·µ»Ø°üº¬×ø±êÖµµÄ½á¹¹Ìå
}

/*****************************************************************************************
* º¯ Êý Ãû: SetRotateCenter
* Èë¿Ú²ÎÊý: int x0 - Ðý×ªÖÐÐÄµÄºá×ø±ê
*           int y0 - Ðý×ªÖÐÐÄµÄ×Ý×ø±ê
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÉèÖÃÐý×ªÖÐÐÄµÄ×ø±ê¡£
* Ëµ    Ã÷: Í¨¹ýµ÷ÓÃ¸Ãº¯Êý£¬¿ÉÒÔÉèÖÃÐý×ª²Ù×÷µÄÖÐÐÄ×ø±ê£¬¼´Î§ÈÆÄÄ¸öµã½øÐÐÐý×ª¡£
******************************************************************************************/
void SetRotateCenter(int x0, int y0)
{
    _RoateValue.center.x = x0;
    _RoateValue.center.y = y0;
}

/*****************************************************************************************
* º¯ Êý Ãû: SetAngleDir
* Èë¿Ú²ÎÊý: int direction - Ðý×ª·½Ïò£¬1ÎªË³Ê±Õë£¬-1ÎªÄæÊ±Õë
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÉèÖÃÐý×ªµÄ·½Ïò¡£
* Ëµ    Ã÷: Í¨¹ýµ÷ÓÃ¸Ãº¯Êý£¬¿ÉÒÔÉèÖÃÐý×ªµÄ·½Ïò£¬1±íÊ¾Ë³Ê±Õë£¬-1±íÊ¾ÄæÊ±Õë¡£
******************************************************************************************/
void SetAngleDir(int direction)
{
    _RoateValue.direct = direction;
}


/*****************************************************************************************
* º¯ Êý Ãû: SetAngle
* Èë¿Ú²ÎÊý: float angle - Ðý×ª½Ç¶È£¨µ¥Î»£º¶È£©
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÉèÖÃÐý×ªµÄ½Ç¶È¡£
* Ëµ    Ã÷: Í¨¹ýµ÷ÓÃ¸Ãº¯Êý£¬¿ÉÒÔÉèÖÃÐý×ªµÄ½Ç¶È£¬µ¥Î»Îª¶È¡£ÄÚ²¿»á½«½Ç¶È×ª»»Îª»¡¶È½øÐÐ´¦Àí¡£
******************************************************************************************/
void SetAngle(float angle)
{
    _RoateValue.angle = RADIAN(angle);
}

/*****************************************************************************************
* º¯ Êý Ãû: Rotate
* Èë¿Ú²ÎÊý: int x0 - Ðý×ªÖÐÐÄµÄºá×ø±ê
*           int y0 - Ðý×ªÖÐÐÄµÄ×Ý×ø±ê
*           int *x - ´ýÐý×ªµãµÄºá×ø±ê
*           int *y - ´ýÐý×ªµãµÄ×Ý×ø±ê
*           double angle - Ðý×ª½Ç¶È£¨»¡¶È£©
*           int direction - Ðý×ª·½Ïò£¬·ÇÁã±íÊ¾Ë³Ê±Õë£¬Áã±íÊ¾ÄæÊ±Õë
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ¶ÔÖ¸¶¨µã½øÐÐÐý×ª¡£
* Ëµ    Ã÷: Í¨¹ýµ÷ÓÃ¸Ãº¯Êý£¬¿ÉÒÔ¶Ô¸ø¶¨µÄµãÎ§ÈÆÖ¸¶¨ÖÐÐÄ½øÐÐÐý×ª¡£Ðý×ª½Ç¶ÈÓÉ angle ²ÎÊýÖ¸¶¨£¬
*           direction ²ÎÊýÖ¸¶¨Ðý×ª·½Ïò£¬·ÇÁã±íÊ¾Ë³Ê±Õë£¬Áã±íÊ¾ÄæÊ±Õë¡£
******************************************************************************************/
static void Rotate(int x0, int y0, int *x, int *y, double angle, int direction)
{
    int temp = (*y - y0) * (*y - y0) + (*x - x0) * (*x - x0);
    double r = mySqrt(temp);
    double a0 = atan2(*x - x0, *y - y0);

    if (direction)
    {
        *x = x0 + r * cos(a0 + angle);
        *y = y0 + r * sin(a0 + angle);
    }
    else
    {
        *x = x0 + r * cos(a0 - angle);
        *y = y0 + r * sin(a0 - angle);
    }
}


/*****************************************************************************************
* º¯ Êý Ãû: mySqrt
* Èë¿Ú²ÎÊý: float x - ´ýÇóÆ½·½¸ùµÄÊýÖµ
* ·µ »Ø Öµ: float - ¼ÆËãµÃµ½µÄÆ½·½¸ùÖµ
* º¯Êý¹¦ÄÜ: ¼ÆËã¸ø¶¨ÊýÖµµÄÆ½·½¸ù¡£
* Ëµ    Ã÷: ¸Ãº¯Êý²ÉÓÃÅ£¶Ùµü´ú·¨¼ÆËãÆ½·½¸ù£¬¾­¹ý¶à´Îµü´ú±Æ½üÆ½·½¸ùµÄÕæÊµÖµ¡£
******************************************************************************************/
float mySqrt(float x)
{
    float a = x;
    unsigned int i = *(unsigned int *)&x;
    i = (i + 0x3f76cf62) >> 1;
    x = *(float *)&i;
    x = (x + a / x) * 0.5;
    return x;
}


/*****************************************************************************************
* º¯ Êý Ãû: GetRotateXY
* Èë¿Ú²ÎÊý: int x - ´ýÐý×ªµÄµãµÄÔ­Ê¼ºá×ø±ê
*           int y - ´ýÐý×ªµÄµãµÄÔ­Ê¼×Ý×ø±ê
* ·µ »Ø Öµ: TypeXY - Ðý×ªºóµÄµã×ø±ê
* º¯Êý¹¦ÄÜ: »ñÈ¡¾­¹ýÐý×ª±ä»»ºóµÄµã×ø±ê¡£
* Ëµ    Ã÷: Èç¹ûÐý×ª½Ç¶È²»Îª0£¬Í¨¹ýµ÷ÓÃ Rotate º¯ÊýÊµÏÖ¶Ô¸ø¶¨µãµÄÐý×ª£¬
*          È»ºó½«Ðý×ªºóµÄ×ø±ê±£´æÔÚ TypeXY ½á¹¹ÌåÖÐ·µ»Ø¡£
******************************************************************************************/
TypeXY GetRotateXY(int x, int y)
{
    TypeXY temp;
    int m = x, n = y;
    Rotate(_RoateValue.center.x, _RoateValue.center.y, &m, &n, _RoateValue.angle, _RoateValue.direct);
    temp.x = m;
    temp.y = n;
    return temp;
}


/*****************************************************************************************
* º¯ Êý Ãû: MoveTo
* Èë¿Ú²ÎÊý: int x - ÒÆ¶¯µ½µÄÄ¿±êºá×ø±ê
*           int y - ÒÆ¶¯µ½µÄÄ¿±ê×Ý×ø±ê
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÒÆ¶¯»æÍ¼¹â±êµ½Ö¸¶¨µÄÄ¿±êÎ»ÖÃ¡£
* Ëµ    Ã÷: ½«»æÍ¼¹â±êµÄµ±Ç°Î»ÖÃ¸üÐÂÎª¸ø¶¨µÄÄ¿±êÎ»ÖÃ£¬ÒÔ±ãºóÐøµÄ»æÍ¼²Ù×÷·¢ÉúÔÚ¸ÃÄ¿±êÎ»ÖÃÉÏ¡£
******************************************************************************************/
void MoveTo(int x, int y)
{
    _pointx = x;
    _pointy = y;
}


/*****************************************************************************************
* º¯ Êý Ãû: LineTo
* Èë¿Ú²ÎÊý: int x - Ä¿±êºá×ø±ê
*           int y - Ä¿±ê×Ý×ø±ê
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÔÚµ±Ç°¹â±êÎ»ÖÃÓëÖ¸¶¨Ä¿±êÎ»ÖÃÖ®¼ä»æÖÆÖ±Ïß¡£
* Ëµ    Ã÷: ¸Ãº¯Êý»æÖÆ´Óµ±Ç°¹â±êÎ»ÖÃµ½Ä¿±êÎ»ÖÃµÄÖ±Ïß£¬²¢½«¹â±ê¸üÐÂÎªÄ¿±êÎ»ÖÃ¡£
******************************************************************************************/
void LineTo(int x, int y)
{
    LCD_DrawLine(_pointx, _pointy, x, y);
    _pointx = x;
    _pointy = y;
}


/*****************************************************************************************
* º¯ Êý Ãû: SetRotateValue
* Èë¿Ú²ÎÊý: int x - Ðý×ªÖÐÐÄºá×ø±ê
*           int y - Ðý×ªÖÐÐÄ×Ý×ø±ê
*           float angle - Ðý×ª½Ç¶È£¨»¡¶È£©
*           int direct - Ðý×ª·½Ïò£¨0ÎªË³Ê±Õë£¬1ÎªÄæÊ±Õë£©
* ·µ »Ø Öµ: ÎÞ
* º¯Êý¹¦ÄÜ: ÉèÖÃÐý×ª²ÎÊý£¬°üÀ¨Ðý×ªÖÐÐÄ¡¢Ðý×ª½Ç¶ÈºÍÐý×ª·½Ïò¡£
* Ëµ    Ã÷: ¸Ãº¯ÊýÍ¨¹ýµ÷ÓÃÆäËûÉèÖÃº¯Êý£¬ÊµÏÖÁË¶ÔÐý×ª²ÎÊýµÄÒ»´ÎÐÔÉèÖÃ¡£
******************************************************************************************/
void SetRotateValue(int x, int y, float angle, int direct)
{
    SetRotateCenter(x, y);
    SetAngleDir(direct);
    SetAngle(angle);
}

/**********************************************************************************************************************************
*
* ÒÔÏÂ¼¸¸öº¯ÊýÐÞ¸ÄÓÚHALµÄ¿âº¯Êý£¬Ä¿µÄÊÇÎªÁËSPI´«ÊäÊý¾Ý²»ÏÞÊý¾Ý³¤¶ÈµÄÐ´Èë£¬²¢ÇÒÌá¸ßÇåÆÁµÄËÙ¶È
*
*****************************************************************************************************************FANKE************/


/**
  * @brief Handle SPI Communication Timeout.
  * @param hspi: pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @param Flag: SPI flag to check
  * @param Status: flag state to check
  * @param Timeout: Timeout duration
  * @param Tickstart: Tick start value
  * @retval HAL status
  */
HAL_StatusTypeDef LCD_SPI_WaitOnFlagUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status,
                                                    uint32_t Tickstart, uint32_t Timeout)
{
   /* Wait until flag is set */
   while ((__HAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) == Status)
   {
      /* Check for the Timeout */
      if ((((HAL_GetTick() - Tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
      {
         return HAL_TIMEOUT;
      }
   }
   return HAL_OK;
}


/**
 * @brief  Close Transfer and clear flags.
 * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval HAL_ERROR: if any error detected
 *         HAL_OK: if nothing detected
 */
 void LCD_SPI_CloseTransfer(SPI_HandleTypeDef *hspi)
{
  uint32_t itflag = hspi->Instance->SR;

  __HAL_SPI_CLEAR_EOTFLAG(hspi);
  __HAL_SPI_CLEAR_TXTFFLAG(hspi);

  /* Disable SPI peripheral */
  __HAL_SPI_DISABLE(hspi);

  /* Disable ITs */
  __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));

  /* Disable Tx DMA Request */
  CLEAR_BIT(hspi->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

  /* Report UnderRun error for non RX Only communication */
  if (hspi->State != HAL_SPI_STATE_BUSY_RX)
  {
    if ((itflag & SPI_FLAG_UDR) != 0UL)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_UDR);
      __HAL_SPI_CLEAR_UDRFLAG(hspi);
    }
  }

  /* Report OverRun error for non TX Only communication */
  if (hspi->State != HAL_SPI_STATE_BUSY_TX)
  {
    if ((itflag & SPI_FLAG_OVR) != 0UL)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_OVR);
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }
  }

  /* SPI Mode Fault error interrupt occurred -------------------------------*/
  if ((itflag & SPI_FLAG_MODF) != 0UL)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_MODF);
    __HAL_SPI_CLEAR_MODFFLAG(hspi);
  }

  /* SPI Frame error interrupt occurred ------------------------------------*/
  if ((itflag & SPI_FLAG_FRE) != 0UL)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FRE);
    __HAL_SPI_CLEAR_FREFLAG(hspi);
  }

  hspi->TxXferCount = (uint16_t)0UL;
  hspi->RxXferCount = (uint16_t)0UL;
}


/**
  * @brief  ×¨ÎªÆÁÄ»ÇåÆÁ¶øÐÞ¸Ä£¬½«ÐèÒªÇåÆÁµÄÑÕÉ«ÅúÁ¿´«Êä
  * @param  hspi   : spiµÄ¾ä±ú
  * @param  pData  : ÒªÐ´ÈëµÄÊý¾Ý
  * @param  Size   : Êý¾Ý´óÐ¡
  * @retval HAL status
  */

HAL_StatusTypeDef LCD_SPI_Transmit(SPI_HandleTypeDef *hspi,uint16_t pData, uint32_t Size)
{
   uint32_t    tickstart;
   uint32_t    Timeout = 1000;      // ³¬Ê±ÅÐ¶Ï
   uint32_t    LCD_pData_32bit;     // °´32Î»´«ÊäÊ±µÄÊý¾Ý
   uint32_t    LCD_TxDataCount;     // ´«Êä¼ÆÊý
   HAL_StatusTypeDef errorcode = HAL_OK;

	/* Check Direction parameter */
	assert_param(IS_SPI_DIRECTION_2LINES_OR_1LINE_2LINES_TXONLY(hspi->Init.Direction));

	/* Process Locked */
	__HAL_LOCK(hspi);

	/* Init tickstart for timeout management*/
	tickstart = HAL_GetTick();

	if (hspi->State != HAL_SPI_STATE_READY)
	{
		errorcode = HAL_BUSY;
		__HAL_UNLOCK(hspi);
		return errorcode;
	}

	if ( Size == 0UL)
	{
		errorcode = HAL_ERROR;
		__HAL_UNLOCK(hspi);
		return errorcode;
	}

	/* Set the transaction information */
	hspi->State       = HAL_SPI_STATE_BUSY_TX;
	hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

	LCD_TxDataCount   = Size;                // ´«ÊäµÄÊý¾Ý³¤¶È
	LCD_pData_32bit   = (pData<<16)|pData ;  // °´32Î»´«ÊäÊ±£¬ºÏ²¢2¸öÏñËØµãµÄÑÕÉ«

	/*Init field not used in handle to zero */
	hspi->pRxBuffPtr  = NULL;
	hspi->RxXferSize  = (uint16_t) 0UL;
	hspi->RxXferCount = (uint16_t) 0UL;
	hspi->TxISR       = NULL;
	hspi->RxISR       = NULL;

	/* Configure communication direction : 1Line */
	if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
	{
		SPI_1LINE_TX(hspi);
	}

// ²»Ê¹ÓÃÓ²¼þ TSIZE ¿ØÖÆ£¬´Ë´¦ÉèÖÃÎª0£¬¼´²»ÏÞÖÆ´«ÊäµÄÊý¾Ý³¤¶È
	MODIFY_REG(hspi->Instance->CR2, SPI_CR2_TSIZE, 0);

	/* Enable SPI peripheral */
	__HAL_SPI_ENABLE(hspi);

	if (hspi->Init.Mode == SPI_MODE_MASTER)
	{
		 /* Master transfer start */
		 SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
	}

	/* Transmit data in 16 Bit mode */
	while (LCD_TxDataCount > 0UL)
	{
		/* Wait until TXP flag is set to send data */
		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXP))
		{
			if ((hspi->TxXferCount > 1UL) && (hspi->Init.FifoThreshold > SPI_FIFO_THRESHOLD_01DATA))
			{
				*((__IO uint32_t *)&hspi->Instance->TXDR) = (uint32_t )LCD_pData_32bit;
				LCD_TxDataCount -= (uint16_t)2UL;
			}
			else
			{
				*((__IO uint16_t *)&hspi->Instance->TXDR) =  (uint16_t )pData;
				LCD_TxDataCount--;
			}
		}
		else
		{
			/* Timeout management */
			if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
			{
				/* Call standard close procedure with error check */
				LCD_SPI_CloseTransfer(hspi);

				/* Process Unlocked */
				__HAL_UNLOCK(hspi);

				SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_TIMEOUT);
				hspi->State = HAL_SPI_STATE_READY;
				return HAL_ERROR;
			}
		}
	}

	if (LCD_SPI_WaitOnFlagUntilTimeout(hspi, SPI_SR_TXC, RESET, tickstart, Timeout) != HAL_OK)
	{
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
	}

	SET_BIT((hspi)->Instance->CR1 , SPI_CR1_CSUSP); // ÇëÇó¹ÒÆðSPI´«Êä
	/* µÈ´ýSPI¹ÒÆð */
	if (LCD_SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_SUSP, RESET, tickstart, Timeout) != HAL_OK)
	{
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
	}
	LCD_SPI_CloseTransfer(hspi);   /* Call standard close procedure with error check */

	SET_BIT((hspi)->Instance->IFCR , SPI_IFCR_SUSPC);  // Çå³ý¹ÒÆð±êÖ¾Î»


	/* Process Unlocked */
	__HAL_UNLOCK(hspi);

	hspi->State = HAL_SPI_STATE_READY;

	if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
	{
		return HAL_ERROR;
	}
	return errorcode;
}

/**
  * @brief  ×¨ÎªÅúÁ¿Ð´ÈëÊý¾ÝÐÞ¸Ä£¬Ê¹Ö®²»ÏÞ³¤¶ÈµÄ´«ÊäÊý¾Ý
  * @param  hspi   : spiµÄ¾ä±ú
  * @param  pData  : ÒªÐ´ÈëµÄÊý¾Ý
  * @param  Size   : Êý¾Ý´óÐ¡
  * @retval HAL status
  */
HAL_StatusTypeDef LCD_SPI_TransmitBuffer (SPI_HandleTypeDef *hspi, uint16_t *pData, uint32_t Size)
{
   uint32_t    tickstart;
   uint32_t    Timeout = 1000;      // ³¬Ê±ÅÐ¶Ï
   uint32_t    LCD_TxDataCount;     // ´«Êä¼ÆÊý
   HAL_StatusTypeDef errorcode = HAL_OK;

	/* Check Direction parameter */
	assert_param(IS_SPI_DIRECTION_2LINES_OR_1LINE_2LINES_TXONLY(hspi->Init.Direction));

	/* Process Locked */
	__HAL_LOCK(hspi);

	/* Init tickstart for timeout management*/
	tickstart = HAL_GetTick();

	if (hspi->State != HAL_SPI_STATE_READY)
	{
		errorcode = HAL_BUSY;
		__HAL_UNLOCK(hspi);
		return errorcode;
	}

	if ( Size == 0UL)
	{
		errorcode = HAL_ERROR;
		__HAL_UNLOCK(hspi);
		return errorcode;
	}

	/* Set the transaction information */
	hspi->State       = HAL_SPI_STATE_BUSY_TX;
	hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

	LCD_TxDataCount   = Size;                // ´«ÊäµÄÊý¾Ý³¤¶È

	/*Init field not used in handle to zero */
	hspi->pRxBuffPtr  = NULL;
	hspi->RxXferSize  = (uint16_t) 0UL;
	hspi->RxXferCount = (uint16_t) 0UL;
	hspi->TxISR       = NULL;
	hspi->RxISR       = NULL;

	/* Configure communication direction : 1Line */
	if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
	{
		SPI_1LINE_TX(hspi);
	}

// ²»Ê¹ÓÃÓ²¼þ TSIZE ¿ØÖÆ£¬´Ë´¦ÉèÖÃÎª0£¬¼´²»ÏÞÖÆ´«ÊäµÄÊý¾Ý³¤¶È
	MODIFY_REG(hspi->Instance->CR2, SPI_CR2_TSIZE, 0);

	/* Enable SPI peripheral */
	__HAL_SPI_ENABLE(hspi);

	if (hspi->Init.Mode == SPI_MODE_MASTER)
	{
		 /* Master transfer start */
		 SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
	}

	/* Transmit data in 16 Bit mode */
	while (LCD_TxDataCount > 0UL)
	{
		/* Wait until TXP flag is set to send data */
		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXP))
		{
			if ((LCD_TxDataCount > 1UL) && (hspi->Init.FifoThreshold > SPI_FIFO_THRESHOLD_01DATA))
			{
				*((__IO uint32_t *)&hspi->Instance->TXDR) = *((uint32_t *)pData);
				pData += 2;
				LCD_TxDataCount -= 2;
			}
			else
			{
				*((__IO uint16_t *)&hspi->Instance->TXDR) = *((uint16_t *)pData);
				pData += 1;
				LCD_TxDataCount--;
			}
		}
		else
		{
			/* Timeout management */
			if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
			{
				/* Call standard close procedure with error check */
				LCD_SPI_CloseTransfer(hspi);

				/* Process Unlocked */
				__HAL_UNLOCK(hspi);

				SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_TIMEOUT);
				hspi->State = HAL_SPI_STATE_READY;
				return HAL_ERROR;
			}
		}
	}

	if (LCD_SPI_WaitOnFlagUntilTimeout(hspi, SPI_SR_TXC, RESET, tickstart, Timeout) != HAL_OK)
	{
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
	}

	SET_BIT((hspi)->Instance->CR1 , SPI_CR1_CSUSP); // ÇëÇó¹ÒÆðSPI´«Êä
	/* µÈ´ýSPI¹ÒÆð */
	if (LCD_SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_SUSP, RESET, tickstart, Timeout) != HAL_OK)
	{
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
	}
	LCD_SPI_CloseTransfer(hspi);   /* Call standard close procedure with error check */

	SET_BIT((hspi)->Instance->IFCR , SPI_IFCR_SUSPC);  // Çå³ý¹ÒÆð±êÖ¾Î»


	/* Process Unlocked */
	__HAL_UNLOCK(hspi);

	hspi->State = HAL_SPI_STATE_READY;

	if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
	{
		return HAL_ERROR;
	}
	return errorcode;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
