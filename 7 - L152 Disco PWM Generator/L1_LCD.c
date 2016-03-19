/**
  ********************************************************************************
  * @brief   STM32 L152 Discovery LCD
  * @date    Mar 2016
  * @version 1.0
  * @author  George Christidis
  ********************************************************************************
  * @details
			This library provides basic functions to initialize the onboard LCD
			and display text and bargraphs.
	******************************************************************************
	*/
	
#include "MyL1_LCD.h"

void LCDInit(void)
{
	LCD_GLASS_PreInit();
	LCD_GLASS_Init();
}

void LCD_GLASS_PreInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_LCD | RCC_APB1Periph_PWR, ENABLE);

	PWR_RTCAccessCmd(ENABLE);

	RCC_RTCResetCmd(ENABLE);
	RCC_RTCResetCmd(DISABLE);

	RCC_LSEConfig(RCC_LSE_ON);

	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  	{}

	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_LCD) ;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 \
                                | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_LCD) ;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 \
                                | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	GPIO_PinAFConfig(GPIOC, GPIO_PinSource0, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_LCD) ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_LCD) ;
}

void bargraph(int value, int max)
{
	double percentage=(double)value/(double)max;
	BAR0_OFF;
	BAR1_OFF;
	BAR2_OFF;
	BAR3_OFF;
	if (percentage>0.25) BAR0_ON;
	if (percentage>=0.5) BAR1_ON;
	if (percentage>=0.75) BAR2_ON;
	if (percentage==1) BAR3_ON;
	LCD_bar();
}

void UpdateDisplay(char * displaystring, int columns, int points, int bars)
{
	//displaystring: 6characters + terminator
	//points, columns: hexadecimal 0x111111: all on
	//bars: hexadecimal 0x1111: all on
	int i, point, column;
	
	if ((bars&0x0001)>0) BAR0_ON; else BAR0_OFF;
	if ((bars&0x0010)>0) BAR1_ON; else BAR1_OFF;
	if ((bars&0x0100)>0) BAR2_ON; else BAR2_OFF;
	if ((bars&0x1000)>0) BAR3_ON; else BAR3_OFF;
	
	for (i=0; i<6; i++){
		if ((points & (1<<(4*(5-i)))) > 0) point = POINT_ON;
		else point = 0;
		if ((columns & (1<<(4*(5-i))))> 0) column = COLUMN_ON;
		else column = 0;
		LCD_GLASS_WriteChar((unsigned char *)displaystring + i, point, column, i+1);
	}
}
