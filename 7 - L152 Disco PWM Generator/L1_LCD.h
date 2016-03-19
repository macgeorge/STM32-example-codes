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

#ifndef __LCD_H
#define __LCD_H

#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_tim.h"
#include "stm32l_discovery_lcd.h"
#include "stm32l1xx_syscfg.h"

/* Macros used for set/reset bar LCD bar */
#define BAR0_ON  	t_bar[1] |= 8
#define BAR0_OFF 	t_bar[1] &= ~8
#define BAR1_ON  	t_bar[0] |= 8
#define BAR1_OFF 	t_bar[0] &= ~8
#define BAR2_ON  	t_bar[1] |= 2
#define BAR2_OFF 	t_bar[1] &= ~2
#define BAR3_ON 	t_bar[0] |= 2
#define BAR3_OFF 	t_bar[0] &= ~2

void LCDInit(void);

void LCD_GLASS_PreInit(void);

void bargraph(int, int);

void UpdateDisplay(char *, int, int, int);

static unsigned char Slider_Position[7];
extern uint8_t t_bar[2];

#endif
