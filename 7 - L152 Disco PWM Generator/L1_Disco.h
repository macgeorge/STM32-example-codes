/**
  ********************************************************************************
  * @brief   STM32 L152 Discovery Library
  * @date    Mar 2016
  * @version 1.0
  * @author  George Christidis
  ********************************************************************************
  * @details
			This library contains the basic functions to initialize and switch
			on/off the onboard LEDs. It also initializes the user button and
			assigns it to the external interrupt EXTI0.
	******************************************************************************
	*/

#idndef __L1_DISCO_H
#define __L1_DISCO_H
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"

// GPIOs
#define PIN_00 ((uint32_t)0)
#define PIN_01 ((uint32_t)1)
#define PIN_02 ((uint32_t)2)
#define PIN_03 ((uint32_t)3)
#define PIN_04 ((uint32_t)4)
#define PIN_05 ((uint32_t)5)
#define PIN_06 ((uint32_t)6)
#define PIN_07 ((uint32_t)7)
#define PIN_08 ((uint32_t)8)
#define PIN_09 ((uint32_t)9)
#define PIN_10 ((uint32_t)10)
#define PIN_11 ((uint32_t)11)
#define PIN_12 ((uint32_t)12)
#define PIN_13 ((uint32_t)13)
#define PIN_14 ((uint32_t)14)
#define PIN_15 ((uint32_t)15)
#define MODE_OUT ((uint32_t)1)
#define MODE_AF  ((uint32_t)2)
#define MODE_ANA ((uint32_t)3)
#define OTYPE_OD ((uint32_t)1)
#define SPEED_50 ((uint32_t)2)

#define LED_BLUE	GPIO_Pin_6
#define LED_GREEN	GPIO_Pin_7

#define DISCO_LedOn(led)		GPIO_SetBits(GPIOB, led)
#define DISCO_LedOff(led)		GPIO_ResetBits(GPIOB, led)
#define DISCO_LedToggle(led)	GPIO_ToggleBits(GPIOB, led)

void DISCO_LedInit(void);
void DISCO_ButtonInit(void);

#endif
