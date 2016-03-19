/**
  ********************************************************************************
  * @brief   L152 Discovery PWM Generator
  * @date    Mar 2016
  * @version 1.0
  * @author  George Christidis
  ********************************************************************************
  * @details
			This program output a PWM signal to PB7, with variable frequency and
			duty cycle. The values are set using the Discovery's touch sensor and
			are displayed on the embedded LCD. The user button is pressed to
			change between setting the frequency and the duty cycle.
	******************************************************************************
	*/

#include "L1_LCD.h"
#include "L1_Disco.h"
#include "TSL.h"
#include "Delay.h"
#include "stdio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_tim.h"
#include "stm32l_discovery_lcd.h"
#include "stm32l1xx_syscfg.h"

enum selection {FREQ=0, DUTY} state;
char displaystring[7];
volatile double frequency=50., duty=50.;
volatile int touchvalue;
void UpdateDisplay1(void), UpdatePWM(double, double), TIM4Init();

void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		DISCO_LedToggle(LED_BLUE);
		if (state==FREQ) state=DUTY;
		else state=FREQ;
		touchvalue=getTouch();
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

int main()
{
	LCDInit();
	DISCO_LedInit();
	DISCO_ButtonInit();
	DelayInit();
  	TIM4Init();
	TSLInit();
	
	LCD_GLASS_DisplayString((uint8_t*)" FREQ");
	Delayms(2000);
	LCD_GLASS_Clear();
	DISCO_LedOn(LED_BLUE);
	
	
	while(1)
	{
		if (TSL_user_Action() == TSL_STATUS_OK) {
			if (state==FREQ) {
				if (getTouch()!=touchvalue){
					frequency = 200.*getTouch()/127.;
					if (frequency<0.1) frequency=0.1;
				}
			}
			if (state==DUTY) {
				if (getTouch()!=touchvalue){
					duty = 100.*getTouch()/127.;
					if (duty<0.5) duty=0.5;
				}
			}
			UpdatePWM(frequency*1000., duty);
			UpdateDisplay1();
		}
    }
}

void UpdateDisplay1(void)
{
	bargraph((int) duty, 100);
	if (state==FREQ) sprintf(displaystring,"f%4.2dk", (int) (10*frequency));
	if (state==DUTY) sprintf(displaystring,"d%4.2d%%", (int) (10*duty));
  	LCD_GLASS_WriteChar((unsigned char *)displaystring + 0, 0, COLUMN_ON, 1);
  	LCD_GLASS_WriteChar((unsigned char *)displaystring + 1, 0, 0, 2);
  	LCD_GLASS_WriteChar((unsigned char *)displaystring + 2, 0, 0, 3);
  	LCD_GLASS_WriteChar((unsigned char *)displaystring + 3, POINT_ON, 0, 4);
  	LCD_GLASS_WriteChar((unsigned char *)displaystring + 4, 0, 0, 5);
  	LCD_GLASS_WriteChar((unsigned char *)displaystring + 5, 0, 0, 6);
}

void TIM4Init()
{
	TIM_TimeBaseInitTypeDef TIM_InitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1 ;
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStruct.TIM_Period = 32000;
	TIM_InitStruct.TIM_Prescaler = 1;
  	TIM_TimeBaseInit(TIM4, &TIM_InitStruct);
  	TIM_Cmd(TIM4, ENABLE);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_Pulse = 100;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM4, &TIM_OCInitStruct);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	
	UpdatePWM(frequency, duty);
}

void UpdatePWM(double frequency, double duty)
{
	//frequency in Hz, duty 0-100.0
	long int ratio;
	
	ratio = (long)SystemCoreClock/frequency;
	TIM4->PSC = (int)(ratio/0xFFFF);
    
	ratio = ((long)SystemCoreClock/(TIM4->PSC+1));
	TIM4->ARR = (int)(ratio/frequency)-1;
    
	TIM4->CCR2 = (int)((TIM4->ARR+1)*duty)/100-1;
}
