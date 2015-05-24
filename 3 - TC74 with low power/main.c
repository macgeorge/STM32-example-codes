/**
  ******************************************************************************
  * @brief   TC74 temperature sensor example code on STM32 with stanby mode
  * @date    May 2015
  * @version 1.0
  * @author  George Christidis
  ******************************************************************************
  * @details
			This program reads the temperature of a TC74 connected to an STM32L152
			discovery board. The sensor is connected on the I2C1 pins (SCL: PB6,
			SDA: PB7). It returns the value to a variable named temperature which then
			can be sent via UART, shown on a display or through a debug session. The
			temperature is sampled every 5 seconds, via TIM6. After that the TC74
			enters standby mode until the next read.
	******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/

#include "stm32l1xx_usart.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_tim.h"
#include "stm32l1xx_i2c.h"

#define TC74_ADDRESS 0x9A	// TC74	A5 address

void TC74_Config(void), TIM6_Config(void);
int	TC74_Read_Register(uint8_t, uint8_t), TC74_Sleep(uint8_t), TC74_WakeUp(uint8_t);
int I2C_start(I2C_TypeDef *, uint8_t, uint8_t), I2C_write(I2C_TypeDef *, uint8_t), I2C_stop(I2C_TypeDef *);
int8_t I2C_read_ack(I2C_TypeDef *), I2C_read_nack(I2C_TypeDef *);
int temperature;
	
typedef enum {TC74_SLEEPING, TC74_WAITING} TC74_statetypedef;
TC74_statetypedef Temp_read_status=TC74_SLEEPING;

int main()
{
	TC74_Config();
	TIM6_Config();
	
	while(1);	
}

void TC74_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 10000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_SMBusHost;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}

int TC74_Read_Register(uint8_t TC74address, uint8_t reg)
{
	int8_t data1, data2;

	if(I2C_start(I2C1, TC74address, I2C_Direction_Transmitter)>0) {
		return -127;
	}
	if(I2C_write(I2C1, reg)>0) return -127;
	if(I2C_stop(I2C1)==1) return -127;

	if(I2C_start(I2C1, TC74address, I2C_Direction_Receiver)>0) {
		return -127;
	}
	data1 = I2C_read_ack(I2C1);
	data2 = I2C_read_nack(I2C1);

	return data1;
}

int TC74_WakeUp(uint8_t TC74address)
{
	if(I2C_start(I2C1, TC74address, I2C_Direction_Transmitter)>0) {
	return -127;
	}
	if(I2C_write(I2C1, 0x01)>0) return -127;
	if(I2C_write(I2C1, 0x00)>0) return -127;
	if(I2C_stop(I2C1)==1) return -127;
	return 0;
}

int TC74_Sleep(uint8_t TC74address)
{
	if(I2C_start(I2C1, TC74address, I2C_Direction_Transmitter)>0) {
	return -127;
	}
	if(I2C_write(I2C1, 0x01)>0) return -127;
	if(I2C_write(I2C1, 0x80)>0) return -127;
	if(I2C_stop(I2C1)==1) return -127;
	return 0;
}

int I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_TIMEOUT)!=RESET) {
			I2C_ClearFlag(I2Cx, I2C_FLAG_TIMEOUT);
			return 1;
		}
		if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF)!=RESET) {
			I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
			return 2;
		}
	}
	
	I2C_Send7bitAddress(I2Cx, address, direction);
	
	if (direction== I2C_Direction_Transmitter) {
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
			if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TIMEOUT)!=RESET) {
				I2C_ClearFlag(I2Cx, I2C_FLAG_TIMEOUT);
				return 1;
		}
			if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF)!=RESET) {
				I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
				return 2;
			}
		}
	}
	
	else if(direction == I2C_Direction_Receiver) {
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
			if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TIMEOUT)!=RESET) {
				I2C_ClearFlag(I2Cx, I2C_FLAG_TIMEOUT);
				return 3;
			}
			if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF)!=RESET) {
				I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
				return 4;
			}
		}
	}
	return 0;
}

int I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TIMEOUT)!=RESET) {
			I2C_ClearFlag(I2Cx, I2C_FLAG_TIMEOUT);
			return 1;
		}
		if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF)!=RESET) {
			I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
			return 2;
		}
	}
	return 0;
}

int I2C_stop(I2C_TypeDef* I2Cx)
{
	I2C_GenerateSTOP(I2Cx, ENABLE);
	return 0;
}

int8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{
	int8_t data;
	
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)){
		if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TIMEOUT)!=RESET) {
			I2C_ClearFlag(I2Cx, I2C_FLAG_TIMEOUT);
			return -127;
		}
		if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF)!=RESET) {
			I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
			return -127;
		}
	}
	data=I2C_ReceiveData(I2Cx);
	
	return data;
}

int8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
	uint8_t data;
	
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)){
		if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TIMEOUT)!=RESET) {
			I2C_ClearFlag(I2Cx, I2C_FLAG_TIMEOUT);
			return -127;
		}
	}
	data=I2C_ReceiveData(I2Cx);
	
	return data;
}

void TIM6_Config(void)
{
	TIM_TimeBaseInitTypeDef timerInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	timerInitStructure.TIM_Prescaler=1000;
	timerInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	timerInitStructure.TIM_Period=32000;
	timerInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6, &timerInitStructure);
	TIM_Cmd(TIM6, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
}

void TIM6_IRQHandler(void)
{
	if (TIM_GetFlagStatus(TIM6, TIM_IT_Update)!=RESET) {
			TIM_Cmd(TIM6, DISABLE);

		if (Temp_read_status==TC74_SLEEPING) {

			TC74_WakeUp(TC74_ADDRESS);
			TIM6->PSC = 5000;
			Temp_read_status=TC74_WAITING;
		}

		else {
			temperature=TC74_Read_Register(TC74_ADDRESS,0x00);
			TC74_Sleep(TC74_ADDRESS);

			TIM6->PSC = 500;
			Temp_read_status=TC74_SLEEPING;
		}

		TIM6->CNT=0;
		TIM_Cmd(TIM6, ENABLE);
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}
}