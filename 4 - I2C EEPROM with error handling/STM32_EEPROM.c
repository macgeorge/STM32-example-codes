/**
  ********************************************************************************
  * @brief   STM32 Library for I2C EEPROM memory
  * @date    Nov 2015
  * @version 1.0
  * @author  George Christidis
  ********************************************************************************
  * @details
			This library contains the necessary functions to initialize, read and
			write data to an EEPROM using the I2C protocol. As an example, the
			code was used on an STM32F429 using the I2C3 peripheral (PA8 for SCL,
			PC9 for SDA) with a 24LC01B chip.
	******************************************************************************
	*/
	
#include "STM32_EEPROM.h"

void I2C_Memory_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3); //SCL
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3); //SDA
	
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
  	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
  	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  	I2C_InitStruct.I2C_ClockSpeed = 100000;
	I2C_DeInit(I2C3);
	I2C_Init(I2C3, &I2C_InitStruct);
	I2C_Cmd(I2C3, ENABLE);	
}

int I2C_Memory_Read(I2C_TypeDef* I2Cx, uint8_t address)
{
	uint32_t timeout = I2C_TIMEOUT_MAX;
  	uint8_t Data = 0;
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	timeout = I2C_TIMEOUT_MAX;
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  	{
		if ((timeout--) == 0) return -1;
	}
	I2C_Send7bitAddress(I2Cx, 0xA0, I2C_Direction_Transmitter);
	
	timeout = I2C_TIMEOUT_MAX;
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  	{
		if ((timeout--) == 0) return -1;
  	} 
	
	I2C_SendData(I2Cx, address);

  	timeout = I2C_TIMEOUT_MAX;
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if ((timeout--) == 0) return -1;
  	}
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	timeout = I2C_TIMEOUT_MAX;
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  	{
		if ((timeout--) == 0) return -1;
	}
	I2C_Send7bitAddress(I2Cx, 0xA0, I2C_Direction_Receiver);
	
	timeout = I2C_TIMEOUT_MAX;
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  	{
		if ((timeout--) == 0) return -1;
  	}
	
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	
	timeout = I2C_TIMEOUT_MAX;
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
  	{
		if ((timeout--) == 0) return -1;
  	}
 
	I2C_GenerateSTOP(I2Cx, ENABLE);
  	Data = I2C_ReceiveData(I2Cx);
	
	return Data;	
}

int I2C_Memory_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t data)
{
	uint32_t timeout = I2C_TIMEOUT_MAX;
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	timeout = I2C_TIMEOUT_MAX;
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
 	{
		if ((timeout--) == 0) return -1;
	}
	I2C_Send7bitAddress(I2Cx, 0xA0, I2C_Direction_Transmitter);
	
	timeout = I2C_TIMEOUT_MAX;
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  	{
		if ((timeout--) == 0) return -1;
  	} 
	
	I2C_SendData(I2Cx, address);

  	timeout = I2C_TIMEOUT_MAX;
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
			if ((timeout--) == 0) return -1;
  	}
	
	I2C_SendData(I2Cx, data);

  	timeout = I2C_TIMEOUT_MAX;
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
			if ((timeout--) == 0) return -1;
  	}
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	return 0;	
}
