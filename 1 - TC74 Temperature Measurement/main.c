/**
  ******************************************************************************
  * @brief   TC74 temperature sensor simple example code on STM32
  * @date    Jan 2015
  * @version 1.0
  * @author  George Christidis
  ******************************************************************************
  * @details
			This program reads the temperature of a TC74 connected to an STM32L152
			discovery board. The sensor is connected on the I2C1 pins (SCL: PB6,
			SDA: PB7). It returns the value to a variable named temperature which then
			can be sent via UART, shown on a display or through a debug session.
	******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/

#include "stm32l1xx_usart.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_tim.h"
#include "stm32l1xx_i2c.h"

#define TC74_ADDRESS 0x9A	// TC74	A5 address
#define I2CTIMEOUT 50000

void TC74_Config(void);
int	TC74_Read_Temperature(uint8_t);
void I2C_start(I2C_TypeDef *, uint8_t, uint8_t), I2C_write(I2C_TypeDef *, uint8_t), I2C_stop(I2C_TypeDef *);
int8_t I2C_read_ack(I2C_TypeDef *), I2C_read_nack(I2C_TypeDef *);

int main()
{
	int temperature;
	
	TC74_Config();
	temperature=TC74_Read_Temperature(TC74_ADDRESS);
	
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

int TC74_Read_Temperature(uint8_t TC74address)
{
	int8_t data1, data2;
	
	I2C_start(I2C1, TC74address, I2C_Direction_Transmitter);
	I2C_write(I2C1,0x00);
	I2C_stop(I2C1); 
	I2C_start(I2C1, TC74address, I2C_Direction_Receiver);
	data1 = I2C_read_ack(I2C1);
	data2 = I2C_read_nack(I2C1);
	return data1;
}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
 while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
 I2C_GenerateSTART(I2Cx, ENABLE);
 while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) ;
 I2C_Send7bitAddress(I2Cx, address, direction);
 if (direction== I2C_Direction_Transmitter) {
 while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
 }
 else if(direction == I2C_Direction_Receiver) {
 while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
 }
}

void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
 I2C_SendData(I2Cx, data);
 while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
void I2C_stop(I2C_TypeDef* I2Cx)
{
 I2C_GenerateSTOP(I2Cx, ENABLE);
}

int8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{
 int8_t data;
 
 I2C_AcknowledgeConfig(I2Cx, ENABLE);
 while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
 data=I2C_ReceiveData(I2Cx);
 
 return data;
}

int8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
 uint8_t data;
 
 I2C_AcknowledgeConfig(I2Cx, DISABLE);
 I2C_GenerateSTOP(I2Cx, ENABLE);
 while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
 data=I2C_ReceiveData(I2Cx);
 
 return data;
}
