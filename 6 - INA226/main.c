/**
  ********************************************************************************
  * @brief   I2C INA226 library example
  * @date    Feb 2016
  * @version 1.0
  * @author  George Christidis
  ********************************************************************************
  * @details
			This program uses the I2C INA226 library provided to configure the 
			monitor and read the bus voltag via I2C. It uses I2C1 (PB8 for SCL,
			PB9 for	SDA).
	******************************************************************************
	*/

#include "stm32fxxx_hal.h"

#include "F7_INA226.h"

void InitI2C1(void), InitGPIO(void);
I2C_HandleTypeDef I2c1Handle;

int main(){
	HAL_Init();
	InitI2C1();
	InitGPIO();
	
	INA226_setConfig(&I2c1Handle, INA226_ADDRESS, INA226_MODE_CONT_SHUNT_AND_BUS | INA226_VBUS_140uS | INA226_VBUS_140uS | INA226_AVG_1024);
	
	while(1) {
		if (INA226_getBusV(&I2c1Handle, INA226_ADDRESS) > 1000.0F) HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_Delay (100);
	}
}

void InitGPIO(void) {
	GPIO_InitTypeDef   GPIO_InitStruct;
 
	__HAL_RCC_GPIOI_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);	
}

void InitI2C1(void)
{
	I2c1Handle.Instance = I2C1;
	I2c1Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2c1Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2c1Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2c1Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	I2c1Handle.Init.OwnAddress1 = 0x00;
	I2c1Handle.Init.Timing = 0x80200F73;
	if(HAL_I2C_Init(&I2c1Handle)!= HAL_OK)
	
	HAL_I2CEx_AnalogFilter_Config(&I2c1Handle, I2C_ANALOGFILTER_ENABLED);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	if(hi2c->Instance==I2C1) {
		__HAL_RCC_I2C1_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		
		GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
}
