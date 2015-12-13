/**
  ********************************************************************************
  * @brief   I2C EEPROM library example
  * @date    Nov 2015
  * @version 1.0
  * @author  George Christidis
  ********************************************************************************
  * @details
			This program uses the I2C EEPROM library provided to read and write
			data to an EEPROM chip via I2C. It uses I2C3 (PA8 for SCL, PC9 for
			SDA) with a 24LC01B chip.
	******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/

#include "STM32_EEPROM.h"


int main()
{
	uint8_t data=0xAA;
	
	I2C_Memory_Init();
		
	I2C_Memory_Write(I2C3, 0x00, data);
	
	I2C_Memory_Read(I2C3, 0x00);
	
	while(1);	
}
