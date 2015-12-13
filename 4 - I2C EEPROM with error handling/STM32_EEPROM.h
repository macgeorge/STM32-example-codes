#ifndef __STM32_EEPROM_H
#define __STM32_EEPROM_H
#endif

#define I2C_TIMEOUT_MAX 100000

#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

void I2C_Memory_Init(void);

int I2C_Memory_Read(I2C_TypeDef*, uint8_t);

int I2C_Memory_Write(I2C_TypeDef*, uint8_t, uint8_t);
