# STM32-example-codes
# Libraries and examples
This project contains various example codes and custom libraries created for STM32 micronctrollers (L1, F4 and F7 variants).
The L1 and F4 examples use the Standard Peripheral Library, whereas the F7 examples use the Hardware Abstraction Layer (HAL) drivers.

1. TC74 Temperature Measurement<BR>
      This program reads the temperature of a TC74 connected to an STM32L152
			discovery board. The sensor is connected on the I2C1 pins (SCL: PB6,
			SDA: PB7). It returns the value to a variable named temperature which then
			can be sent via UART, shown on a display or through a debug session.
      
2. TC74 with error handling<BR>
			This program reads the temperature of a TC74 connected to an STM32L152
			discovery board. The sensor is connected on the I2C1 pins (SCL: PB6,
			SDA: PB7). It returns the value to a variable named temperature which then
			can be sent via UART, shown on a display or through a debug session. In
			case of a timeout or an acknowledge failure, the program returns the error
			code and continues normal execution.
      
3. TC74 with low power<BR>
      This program reads the temperature of a TC74 connected to an STM32L152
			discovery board. The sensor is connected on the I2C1 pins (SCL: PB6,
			SDA: PB7). It returns the value to a variable named temperature which then
			can be sent via UART, shown on a display or through a debug session. The
			temperature is sampled every 5 seconds, via TIM6. After that the TC74
			enters standby mode until the next read.
      
4. I2C EEPROM with error handling<BR>
      Library and example in order to read and write data to an EEPROM chip
			via I2C. It uses I2C3 (PA8 for SCL, PC9 for SDA) with a 24LC01B chip.

5. SPI Potentiometer<BR>
      Library and example to operate a Microchip MCP41XXX potentiometer.

6. INA226<BR>
      Library and example for the I2C INA226 in order to configure the 
			monitor and read the bus voltage via I2C. It uses I2C1 (PB8 for SCL,
			PB9 for	SDA).
      
7. L152 Disco PWM Generator<BR>
      This program output a PWM signal to PB7, with variable frequency and
			duty cycle. The values are set using the Discovery's touch sensor and
			are displayed on the embedded LCD. The user button is pressed to
			change between setting the frequency and the duty cycle.

8. F4 Dot Matrix Controller<BR>
      This example shows how to drive an 64x32 Dot Matrix Display using
			an STM32F429 Discovery board. It uses UART to get new frames and stores
			them in two different memory banks.
