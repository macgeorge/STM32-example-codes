/**
  ********************************************************************************
  * @brief   STM32 F429 Discovery Dot Matrix Controller
  * @date    Dec 2016
  * @version 1.0
  * @author  George Christidis
  ********************************************************************************
  * @details
			This example shows how to drive an 64x32 Dot Matrix Display using
			an STM32F429 Discovery board. It uses UART to get new frames and stores
			them in two different memory banks.
	******************************************************************************
	*/
 

#include "stm32f4xx.h"
#include <stdio.h>

#include "defines.h"
#include "attributes.h"
#include "stm32f4xx_usart.h"
#include "tm_stm32f4_delay.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

uint8_t rowcounter=0;
uint8_t colcounter=0;
int8_t pwmindex=0;
uint8_t delaybrightness=100-BRIGHTNESS;
uint8_t brightvalue=50; //if needed to dim display even more
uint8_t EN=1;
volatile uint8_t *readmemory, *writememory;
volatile uint8_t mem1[ROWS*COLUMNS*3], mem2[ROWS*COLUMNS*3];
void InitMatrixPorts(void), InitUARTDMA(void), InitDiscoButton(void);

int main(void) {
	int i;
	extern uint8_t initimage[];
	
	SystemInit();
	TM_DELAY_Init();
	
	//Memory Banks//
	readmemory = mem1;
	writememory = mem2;
	
	InitDiscoButton();
	InitMatrixPorts();
	InitUARTDMA();

	for (i=0; i<6144; i++) {
		mem1[i] = initimage[i];
	}
		
	while(1) {
		if (EN==1)
		{
			for (pwmindex = 7; pwmindex>=(7-(COLDEPTH-1)); pwmindex--){ 
				for (rowcounter=0 ; rowcounter<(ROWS/2) ; rowcounter++) {
					GPIO_SetBits(OEPORT,OEPIN);
					for (colcounter=0 ; colcounter<COLUMNS ; colcounter++) {
						GPIO_ResetBits(CLKPORT,CLKPIN);
						if (((readmemory[rowcounter*COLUMNS*3+colcounter*3+0] >> pwmindex) & 0x01)==1) GPIO_SetBits(R1PORT,R1PIN);
						else GPIO_ResetBits(R1PORT,R1PIN);
						if (((readmemory[rowcounter*COLUMNS*3+colcounter*3+1] >> pwmindex) & 0x01)==1) GPIO_SetBits(G1PORT,G1PIN);
						else GPIO_ResetBits(G1PORT,G1PIN);
						if (((readmemory[rowcounter*COLUMNS*3+colcounter*3+2] >> pwmindex) & 0x01)==1) GPIO_SetBits(B1PORT,B1PIN);
						else GPIO_ResetBits(B1PORT,B1PIN);
						if (((readmemory[(ROWS*COLUMNS/2)*3+rowcounter*COLUMNS*3+colcounter*3+0] >> pwmindex) & 0x01)==1) GPIO_SetBits(R2PORT,R2PIN);
						else GPIO_ResetBits(R2PORT,R2PIN);
						if (((readmemory[(ROWS*COLUMNS/2)*3+rowcounter*COLUMNS*3+colcounter*3+1] >> pwmindex) & 0x01)==1) GPIO_SetBits(G2PORT,G2PIN);
						else GPIO_ResetBits(G2PORT,G2PIN);
						if (((readmemory[(ROWS*COLUMNS/2)*3+rowcounter*COLUMNS*3+colcounter*3+2] >> pwmindex) & 0x01)==1) GPIO_SetBits(B2PORT,B2PIN);
						else GPIO_ResetBits(B2PORT,B2PIN);
						GPIO_SetBits(CLKPORT,CLKPIN);
						if (colcounter>brightvalue) GPIO_ResetBits(OEPORT,OEPIN);
					}
					
					GPIO_SetBits(OEPORT,OEPIN);
					Delay(1);
					GPIO_SetBits(LEPORT,LEPIN);
					if ((rowcounter & 0x01)==0x01) GPIO_SetBits(ROW0PORT,ROW0PIN); else GPIO_ResetBits(ROW0PORT,ROW0PIN);
					if ((rowcounter & 0x02)==0x02) GPIO_SetBits(ROW1PORT,ROW1PIN); else GPIO_ResetBits(ROW1PORT,ROW1PIN);
					if ((rowcounter & 0x04)==0x04) GPIO_SetBits(ROW2PORT,ROW2PIN); else GPIO_ResetBits(ROW2PORT,ROW2PIN);
					if ((rowcounter & 0x08)==0x08) GPIO_SetBits(ROW3PORT,ROW3PIN); else GPIO_ResetBits(ROW3PORT,ROW3PIN);
					GPIO_ResetBits(LEPORT,LEPIN);
					Delay(delaybrightness);
					Delay(1);
					GPIO_ResetBits(OEPORT,OEPIN);
					Delay(59*(pwmindex-(8-COLDEPTH)+1)*(pwmindex-(8-COLDEPTH)+1)-59);
				}
			}
		}
		else GPIO_SetBits(OEPORT,OEPIN);
	}
}

void EXTI0_IRQHandler() {
	if (EXTI_GetITStatus(EXTI_Line0) == SET){
		if (EN==1) EN=0;
		else EN=1;
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void DMA2_Stream5_IRQHandler(void)
{
	volatile uint8_t *temppointer;
	if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5) == SET)
	{
		temppointer = readmemory;
		readmemory = writememory;
		writememory = temppointer;
		DMA_Cmd(DMA2_Stream5, DISABLE);
		DMA2_Stream5->M0AR = (uint32_t)writememory;
		DMA_Cmd(DMA2_Stream5, ENABLE);
    	DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
    }
}

void InitMatrixPorts() {
	GPIO_InitTypeDef      GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOG, ENABLE);
	
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_Pin = R1PIN;
  	GPIO_Init(R1PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = G1PIN;
  	GPIO_Init(G1PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = B1PIN;
  	GPIO_Init(B1PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = R2PIN;
  	GPIO_Init(R2PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = G2PIN;
  	GPIO_Init(G2PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ROW3PIN;
  	GPIO_Init(ROW3PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ROW2PIN;
  	GPIO_Init(ROW2PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ROW1PIN;
  	GPIO_Init(ROW1PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ROW0PIN;
  	GPIO_Init(ROW0PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = B2PIN;
  	GPIO_Init(B2PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = OEPIN;
  	GPIO_Init(OEPORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = LEPIN;
  	GPIO_Init(LEPORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = CLKPIN;
  	GPIO_Init(CLKPORT, &GPIO_InitStructure);
}

void InitUARTDMA() {
	
	//UART 1 PB7 RX
	//DMA2 Channel 4 Stream 5
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7, GPIO_AF_USART1);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
  	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)writememory;
  	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  	DMA_InitStructure.DMA_BufferSize = ROWS*COLUMNS*3;
  	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  	DMA_Init(DMA2_Stream5, &DMA_InitStructure);
  	DMA_Cmd(DMA2_Stream5, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
}

void InitDiscoButton(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
