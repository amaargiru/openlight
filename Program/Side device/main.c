// Project OpenLight, "Side" device software
// STM32F100C4T6B MCU
// For new releases please visit openboardfab.com
// Tab = 3

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include <stdio.h>

__IO uint32_t HSEStatus = 0;

#define RFPOWERPORT				GPIOA
#define RFPOWERPIN1				GPIO_Pin_9
#define RFPOWERPIN2				GPIO_Pin_10
#define RFPOWERPIN3				GPIO_Pin_11
#define RFPOWERPIN4				GPIO_Pin_12

#define BLINK_COUNT	3
#define BLINKSPEED	500
#define BLSPEED		600

#define UART2SPEED	2400

#define POSLIGHT		3000
#define NEGLIGHT		3000

void InitClock(void);	// HSE Clock Init
void InitGPIO(void);		// Init GPIO
void InitUART(unsigned int UartSpeed);		// Init UART2
void InitNVIC(void);		// Init Interrupts
void Delay(unsigned int Val);
void RF_PowerOn(void);	// Switch on RF module
void AllLEDsON(void);
void AllLEDsOFF(void);
void HelloGuys(void);	// Start blinking
void Rise(void);			// Increasing the brightness
void Fall(void);			// Reducing the brightness

unsigned int temp = 0;

void main(void)
{
	InitClock();				// HSE Clock Init
	InitGPIO();					// Init GPIO
	InitUART(UART2SPEED);	// Init UART2. Before set HSE_VALUE in stm32f10x.h
	InitNVIC();					// Init Interrupts
	
	RF_PowerOn();				// Switch on RF module
	
	for(temp = 0; temp < BLINK_COUNT; temp++)	// Blink a LEDs
		HelloGuys();
		
	while (1)
	{
		if (BFResult > 0)
		{
			Rise();
			Delay(1400000);
			Fall();
			Delay(600000);
			USART_Cmd(USART2, ENABLE);
		}
		else
		{
			GPIO_SetBits(GPIOC, GPIO_Pin_14);
			Delay(POSLIGHT);
			GPIO_ResetBits(GPIOC, GPIO_Pin_14);
			Delay(NEGLIGHT);
		}
	}
}

void InitClock(void)	// HSE Clock Init
{
	__IO uint32_t StartUpCounter = 0;

	RCC->CR |= ((uint32_t)RCC_CR_HSEON);	// Switch to HSE oscillator
	do		// Wait until external HSE will be ready
	{
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;  
	} 
	while((HSEStatus == 0) && (StartUpCounter != HSEStartUp_TimeOut));
	
	if((RCC->CR & RCC_CR_HSERDY) != RESET)
		HSEStatus = (uint32_t)0x01;		// HSE
	else
		HSEStatus = (uint32_t)0x00;		// HSI
	
	if (HSEStatus == (uint32_t)0x01)	// HSE (External quartz)
	{											// PLL configuration
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
		RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLXTPRE_PREDIV1_Div2 | RCC_CFGR_PLLMULL8);
	}
	else 										// HSI (internal RC)
	{											// PLL configuration
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
		RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL8);
		while (1)							// do nothing (for debug purpose)
			{
			}
	}

	// Start PLL
	RCC->CR |= RCC_CR_PLLON;
	// Wait until PLL is locked
	while((RCC->CR & RCC_CR_PLLRDY) == 0) {}
	// Select PLL as system clock source
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
	// Wait till PLL is used as system clock source
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08) {}
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	// PORTA clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	// PORTB clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	// PORTC clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	// Alternate function clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);// UART2 clock enable
} // InitClock

void InitGPIO(void)	// Init GPIO
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// Set PA9, PA10, PA11 and PA12 as output (Transmitter Power)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Configure USART2 RXD as input floating
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Configure USART2 TXD as alternate function push-pull
	// For debug purpose only
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Set PC14 as output (EN_PWM)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
} // InitGPIO

void Delay(unsigned int Val)
{
	for(; Val != 0; Val--)
		__NOP();
} // Delay

void InitUART(unsigned int UartSpeed)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = UartSpeed;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	// Enable RXD Interrupt
	USART_Cmd(USART2, ENABLE);
} // InitUART

void RF_PowerOn(void)	// Switch on RF module
{
	GPIO_SetBits(RFPOWERPORT, RFPOWERPIN1 | RFPOWERPIN2 | RFPOWERPIN3 | RFPOWERPIN4);
} // RF_PowerOn

void InitNVIC(void)		// Init Interrupts
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// Enable the USART2 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
} // InitNVIC

void AllLEDsON(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_14);
	GPIO_SetBits(GPIOC, GPIO_Pin_14);
	GPIO_SetBits(GPIOC, GPIO_Pin_14);
} // AllLEDsON

void AllLEDsOFF(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_14);
	GPIO_ResetBits(GPIOC, GPIO_Pin_14);
	GPIO_ResetBits(GPIOC, GPIO_Pin_14);
} // AllLEDsOFF

void HelloGuys(void)		// Start blinking
{  
	unsigned int H1_Bright = 0;
   unsigned int PWM_Cycle = 0;
   
	for (H1_Bright = BLINKSPEED; H1_Bright > 0; H1_Bright--)		// Blink loop
		for (PWM_Cycle = 0; PWM_Cycle < BLINKSPEED; PWM_Cycle++)	// PWM loop
			if(PWM_Cycle >= H1_Bright)
        		AllLEDsON();
			else
         	AllLEDsOFF();

   for (H1_Bright = 0; H1_Bright < BLINKSPEED; H1_Bright++)		// Blink loop
		for (PWM_Cycle = 0; PWM_Cycle < BLINKSPEED; PWM_Cycle++)	// PWM loop
			if(PWM_Cycle >= H1_Bright)
        		AllLEDsON();
         else
        		AllLEDsOFF();

	AllLEDsOFF();
}// HelloGuys()

void Rise(void)	// Increasing the brightness
{  
	unsigned int H1_Bright = 0;
   unsigned int PWM_Cycle = 0;
   
	for (H1_Bright = BLSPEED; H1_Bright > 0; H1_Bright--)		// Blink loop
		for (PWM_Cycle = 0; PWM_Cycle < BLSPEED; PWM_Cycle++)	// PWM loop
			if(PWM_Cycle >= H1_Bright)
				GPIO_SetBits(GPIOC, GPIO_Pin_14);
			else
				GPIO_ResetBits(GPIOC, GPIO_Pin_14);
}// Rise()

void Fall(void)	// Reducing the brightness
{  
	unsigned int H1_Bright = 0;
   unsigned int PWM_Cycle = 0;

   for (H1_Bright = 0; H1_Bright < BLSPEED; H1_Bright++)		// Blink loop
		for (PWM_Cycle = 0; PWM_Cycle < BLSPEED; PWM_Cycle++)	// PWM loop
			if(PWM_Cycle >= H1_Bright)
				GPIO_SetBits(GPIOC, GPIO_Pin_14);
			else
				GPIO_ResetBits(GPIOC, GPIO_Pin_14);
	GPIO_ResetBits(GPIOC, GPIO_Pin_14);
}// Fall()