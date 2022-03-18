// Project OpenLight, "Head" device software
// STM32F100C4T6B MCU

#include "main.h"

#include "stm32f10x.h"

#include "stm32f10x_it.h"

#include <stdio.h>

__IO uint32_t HSEStatus = 0;

#define RFPOWERPORT GPIOA
#define RFPOWERPIN1 GPIO_Pin_8
#define RFPOWERPIN2 GPIO_Pin_9
#define RFPOWERPIN3 GPIO_Pin_10

#define BLINK_COUNT 3
#define BLINKSPEED 500

#define UART2SPEED 2400 // bit/s 

#define PUTCHARDELAY 0 // 1000
#define DELAYBETWEENMESSAGES 10000 // 20000

#define RIGHT_BUTTON_PORT GPIOA // Right Switch
#define LEFT_BUTTON_PORT GPIOA // Left Switch
#define ALARM_BUTTON_PORT GPIOA // Alarm & Freeze Button

#define RIGHT_BUTTON_PIN GPIO_Pin_3 // Right Switch
#define LEFT_BUTTON_PIN GPIO_Pin_4 // Left Switch
#define ALARM_BUTTON_PIN GPIO_Pin_5 // Alarm & Freeze Button

/* Message =	1 start byte 0xAA,
				3 bytes device number,
				1 bytes command,
				1 byte check summ
	Command =	0x0A for switch on Right Side light
				0x33 for switch on Left Side light
				0x09 for switch on Both Side light
				0x12 for control message
*/
const unsigned char RightSideOn[] = {
    0xAA,
    0xCC,
    0xCC,
    0xCC,
    0x0A,
    0xE8
};
const unsigned char LeftSideOn[] = {
    0xAA,
    0xCC,
    0xCC,
    0xCC,
    0x33,
    0xBF
};
const unsigned char BothSideOn[] = {
    0xAA,
    0xCC,
    0xCC,
    0xCC,
    0x09,
    0xE9
};
const unsigned char IamOnAir[] = {
    0xAA,
    0xCC,
    0xCC,
    0xCC,
    0x12,
    0xE0
};

void InitClock(void); // HSE Clock Init
void InitGPIO(void); // Init GPIO
void InitUART(unsigned int UartSpeed); // Init UART2. Before set HSE_VALUE in stm32f10x.h
void InitNVIC(void); // Init Interrupts
void InitTIM(void); // Init Timer
void Delay(unsigned int Val);
void FrontLEDsON(void);
void AllLEDsON(void);
void AllLEDsOFF(void);
void HelloGuys(void); // Start blinking
void SendMessage(const unsigned char Box[], int BoxSize); // Send Message on air
void RF_PowerOn(void); // Switch on RF module

unsigned char LRBtnState = 0; // Left/Right switch state
unsigned char AFBtnState = 0; // Alarm & Freeze Button state
unsigned char OldAFBtnState = 0; // Previous Alarm & Freeze Button state

void main(void) {
    InitClock(); // HSE Clock Init
    InitGPIO(); // Init GPIO
    InitUART(UART2SPEED); // Init UART2. Before set HSE_VALUE in stm32f10x.h

    for (int temp = 0; temp < BLINK_COUNT; temp++) // Blink a LEDs
        HelloGuys();

    FrontLEDsON();

    RF_PowerOn(); // Switch on RF module

    InitNVIC(); // Init Interrupts
    InitTIM(); // Init Timer
    TIM_Cmd(TIM2, DISABLE);

    while (1) {
        if (CurrentCommand != 0x03) // If not Alarm mode
        {
            CurrentCommand = 0x00;
            if ((LRBtnState = GPIO_ReadInputDataBit(RIGHT_BUTTON_PORT, RIGHT_BUTTON_PIN)) == 0)
                CurrentCommand = 0x01; // Switch on Right Side light
            else
            if ((LRBtnState = GPIO_ReadInputDataBit(LEFT_BUTTON_PORT, LEFT_BUTTON_PIN)) == 0)
                CurrentCommand = 0x02; // Switch on Left Side light
        }

        OldAFBtnState = AFBtnState;
        AFBtnState = GPIO_ReadInputDataBit(ALARM_BUTTON_PORT, ALARM_BUTTON_PIN);
        if ((AFBtnState == 0) && (OldAFBtnState == 1)) {
            if (CurrentCommand != 0x03)
                CurrentCommand = 0x03; // Switch on both Right Side and Left Side light (Alarm mode)
            else
            if (CurrentCommand == 0x03)
                CurrentCommand = 0x00; // Exit alarm mode
        }

        switch (CurrentCommand) {
        case 0x01: // Switch on Right Side light
        {
            SendMessage(RightSideOn, sizeof(RightSideOn));
            TIM_Cmd(TIM2, ENABLE);
            break;
        }
        case 0x02: // Switch on Left Side light
        {
            SendMessage(LeftSideOn, sizeof(LeftSideOn));
            TIM_Cmd(TIM2, ENABLE);
            break;
        }
        case 0x03: // Switch on both Right Side and Left Side light (Alarm mode)
        {
            SendMessage(BothSideOn, sizeof(BothSideOn));
            TIM_Cmd(TIM2, ENABLE);
            break;
        }
        default: {
            SendMessage(IamOnAir, sizeof(IamOnAir));
            break;
        }
        }
        Delay(DELAYBETWEENMESSAGES);
    }
} // main

void InitClock(void) // HSE Clock Init
{
    __IO uint32_t StartUpCounter = 0;

    RCC -> CR |= ((uint32_t) RCC_CR_HSEON); // Switch to HSE oscillator
    do // Wait until external HSE will be ready
    {
        HSEStatus = RCC -> CR & RCC_CR_HSERDY;
        StartUpCounter++;
    }
    while ((HSEStatus == 0) && (StartUpCounter != HSEStartUp_TimeOut));

    if ((RCC -> CR & RCC_CR_HSERDY) != RESET)
        HSEStatus = (uint32_t) 0x01; // HSE
    else
        HSEStatus = (uint32_t) 0x00; // HSI

    if (HSEStatus == (uint32_t) 0x01) // HSE (External quartz)
    {
        RCC -> CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
        RCC -> CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLXTPRE_PREDIV1_Div2 | RCC_CFGR_PLLMULL8);
    } else // HSI (internal RC)
    {
        RCC -> CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
        RCC -> CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL8);
        while (1) // do nothing (for debug purpose)
        {}
    }

    // Start PLL
    RCC -> CR |= RCC_CR_PLLON;
    // Wait until PLL is locked
    while ((RCC -> CR & RCC_CR_PLLRDY) == 0) {}
    // Select PLL as system clock source
    RCC -> CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_SW));
    RCC -> CFGR |= (uint32_t) RCC_CFGR_SW_PLL;
    // Wait till PLL is used as system clock source
    while ((RCC -> CFGR & (uint32_t) RCC_CFGR_SWS) != (uint32_t) 0x08) {}

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // PORTA clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // PORTB clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // Alternate function clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // UART2 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // Timer 2 clock enable
} // InitClock

void InitGPIO(void) // Init GPIO
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Set PA8, PA9 and PA10 as output (Transmitter Power)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // Out Push-pull
    GPIO_Init(GPIOA, & GPIO_InitStructure);

    // Configure USART2 TXD as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-pull
    GPIO_Init(GPIOA, & GPIO_InitStructure);

    // Set PA3, PA4 and PA5 as input (Keyboard)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Input Pull-up
    GPIO_Init(GPIOA, & GPIO_InitStructure);

    // Set PB12, PB13 and PB14 as output (Right side LED, Front LEDs, Left side LED)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // Output Push-pull
    GPIO_Init(GPIOB, & GPIO_InitStructure);

    // Set TXD USART2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-pull
    GPIO_Init(GPIOA, & GPIO_InitStructure);
} // InitGPIO

void Delay(unsigned int Val) {
    for (; Val != 0; Val--)
        __NOP();
} // Delay

int putchar(int ch) {
    USART2 -> DR = (uint8_t) ch;

    // Loop until the end of transmission
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {}
    Delay(PUTCHARDELAY);

    return ch;
} // putchar

void FrontLEDsON(void) {
    GPIO_SetBits(FRONT_LED_PORT, FRONT_LED_PIN); // Front LED On
} // FrontLEDsON

void AllLEDsON(void) {
    GPIO_SetBits(FRONT_LED_PORT, FRONT_LED_PIN); // Front LED On
    GPIO_ResetBits(RIGHT_SIDE_LED_PORT, RIGHT_SIDE_LED_PIN); // Right side LED On
    GPIO_ResetBits(LEFT_SIDE_LED_PORT, LEFT_SIDE_LED_PIN); // Left side LED On
} // AllLEDsON

void AllLEDsOFF(void) {
    GPIO_ResetBits(FRONT_LED_PORT, FRONT_LED_PIN); // Front LED Off
    GPIO_SetBits(RIGHT_SIDE_LED_PORT, RIGHT_SIDE_LED_PIN); // Right side LED Off
    GPIO_SetBits(LEFT_SIDE_LED_PORT, LEFT_SIDE_LED_PIN); // Left side LED Off
} // AllLEDsOFF

void HelloGuys(void) // Start blinking
{
    unsigned int H1_Bright = 0;
    unsigned int PWM_Cycle = 0;

    for (H1_Bright = BLINKSPEED; H1_Bright > 0; H1_Bright--) // Blink loop
        for (PWM_Cycle = 0; PWM_Cycle < BLINKSPEED; PWM_Cycle++) // PWM loop
            if (PWM_Cycle >= H1_Bright)
                AllLEDsON();
            else
                AllLEDsOFF();

    for (H1_Bright = 0; H1_Bright < BLINKSPEED; H1_Bright++) // Blink loop
        for (PWM_Cycle = 0; PWM_Cycle < BLINKSPEED; PWM_Cycle++) // PWM loop
            if (PWM_Cycle >= H1_Bright)
                AllLEDsON();
            else
                AllLEDsOFF();

    AllLEDsOFF();
} // HelloGuys()

void InitUART(unsigned int UartSpeed) {
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = UartSpeed;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;

    USART_Init(USART2, & USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
} // InitUART

void SendMessage(const unsigned char Box[], int BoxSize) // Send Message into air
{
    int count = 0;

    for (count = 0; count < BoxSize; count++)
        putchar(Box[count]);
} // SendMessage

void RF_PowerOn(void) // Switch on RF module
{
    GPIO_SetBits(RFPOWERPORT, RFPOWERPIN1 | RFPOWERPIN2 | RFPOWERPIN3);
} // RF_PowerOn

void InitNVIC(void) // Init Interrupts
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; // Enable the TIM2 Interrupt
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( & NVIC_InitStructure);
} // InitNVIC

void InitTIM(void) // Init Timer
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructure.TIM_Period = 13000;
    TIM_TimeBaseStructure.TIM_Prescaler = 999;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, & TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
} // InitTIM
