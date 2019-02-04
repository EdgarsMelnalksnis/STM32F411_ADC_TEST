/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "string.h"
#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

#define ITERATIONS 100 //ADC readings before sending to PC

void init_RCC(void);
void init_Timer10(void);
void init_GPIOB(void);
void init_GPIOC(void);
void init_GPIOA(void);
void init_Interrupts(void);
void init_DMA2(void);
void init_ADC1(void);
void init_USART2(uint32_t);

volatile uint16_t adc1DmaWMem[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //make new file for global variables
uint16_t adc_data[ITERATIONS];

static inline void testSpeedManualCopy(void) {
    for (uint16_t i = 0; i <= ITERATIONS - 10; i += 10) {
	adc_data[i] = adc1DmaWMem[0];
	adc_data[i + 1] = adc1DmaWMem[0];
	adc_data[i + 2] = adc1DmaWMem[0];
	adc_data[i + 3] = adc1DmaWMem[0];
	adc_data[i + 4] = adc1DmaWMem[0];
	adc_data[i + 5] = adc1DmaWMem[0];
	adc_data[i + 6] = adc1DmaWMem[0];
	adc_data[i + 7] = adc1DmaWMem[0];
	adc_data[i + 8] = adc1DmaWMem[0];
	adc_data[i + 9] = adc1DmaWMem[0];

    }
}

static inline void testSpeedLoopCopy(void) {
    for (uint16_t i = 0; i <= ITERATIONS - 10; i += 10) {
	for(uint16_t j = i; j < i + 10; ++j) {
	    adc_data[j] = adc1DmaWMem[0];
	}
    }
}

int main(void) {
    init_RCC();
    init_Timer10();
    init_GPIOB();
    init_GPIOA();
    init_GPIOC();
    //init_Interrupts();
    init_ADC1();
    init_DMA2();
    init_USART2(115200);

    (*((int *) (ADC1_BASE + 0x8u))) |= 1 << 30; //start ADC

    uint16_t i = 0;
    volatile uint16_t Tim10_counter = 0;

    while (1) {

	TIM_Cmd(TIM10, ENABLE);
	TIM_SetCounter(TIM10, 0);

	testSpeedManualCopy();

	Tim10_counter = TIM_GetCounter(TIM10);
	TIM_Cmd(TIM10, DISABLE);

	for (i = 0; i < ITERATIONS; i++) {
	    printf("%i %i\n", i, adc1DmaWMem[0]);
	}
	printf("Manual copy: Counter:%i \nIterations:%i\nCounter/Iterations=%i",
		Tim10_counter, i, Tim10_counter / i);

	TIM_Cmd(TIM10, ENABLE);
	TIM_SetCounter(TIM10, 0);

	testSpeedLoopCopy();

	Tim10_counter = TIM_GetCounter(TIM10);
	TIM_Cmd(TIM10, DISABLE);

	for (i = 0; i < ITERATIONS; i++) {
	    printf("%i %i\n", i, adc1DmaWMem[0]);
	}
	printf("Loop  copy:  Counter:%i \nIterations:%i\nCounter/Iterations=%i",
		Tim10_counter, i, Tim10_counter / i);
    }
}

void init_RCC(void) {
    RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_1);
    RCC_PLLConfig(RCC_PLLSource_HSI, 8, 100, 2, 4);	//16Mhz/8 *100 /2
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    //RCC_PCLK2Config(RCC_HCLK_Div2);	//APB2 CLK / 2 (ADC_CLOCK)

}

void init_Timer10(void) {

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

}

void init_GPIOB(void) {
//configure GPIOB
    GPIO_InitTypeDef GPIO_InitDef; //structure

//enable clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitDef.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1);
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_AN;
    GPIO_Init(GPIOB, &GPIO_InitDef); //pin 0, 1, 2 analog

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

}

void init_GPIOA(void) {

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //enable clock

    GPIO_InitTypeDef GPIO_InitDef; //structure

    GPIO_InitDef.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5
	    | GPIO_Pin_6 | GPIO_Pin_7);
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_AN;

    GPIO_Init(GPIOA, &GPIO_InitDef); //pin 0, 1, 2 analog

    GPIO_PinAFConfig(GPIOA, GPIO_Pin_8, GPIO_AF_MCO);

    GPIO_InitDef.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitDef);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
}

void init_GPIOC(void) {

    GPIO_InitTypeDef GPIO_InitDef; //structure

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitDef.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
	    | GPIO_Pin_4 | GPIO_Pin_5);
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_AN;
    GPIO_Init(GPIOB, &GPIO_InitDef);

    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

}

void init_Interrupts(void) {
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//systick is here??
//sysTick config
    SysTick_Config(SystemCoreClock / 1000);
}

void init_ADC1(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_InitCommonStructure;

    ADC_InitCommonStructure.ADC_Prescaler = ADC_Prescaler_Div2;

    ADC_CommonInit(&ADC_InitCommonStructure);

//  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
//  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_3Cycles);

    ADC_Cmd(ADC1, ENABLE);  //enable ADC1
    ADC_DMACmd(ADC1, ENABLE); //enable DMA for ADC
// ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void init_DMA2(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    DMA2_Stream0->PAR = (uint32_t)(ADC1_BASE + 0x4c); //set ADC1_DR port address DMA_SxPAR
    DMA2_Stream0->M0AR = (uint32_t) &adc1DmaWMem[0]; // SRAM adrese..vajag aspkatiities, vai stack kkur tur neparaskta
    DMA2_Stream0->NDTR = ITERATIONS; //number of data to be transferred
    DMA2_Stream0->CR |= (1 << 8 | 1 << 10 | 1 << 11
	    | 1 << 13); //(1<<8|1<<10|1<<12|1<<14);//circular mode,memory pointer is incremented,32 bit peripheral and 32 bit memory
    (*((uint32_t *) (DMA2_Stream0_BASE + 0x0u))) |= 1 << 0; //starts DMA
// DMA_Cmd(DMA2_Channel1, ENABLE);
}

void init_USART2(uint32_t baudrate) {

    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
    USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
    NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //enable APB2 peripheral clock for USART1

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //enable the peripheral clock for the pins used by  USART1

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // PB6 for TX and PB7. Pins 6 (TX) and 7 (RX) are used
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // the pins are configured as alternate function so the USART peripheral has access to them
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // this defines the IO speed and has nothing to do with the baudrate!
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	// this defines the output type as push pull mode (as opposed to open drain)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;// this activates the pullup resistors on the IO pins
    GPIO_Init(GPIOA, &GPIO_InitStruct);	// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    USART_InitStruct.USART_BaudRate = baudrate;	// the baudrate is set to the value we passed into this init function
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
    USART_InitStruct.USART_StopBits = USART_StopBits_1;	// we want 1 stop bit (standard)
    USART_InitStruct.USART_Parity = USART_Parity_No;// we don't want a parity bit (standard)
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
    USART_Init(USART2, &USART_InitStruct); // again all the properties are passed to the USART_Init function which takes care of all the bit setting

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; // we want to configure the USART1 interrupts
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // this sets the priority group of the USART1 interrupts
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // this sets the subpriority inside the group
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // the USART1 interrupts are globally enabled
    NVIC_Init(&NVIC_InitStructure); // the properties are passed to the NVIC_Init function which takes care of the low level stuff

    USART_Cmd(USART2, ENABLE); // finally this enables the complete USART1 peripheral
}
