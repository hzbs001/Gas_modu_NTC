#include "stm32f0xx.h"
#include "user_init.h"

static NVIC_InitTypeDef NVIC_InitStructure;


void RCC_cfg()
{
	/*******Using HSI = 8MHz***********/
	
	RCC_HSEConfig(RCC_HSE_OFF);
	
	RCC_PLLCmd(DISABLE);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI); //sysclk=8MHz
	RCC_HCLKConfig(RCC_SYSCLK_Div1); 	//Hclk=8MHz
	RCC_PCLKConfig(RCC_HCLK_Div2); 		//Pclk=4MHz*/
	
	//开启外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 | RCC_APB1Periph_I2C1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 | RCC_APB1Periph_I2C1,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 | RCC_APB1Periph_I2C1,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_ADC1 | \
	                       RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17 | \
												 RCC_APB2Periph_SYSCFG,ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_DMA1,ENABLE);
	
}
//中断配置
void NVIC_cfg()
{
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	/*************For infrared receive************/
	NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	/*************For modbus***********************/
	NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	/*************For Led***********************/
	NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

