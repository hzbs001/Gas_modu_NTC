#include "stm32f0xx.h"
#include "metronome.h"

static GPIO_InitTypeDef GPIO_InitStructure;
static TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//static uint8_t i4led = 0;
static uint8_t l2tell = 0;

uint8_t time2tell(void)
{
	if(l2tell==1)
	{
		l2tell = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}
/****** Settings for LED1, LED2 ************/
void led_GPIO_cfg(void)
{
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //LED1 LED2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

void mtn_TIM17_cfg(void)
{
	TIM_TimeBaseStructure.TIM_Prescaler = 3999; //1199; //4MHz/4000= 1KHz
	TIM_TimeBaseStructure.TIM_Period = 499; //1 second
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM17,&TIM_TimeBaseStructure);
	
	TIM_ARRPreloadConfig(TIM17,ENABLE);
	TIM_ITConfig(TIM17,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM17,ENABLE);
}

void TIM17_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM17,TIM_IT_Update)== SET)
    {
      TIM_ClearITPendingBit(TIM17,TIM_IT_Update);//clear interupt mark
			//led_tog();
			l2tell = 1;
		}
}
