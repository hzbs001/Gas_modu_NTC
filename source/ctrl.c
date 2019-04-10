#include "stm32f0xx.h"
#include "ctrl.h"

static GPIO_InitTypeDef GPIO_InitStructure;

void heat_ctr(uint16_t *pADC)
{
	//pos 8==0x0886则关闭加热
	if(pADC[8]==0x0886)
		GPIO_SetBits(GPIOA,GPIO_Pin_5);
	else
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}

void heat_GPIO_cfg(void)
{
	/********* Settings for Hear control ***********/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}
