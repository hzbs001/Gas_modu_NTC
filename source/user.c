#include "stm32f0xx.h"
#include "user_init.h"

#include "adc.h"
#include "metronome.h"
#include "eeprom_emulation.h"

#include "i2c_slave.h"
#include "ctrl.h"

#define TC_CAL1            ((uint16_t *)0x1FFFF7B8)
#define VREFINT_CAL            ((uint16_t *)0x1FFFF7BA)

/* ----------------------- Defines ------------------------------------------*/
const uint8_t REG_HOLDING_START = 0;
const uint8_t REG_HOLDING_NREGS = 32;
float T,H;
/* ----------------------- Static variables ---------------------------------*/
static uint16_t   usRegHoldingStart = REG_HOLDING_START;
static uint16_t   usRegHoldingBuf[REG_HOLDING_NREGS];

/* ----------------------- Start implementation -----------------------------*/
int main( void )
{
	
	static uint16_t tmp1,tmp2;
	uint8_t i;
	static float f1,v0;
	
	//时钟和中断配置
	RCC_cfg();
	NVIC_cfg();
	
	//ADC 配置
	adc_GPIO_cfg();
	ADC_cfg();
	DMA_cfg();
	
	//For I2C1 slave
	i2c1_GPIO_cfg();
	i2c1_cfg();
	
	//节拍定时器配置
	mtn_TIM17_cfg();
	
	//加热IO控制
	heat_GPIO_cfg();
	
	//启动ADC转换
	ADC_StartOfConversion(ADC1);
	
	//上电读取系数
	rd_coff(&usRegHoldingBuf[15]);
	
	// 设置默认温度和湿度
	init_temp_humi(usRegHoldingBuf,25.0,55.0);

  while(1){
		//PA4=1，开启I2C1Ｄ否则关闭
		(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4))?I2C_Cmd(I2C1,ENABLE):I2C_Cmd(I2C1,DISABLE);
		//如果pos 30=0x9999,写入系数
		if(usRegHoldingBuf[30]==0x9999){
			wr_coff(&usRegHoldingBuf[15]);
		}
		//节拍控制
		if(time2tell()==1){
			//加热控制
			heat_ctr(&usRegHoldingBuf[8]);
			
			//更新浓度，温度，返回值无意义
			v0 = adcDataProcess(usRegHoldingBuf); 
			
		}
  }
}
//更新寄存器
void i2cRegUpdate(uint8_t *p,uint16_t regAddr,uint16_t regNum,uint8_t mode)
{
	uint16_t i;
	uint16_t iRegIndex;
	if((regAddr >= REG_HOLDING_START) && ((regAddr+regNum) <= (REG_HOLDING_START+REG_HOLDING_NREGS)))
	{
		iRegIndex = (uint16_t)(regAddr-usRegHoldingStart);
		
		switch(mode)
		{
			case 1:
			{
				for(i=0;i<regNum;i++)
				{
					*p++ = (uint8_t)(usRegHoldingBuf[iRegIndex+i] >> 8 );  //High byte first
					*p++ = (uint8_t)(usRegHoldingBuf[iRegIndex+i] & 0xFF); //low byte first
				}
			}
			break;
			case 0:
			{
				for(i=0;i<regNum;i++)
				{
						*(usRegHoldingBuf+iRegIndex+i)= ((*p++))<<8; 	//High byte
						*(usRegHoldingBuf+iRegIndex+i)|= *(p++); 			//Low byte
				}
				break;

			}	
		}
	}
}

void SystemInit()
{
	
}
