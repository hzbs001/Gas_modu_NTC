#include "stm32f0xx.h"
#include "adc.h"
#include "math.h"
#define ADC1_DR_Address    0x40012440
#define TC_CAL1            ((uint16_t *)0x1FFFF7B8)
#define VREFINT_CAL        ((uint16_t *)0x1FFFF7BA)

union pfloat{
	float a;
	uint8_t b[4];
};
//用于ADC转换DMA设置
static const uint8_t oversamp = 16;
static const uint8_t chQty = 4;
static const uint8_t buf_len = 64;
static uint16_t dma4adc[buf_len];
static uint8_t dma_complete_flag;

static ADC_InitTypeDef ADC_InitStructure;
static DMA_InitTypeDef DMA_InitStructure;
static GPIO_InitTypeDef GPIO_InitStructure;

static uint16_t adcValue,ntcValue,tempValue,vrefValue;
union pfloat prcv[16];
//用于曲线校准
static float sa=0.6,sb=-0.5;
static float ta=0,tb=1.0;
static float ha=0,hb=1.0;

static uint16_t rstd = 600;
static uint16_t rl = 470;
	
/*****设置DMA结束标志 ********/
void setDmaCompleteFlag(void)
{
	dma_complete_flag=1;
}
//传感器曲线多项式
float fx(float t,float h,float s)
{
	return((ta*t+tb)*(ha*h+hb)*sa*pow(s,sb));
}

float ntcCalc(uint16_t ntcVal,uint16_t *pADC)
{
	// NTC相关常数
	const uint16_t R25 = 10000;
	const uint16_t NTC_B = 3950;
	const uint8_t T25 = 25;
	const uint16_t ntcRL = 10000;
	const uint16_t FS = 4095;
	
	float ntcRs,ntcTemp;
	union pfloat ptemp;
	
	/*********获取NTC阻值***********/
	ntcRs = (float)(FS-ntcVal)/ntcVal*ntcRL;
	
	/***********计算温度***********/
	ntcTemp = (float)NTC_B*(T25+273.15)/ \
		(NTC_B+(T25+273.15)*log(ntcRs/R25))-273.15;
	
	//将温度转为IEEE754格式，并返回 
	ptemp.a = ntcTemp;
	*(pADC+2) = (ptemp.b[3]<<8) | ptemp.b[2];
	*(pADC+3) = (ptemp.b[1]<<8) | ptemp.b[0];
	
	return ntcTemp;
}

/**********adc data processing*************/
/*****channel1 with 16bits
******temperature with 12bits
******reference voltage1.2V with 16bits
--by lwang @home 2018/03/12 
******************************************/
float adcDataProcess(uint16_t *pADC)
{
	uint32_t tmp1=0,tmp2=0,tmp3=0,tmp4=0;
	uint16_t i,tDREF,vDREF,rs;
	float vREF;
	float x0,x1,x2,y0,y1;
	float temp,humi;
	float rsoverrstd;
	
	union pfloat temp_1,xroot;
	//ADC转换结束，计算浓度
	if(dma_complete_flag == 1){
			dma_complete_flag = 0;
			tDREF = *TC_CAL1;
			vDREF = *VREFINT_CAL;
			vREF = 1.2;
			for(i=0;i<oversamp;i++){
				tmp1 += dma4adc[0+chQty*i]&0xFFF;
				tmp2 += dma4adc[1+chQty*i]&0xFFF;
				tmp3 += dma4adc[2+chQty*i]&0xFFF;
				tmp4 += dma4adc[3+chQty*i]&0xFFF;
			}
			adcValue = tmp1/oversamp;
			ntcValue = tmp2/oversamp;
			tempValue = tmp3/oversamp;
			vrefValue = tmp4/oversamp;
		}
		
		/*******根据NTC获取温度***/
		temp = ntcCalc(ntcValue,pADC);
		
		/*****如果系数已配置，则读取系数
		 ****/
		if (*(pADC+15)==0x0F0F){
			rstd = *(pADC+16);
			rl = *(pADC+17);
			for(i=0;i<6;i++){
				prcv[i].b[3] = (*(pADC+18+2*i)>>8) & 0xFF;
				prcv[i].b[2] = *(pADC+18+2*i) & 0xFF;
				prcv[i].b[1] = (*(pADC+18+2*i+1)>>8) & 0xFF;
				prcv[i].b[0] = *(pADC+18+2*i+1) & 0xFF;
			}
			sa = prcv[0].a;
			sb = prcv[1].a;
		
			ta = prcv[2].a;
			tb = prcv[3].a;

			ha = prcv[4].a;
			hb = prcv[5].a;
		}

		/**********获取传感器电阻**********/
		rs = rl*(4095-adcValue)/adcValue;
		*(pADC+6)   = rs;
		*(pADC+7)   = adcValue;
		
		/**********二分法解方程**********/
		rsoverrstd = rs/(float)rstd;
		x0 = 0;x1 = 0;x2 = 199.9;
		while( (x1-x2>=0.001) || (x2-x1>=0.001) ){
			x0 = (x1+x2)/2;
			
			y0 = fx(temp,humi,x0)-rsoverrstd;
			y1 = fx(temp,humi,x1)-rsoverrstd;
			
			if (y0*y1>0)
				x1 = x0;
			else
				x2 = x0;
		}
		//唤峁狪EEE754格式
		xroot.a = x0;
		*pADC = (xroot.b[3]<<8) | xroot.b[2];
		*(pADC+1) = (xroot.b[1]<<8) | xroot.b[0];
	
	return adcValue;
}
//设置默认温度湿度值
void init_temp_humi(uint16_t *pADC,float temp,float humi)
{
	union pfloat{
	float a;
	uint8_t b[4];
	};
	union pfloat temp_1,humi_1;
	temp_1.a = temp;
	humi_1.a = humi;
	*(pADC+2) = (temp_1.b[3]<<8) | temp_1.b[2];
	*(pADC+3) = (temp_1.b[1]<<8) | temp_1.b[0];
	*(pADC+4) = (humi_1.b[3]<<8) | humi_1.b[2];
	*(pADC+5) = (humi_1.b[1]<<8) | humi_1.b[0];
}

void adc_GPIO_cfg(void)
{
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

/*************ADC conversion config********************/
/********Turn on CH1,CH16,CH17*************************/
/********Measure external,temp,vref********************/
void ADC_cfg()
{
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_ClockModeConfig(ADC1,ADC_ClockMode_SynClkDiv4); //PCLK/4
	ADC_ChannelConfig(ADC1,ADC_Channel_0 | ADC_Channel_1 | \
	    ADC_Channel_16 | ADC_Channel_17,ADC_SampleTime_239_5Cycles);
	ADC_TempSensorCmd(ENABLE);
	ADC_VrefintCmd(ENABLE);
	ADC_GetCalibrationFactor(ADC1);
	
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
  ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
	
}
/***************DMA control,16 data to average**********/
void DMA_cfg()
{
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dma4adc;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = buf_len;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

/********ADC interupt processing*****************/
void ADC1_IRQHandler(void)
{
	
}
/********DMA interupt processing*****************/
void DMA1_Channel1_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC1) != RESET)
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		setDmaCompleteFlag();
	}
}
