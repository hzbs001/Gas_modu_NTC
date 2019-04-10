void ADC_cfg(void);
void DMA_cfg(void);
void adc_GPIO_cfg(void);

void setDmaCompleteFlag(void);
float adcDataProcess(uint16_t *pADC);
void init_temp_humi(uint16_t *pADC,float temp,float humi);
//void adcData4Brc(uint16_t *pADC,uint8_t count);
//float getAdc0Data(void);
//void solve2f(float v0,float temp,float humi,uint16_t *pbuf,uint16_t *out);
/*******interupt process*****/
void ADC1_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
