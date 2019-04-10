//#include "stm32f0xx_it.h"
#include "stm32f0xx.h"
#include "i2c_slave.h"
union pfloat{
	float a;
	uint8_t b[4];
};
union pfloat aa;
static GPIO_InitTypeDef GPIO_InitStructure;
static I2C_InitTypeDef I2C_InitStructure;

const static uint8_t i2c1_addr = 0x30;
const static uint8_t MAX_CNT = 200;

static uint8_t i2c_mode = 0;
//static uint16_t MB_TX_EN=0;
static uint16_t reg_offset = 0;

static uint8_t rx_cnt = 0;
static uint8_t tx_cnt = 0;
static uint8_t bytematch = 0;
static uint8_t regnum = 0;
static uint8_t data_buf[MAX_CNT];

void i2c1_GPIO_cfg(void)
{
	/****** Settings for USART1 *******************/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_4);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_4);
	
	//PA10:SDA  PA9:SCL
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
	//GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void i2c1_cfg(void)
{
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_OwnAddress1 = i2c1_addr<<1;
   
  I2C_Init(I2C1, &I2C_InitStructure);
	//I2C_Cmd(I2C1,DISABLE);
	I2C_Cmd(I2C1,ENABLE);
	I2C_ITConfig(I2C1, I2C_IT_ADDRI | I2C_IT_STOPI | \
			         I2C_IT_RXI | I2C_IT_TXI , ENABLE);
}
//i2c从机中断处理，遵循标准I2C
void I2C1_IRQHandler(void)
{
	if(I2C_GetITStatus(I2C1,I2C_IT_ADDR))
	{
		I2C_ClearITPendingBit(I2C1,I2C_IT_ADDR);
		if(I2C_GetTransferDirection(I2C1))
		{
			i2c_mode = 1;//Master Read
			i2cRegUpdate(data_buf,0,32,i2c_mode);
			tx_cnt = 0;
			I2C1->ISR |= I2C_ISR_TXE;
		}
		else if(I2C_GetTransferDirection(I2C1)==0)
		{
			i2c_mode = 0;//Master Write
			rx_cnt = 0;
		}
		
	}
	else if(I2C_GetITStatus(I2C1,I2C_IT_TXIS))
	{
		if(i2c_mode)
		{
		I2C_SendData(I2C1,data_buf[reg_offset+tx_cnt]);
		tx_cnt++;
		}
		if(tx_cnt>MAX_CNT)
		{
			tx_cnt = 0;
		}
	}
	else if(I2C_GetITStatus(I2C1,I2C_IT_RXNE))
	{
		data_buf[rx_cnt++] = I2C_ReceiveData(I2C1);
		bytematch++;
		if(rx_cnt==1)
		{
			reg_offset = data_buf[0];
			bytematch = 0;
			regnum = 0;
		}
		if(bytematch==2)
		{
			bytematch=0;
			i2cRegUpdate(&data_buf[rx_cnt-2],reg_offset+regnum,1,i2c_mode);
			regnum++;
		}
			
	}
		
	else if(I2C_GetITStatus(I2C1,I2C_IT_NACKF))
	{
		I2C_ClearITPendingBit(I2C1,I2C_IT_NACKF);
		rx_cnt = 0;
		tx_cnt = 0;
		regnum = 0;	
		bytematch = 0;
	}
	else if(I2C_GetITStatus(I2C1,I2C_IT_STOPF))
	{
		I2C_ClearITPendingBit(I2C1,I2C_IT_STOPF);
		rx_cnt = 0;
		tx_cnt = 0;
		regnum = 0;
		reg_offset = 0;
		bytematch = 0;
		I2C_Cmd(I2C1,ENABLE);
	}
	else if(I2C_GetITStatus(I2C1,I2C_IT_BERR))
	{
		I2C_ClearITPendingBit(I2C1,I2C_IT_BERR);
	}
	else if(I2C_GetITStatus(I2C1,I2C_IT_OVR))
	{
		I2C_ClearITPendingBit(I2C1,I2C_IT_OVR);
	}
	
}
