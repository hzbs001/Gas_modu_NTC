#include "stm32f0xx.h"
#include "eeprom_emulation.h"
union pfloat{
	float a;
	uint8_t b[4];
};
const uint32_t EEPROM_ADDR = FLASH_BASE + 0x3C00;

void wr_flash_hw(uint16_t *buf,uint8_t strtNum,uint8_t num)
{
	//first half word 0x0707 is write protect flag
	//if first half word=0x0707 then stop write
	uint8_t i;
	uint16_t *p;
	p = (uint16_t *)EEPROM_ADDR;
	//if(*(p+strtNum) !=0x0707)
	//{
	FLASH_Unlock();
	//FLASH_ProgramHalfWord(EEPROM_ADDR+strtNum*2,0x0707);
	for(i=0;i<num;i++)
	{
		FLASH_ProgramHalfWord(EEPROM_ADDR+strtNum*2+2*i,*(buf+i));
	}
	FLASH_Lock();
	//}
}

void rd_flash_hw(uint16_t *pbuf,uint8_t strtNum,uint8_t num)
{
	uint16_t *p;
	uint8_t i;
	p = (uint16_t *)EEPROM_ADDR;
	p += strtNum;
	for(i=0;i<num;i++)
	{
		*pbuf++ = *p++;
		
	}
}

uint16_t rd_flash_1hw(uint16_t *pbuf,uint8_t strtNum)
{
	uint16_t *p;
	uint8_t i;
	p = (uint16_t *)EEPROM_ADDR;
	p += strtNum;

	return *p;

}

void rd_coff(uint16_t *pbuf)
{
	uint16_t temp;
	//rd_flash_1hw(pbuf,0);
	temp = rd_flash_1hw(pbuf,0);
	
	switch(temp)
	{
		case 0x0F01:rd_flash_hw(pbuf,0,3);break;//status1+RS_RL2
		case 0x0F03:rd_flash_hw(pbuf,0,11);break;//status1+RS_RL2+Sens8
		case 0x0F07:rd_flash_hw(pbuf,0,19);break;//status1+RS_RL2+Sens8+temp8
		case 0x0F0F:rd_flash_hw(pbuf,0,15);break;//status1+RS_RL2+Sens8+temp8+Hum4
	}
}
void wr_coff(uint16_t *pbuf)
{
	/*** sensity parameter ***
		 *** ax3+bx2+cx+d    ***
		 temp parameter
		 *** a2t3+b2t2+c2t+d2***
	   *** Rs value ***
	   *** RL value ***/
	if(rd_flash_1hw(pbuf,0)==0xFFFF)
	{
	switch(*pbuf)
	{
		case 0x0F01:wr_flash_hw(pbuf,0,3);break;//status1+RS_RL2
		case 0x0F03:wr_flash_hw(pbuf,0,11);break;//status1+RS_RL2+Sens8
		case 0x0F07:wr_flash_hw(pbuf,0,19);break;//status1+RS_RL2+Sens8+temp8
		case 0x0F0F:wr_flash_hw(pbuf,0,15);break;//status1+RS_RL2+Sens8+temp8+Hum4
	}
	}
}
