/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Initialize Usart function and prosessing the characters transmitting
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "Flash.h"
#include "config.h"


#define FLASH_ADDRESS	0x0803f800


void Add_Life(uint32_t T)
{
	uint16_t temp;
	temp = Read_Flash(FLASH_ADDRESS);
	if(temp>9999)temp=0;//over 999 hours 
	if(T>=720)
	{
		temp+=(T/720);
		fmc_halfword_program(FLASH_ADDRESS, temp);
	}
}

void Clear_Life(void)
{
	fmc_halfword_program(FLASH_ADDRESS, 0);
}


uint32_t Read_Flash(uint32_t add)
{
	uint32_t data;
	data = *(__IO uint16_t*)add;
	return data;
}



