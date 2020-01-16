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
#include "IR.h"
#include "Display.h"


void IR_Transmite_Cycling(uint8_t Step,uint16_t data)
{
	while(1)
	{
	  IR_Transmite_Status(Step,data);
	}
}
void IR_Transmite_Status(uint8_t Step,uint16_t data)
{
	uint32_t Temp=0;
	Temp|=Step;
	Temp<<=25;
	Temp|=data;
	if(data&0x00000001)Temp|=0x80000000;
	IR_Transmite_Data(Temp);
}
void IR_Transmite_Error(uint8_t Step,uint16_t code,uint16_t data)
{
	uint32_t Temp=0;
	
	Temp|=Step;
	Temp<<=9;
	Temp|= code;
	Temp<<=16;
	Temp|=data;
	Temp|=0x00800000;
	if(data&0x00000001)Temp|=0x80000000;
	IR_Transmite_Data(Temp);
  Set_LED_On_Switch(0,0,0,0,0,0);
	Set_LED_On_Blink(1,1,1,0,0,0);
}

void IR_Transmite_Data(uint32_t data)
{	
	IR_Timer_Configure();//set tim2 to 38 KHZ
	IR_L;
	delay(500);
	IR_Transmite_Header();
	IR_Transmite_Source(data);
	delay(50);
	IR_Transmite_Header();
	IR_Transmite_Source(data);
}

void IR_Timer_Configure(void)
{
//	TIM15->ARR = IR_Count_Range;
//	
//  TIM15->PSC = IR_Prescaler;
//	
//  TIM15->DIER &=~ TIM_DIER_CC4IE|TIM_DIER_CC3IE;
//	
//  TIM15->CCMR1  = (uint16_t)(TIM_CCMR1_OC1PE|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2);
//	TIM15->CCMR1 |= (uint16_t)(TIM_CCMR1_OC2PE|TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2);
}

void IR_Transmite_Header(void)
{
	IR_H;
	delay(60);
	IR_L;
	delay(40);
}

void IR_Transmite_Source(uint32_t data)
{
	uint8_t Bit=0;
	for(Bit=0;Bit<32;Bit++)
	{
		IR_H;
		if(data&0x80000000)
		{
			delay(20);
		}
		else
		{
			delay(10);
		}
		IR_L;
		delay(10);
		data<<=1;
	}
	IR_L;
}

