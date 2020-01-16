/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2018
  * @brief   Basical Movement functions
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "Bldc.h"

volatile int32_t BLDC_Pulse_Counter=0;
volatile int32_t BLDC_Temp_PWM=0;
volatile int32_t BLDC_Speed=0;
volatile uint8_t Vac_Mode=0;
volatile uint8_t BLDC_Fail_Flag=0;


void Set_Vac_Speed()
{
	if(Is_Water_Tank())
	{
		Set_BLDC_Speed(0);
		BLDC_OFF;
	}
	else
	{
		if(Get_VacMode()==Vac_Max)
		{
			Set_BLDC_Speed(VAC_SPEED_MAX);
		}
		else
		{
			Set_BLDC_Speed(VAC_SPEED_NORMAL);
		}
		BLDC_ON;
	}
}
void Set_BLDC_Speed(uint32_t S)
{
	BLDC_Speed=S;
}

/*-------------------------------------------------*/
void Set_BLDC_Fail(void)
{
	BLDC_Fail_Flag=1;
}
void Clear_BLDC_Fail(void)
{
	BLDC_Fail_Flag=0;
}
uint8_t Is_BLDC_Fail(void)
{
	return BLDC_Fail_Flag;
}

/*-------------------------------------------------*/
void Turn_BLDC_Off(void)
{
	Set_BLDC_Speed(0);
	BLDC_OFF;
	Set_BLDC_TPWM(10);
}
void Set_BLDC_TPWM(uint16_t P)
{
	BLDC_Temp_PWM=P;
}
uint8_t Get_VacMode(void)
{
	return Vac_Mode;
}
void Set_VacMode(uint8_t data)
{
	Vac_Mode = data;
}
void Switch_VacMode(void)
{
	if(Get_VacMode()==Vac_Normal)
	{
		Set_BLDC_TPWM(60);
		Set_VacMode(Vac_Max);
	}
	else
	{
		Set_BLDC_TPWM(30);
		Set_VacMode(Vac_Normal);
	}
	Set_Vac_Speed();
}

void Bldc_Loop(void)
{
	static volatile uint32_t BLDC_FAIL_Counter=0;
	int32_t Dif_RPM=0;
	if(Is_Bldc_Open())
	{
	 Dif_RPM = BLDC_Speed - BLDC_Pulse_Counter;
	 if(Dif_RPM>0)
	 {
		 if(Dif_RPM>300)
		 {
			 BLDC_Temp_PWM+=5;
		 }
		 else if(Dif_RPM>80)
		 {
			 BLDC_Temp_PWM++;
		 }
		 if(BLDC_Temp_PWM>99)BLDC_Temp_PWM=100;
	 }
	 else
	 {
		 if(Dif_RPM<-100)
		 {
			 if(BLDC_Temp_PWM>5)BLDC_Temp_PWM-=5;
		 }
		 else if(Dif_RPM<-80)
		 {
			 BLDC_Temp_PWM--;
		 }
	 }
	 if(BLDC_Temp_PWM<10)BLDC_Temp_PWM=10;
	 BLDC_PWM = BLDC_Temp_PWM;
	 if(BLDC_Pulse_Counter==0)
	 {
		 BLDC_FAIL_Counter++;
		 if(BLDC_FAIL_Counter>10)
		 {
			 BLDC_FAIL_Counter=0;
			 Set_BLDC_Fail();
		 }
	 }
	 else
	 {
		 BLDC_FAIL_Counter=0;
	 }
	}
	else
	{
	 BLDC_FAIL_Counter=0;
	}
	BLDC_Pulse_Counter=0;
}

