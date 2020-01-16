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
#include "include.h"
#include "main.h"
#include "charge.h"
#include "projecttask.h"
#include "userinterface.h"
#include "remote_mode.h"
#include "spot.h"
#include "standby.h"
#include "wallfollow.h"
#include "homestraight.h"
#include "cormove.h"
#include "pathplanning.h"
#include "shortestpath.h"
#include "map.h"
#include "wifi.h"
#include "movement.h"
#include "display.h"
#include "usart.h"
#include "spi.h"
#include "speaker.h"
#include "rcon.h"
#include "rtc.h"
#include "touchpad.h"
#include "gyro.h"
#include "wheel.h"
#include "obscliff.h"
#include "bldc.h"
#include "brush.h"
#include "w25q16.h"
#include "mymath.h"
#include "debug.h"
#include "cmsis_os.h"
#include "config.h"

volatile int32_t g_vac_pulse_cnt=0;
volatile int32_t g_vac_temp_pwm=0;
volatile int32_t g_vac_speed=0;
volatile uint8_t Vac_Mode=Vac_Normal;



void Set_Vac_Speed(void)
{
//	if(Is_Water_Tank())
//	{
//		Vacuum_SetSpeed(0);
//		BLDC_OFF;
//	}
//	else
//	{
//		if(Get_VacMode()==Vac_Max)
//		{
//			Vacuum_SetSpeed(VAC_SPEED_MAX);
//		}
//		else
//		{
//			Vacuum_SetSpeed(VAC_SPEED_NORMAL);
//		}
//		BLDC_ON;
//	}
}
void Vacuum_SetSpeed(uint32_t S)
{
	g_vac_speed=S;
	if(g_vac_speed == 0)
	{
		BLDC_OFF;
		return;
	}
	if(BLDC_PWM < 10)
	{
		if(Is_Bldc_Open())BLDC_OFF;
		BLDC_PWM = 10;
	}
	BLDC_ON;	
}
uint32_t Vacuum_GetSpeed(void)
{
	return  g_vac_speed;
}


/*-------------------------------------------------*/
/*vac fail*/
volatile uint8_t g_vac_fail_flag = 0;
uint8_t Vacuum_IsFail(void)
{
	return g_vac_fail_flag;
}
void Vacuum_SetFailFlag(void)
{
	g_vac_fail_flag = 1;
}
void Vacuum_ResetFailFlag(void)
{
	g_vac_fail_flag = 0;
}

/*vac cur*/
volatile int16_t g_vac_current = 0;
void Vacuum_SetCurrent(int16_t current)
{
	g_vac_current = current;
}
int16_t Vacuum_GetCurrent(void) 
{
	return g_vac_current;
}
uint16_t Vacuum_GetCurrentAdc(void)
{
  return g_adc_value.Vacuum_Current;
}

/*vac check cur*/
uint8_t Vacuum_CheckCurrent(void) 
{
	static uint8_t vac_temp_stall_cnt=0;
	int16_t temp_current = 0;
	
	temp_current = Vacuum_GetCurrent();
	if(BLDC_PWM < 50)return 0;
	if((temp_current >  VACUUM_STALL_LIMIT)
		|| (temp_current < VACUUM_STALL_NOLOAD))
	{
		Usprintf("vac current error:%d\n",temp_current);
		vac_temp_stall_cnt++;
	}
	else
	{
		vac_temp_stall_cnt = 0;
	}
	if(vac_temp_stall_cnt > 200)
	{
		Usprintf("vac current error\n");
		vac_temp_stall_cnt = 0;
		return 1;
	}
	return 0;
}

/*-------------------------------------------------*/
void Vacuum_TurnOff(void)
{
	Vacuum_SetSpeed(0);
	BLDC_OFF;
//	Vacuum_SetTempPWM(10);
}
void Vacuum_SetTempPWM(uint16_t P)
{
	g_vac_temp_pwm=P;
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
//	if(Get_VacMode()==Vac_Normal)
//	{
//		Vacuum_SetTempPWM(60);
//		Set_VacMode(Vac_Max);
//	}
//	else
//	{
//		Vacuum_SetTempPWM(30);
//		Set_VacMode(Vac_Normal);
//	}
//	Set_Vac_Speed();
}

void Vacuum_TuneProcess(void)
{
	int32_t diff_rpm = 0;
	static uint8_t tune_cyc_cnt = 0;
	static uint32_t vac_fail_cnt = 0;
	tune_cyc_cnt++;
	if(tune_cyc_cnt > 19)//500ms
	{
		tune_cyc_cnt = 0;
		if(Is_Bldc_Open())//check if BLDC enable
		{
			if(g_vac_speed>0)
			{
				if(Get_VacMode()==Vac_Max)g_vac_speed=VAC_SPEED_MAX;
				if(Get_VacMode()==Vac_Normal)g_vac_speed=VAC_SPEED_NORMAL;
				if(Get_VacMode()==Vac_Eco)g_vac_speed=VAC_SPEED_ECO;
			}
			diff_rpm = g_vac_speed - g_vac_pulse_cnt;
			if(diff_rpm > 0)
			{
				if(diff_rpm > 300)
				{
					g_vac_temp_pwm += 10;//20
				}
				else if(diff_rpm > 80)
				{
					g_vac_temp_pwm += 3;
				}
				if(g_vac_temp_pwm > 99)g_vac_temp_pwm = 99;
			}
			else
			{
				if(diff_rpm <- 100)
				{
					if(g_vac_temp_pwm > 5)g_vac_temp_pwm -= 3;
				}
				else if(diff_rpm <- 80)
				{
					g_vac_temp_pwm --;
				}
			}
			if(g_vac_temp_pwm < 5)g_vac_temp_pwm = 5;
			BLDC_PWM = g_vac_temp_pwm;
			if(g_vac_pulse_cnt == 0)
			{
				vac_fail_cnt++;
				if(vac_fail_cnt > 10)
				{
					vac_fail_cnt = 0;
					Vacuum_SetFailFlag();
				}
			}
			else
			{
				vac_fail_cnt = 0;
			}
		}
		else
		{
			vac_fail_cnt = 0;
		}
		g_vac_pulse_cnt = 0;
	}
}


