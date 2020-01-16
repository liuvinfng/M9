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


void Brush_Main_SetPWM(uint8_t pwm)
{
  Main_Brush_PWM = pwm;
}

void Brush_Side_SetPWM(uint8_t pwm)
{
  Right_Brush_PWM = pwm;
}

/*side brush cur*/
volatile int16_t g_side_brush_current = 0;
void Side_Brush_SetCurrent(int16_t current)
{
	g_side_brush_current = current;
}
int16_t Side_Brush_GetCurrent(void)
{
	return g_side_brush_current;
}
uint16_t Side_Brush_GetCurrentAdc(void)
{
  return g_adc_value.Right_Brush_Current;
}

/*main brush cur*/
volatile int16_t g_main_brush_current = 0;
void Main_Brush_SetCurrent(int16_t current)
{
	g_main_brush_current = current;
}
int16_t Main_Brush_GetCurrent(void)
{
	return g_main_brush_current;
}
uint16_t Main_Brush_GetCurrentAdc(void)
{
  return g_adc_value.Main_Brush_Current;
}

/*check brush cur*/
uint8_t Side_Brush_CheckCurrent(void) 
{
	static uint8_t side_brush_stall_cnt;
  int16_t temp_current = 0;
	
	temp_current = Side_Brush_GetCurrent();
	if(temp_current < SIDE_BRUSH_STALL_NOLOAD)
	{

	}
	else if(temp_current > SIDE_BRUSH_STALL_LIMIT)
	{
		side_brush_stall_cnt += 4;
	}
	else
	{
		side_brush_stall_cnt = 0;
	}	
	
	if(side_brush_stall_cnt > 200)
	{
		Usprintf("%s(%d):brush Error %d\n",__FUNCTION__, __LINE__,temp_current);
		side_brush_stall_cnt = 0;
		return 1;
	}
	return 0;	
}
uint8_t Main_Brush_CheckCurrent(void) 
{
	static uint8_t main_brush_stall_cnt;
  int16_t temp_current = 0;
	
	temp_current = Main_Brush_GetCurrent();
	if(temp_current < MAIN_BRUSH_STALL_NOLOAD)
	{

	}
	else if(temp_current > MAIN_BRUSH_STALL_LIMIT)
	{
		main_brush_stall_cnt += 4;
	}
	else
	{
		main_brush_stall_cnt = 0;
	}	
	
	if(main_brush_stall_cnt > 200)
	{
		Usprintf("%s(%d):brush Error %d\n",__FUNCTION__, __LINE__,temp_current);
		main_brush_stall_cnt = 0;
		return 1;
	}
	return 0;	
}







