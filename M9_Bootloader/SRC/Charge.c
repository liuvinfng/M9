 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    5-Dec-2011
  * @brief   Charge Function 
	           initial charge current 50 mA then increasing by time to 500 mA 
						 untill Fully charged ,which by detecting the rising rate of the 
						 temperature ( equote to falling rate of NTC >15 per minute).or 
						 battery voltage reach 1850 mV or NTC temperature reach 45 degree
						 or battery voltage overcharge flag came .
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "Charge.h"
#include "Speaker.h"
#include "Display.h"
#include "USART.h"
#include "TouchPad.h"
#include "Charge.h"
#include "Movement.h"
#include "UserInterface.h"
#include "Rcon.h"
#include "RTC.h"
#include "SysInitialize.h"
#include "display.h"
#include "debug.h"
#include "usart.h"



volatile Charge_t g_charge;


void Charge_SetSwitch(uint8_t state)
{
	g_charge.power_switch = state;
}


void Charge_Process(void) 
{
	static uint8_t bat_error_cnt = 0,tick = 0;
	static uint16_t set_current = 50,temp_pwm = 0;
	static int8_t stable_cnt = 0;
	int32_t temp_charge_current = 0;
	int16_t temp_bat_v = 0;
		
	if(g_charge.power_switch != ENABLE)
	{
		if(g_charge.step == CHARGE_IDLE)
		{
			return;
		}
		else
		{
			g_charge.step = CHARGE_EXIT;
		}
	}

	switch(g_charge.step)
	{
		case CHARGE_IDLE:	
										if(g_charge.power_switch == ENABLE)g_charge.step = CHARGE_INIT;									
										break;
		
		case CHARGE_INIT:
										g_minute_counter=0;
										Charge_Configuration();
										g_baselineadc=GetBaseLineADCV();
										g_charge.type = CHARGE_STATION;
										Speaker(SPK_CHARGING_START);
										Display_SetBattery(LED_BAT_Y);
		
										if(GetBatteryVoltage() < 1650)
										{
											g_charge.charge_mode = CC_CHARGE; 
											temp_pwm = 0;
										}	
										else
										{
											g_charge.charge_mode = CV_CHARGE;
											temp_pwm = 500;	//avoid into the Finished_Charge immediately.
										}	
										Charge_PWM = 0;		
										
										bat_error_cnt = 0;
										set_current = 50;
										g_charge.step = CHARGEING;	
										g_charge.time = 0;
										stable_cnt = 0;
										break;
		case CHARGEING:		
										if((++tick) >= 4)tick = 0;
										temp_bat_v = GetBatteryVoltage();

										if(temp_bat_v > 1720)   //Normally a battery Never reach 1900 mV ,which means no battery connected (main switch off).
										{
											if((++bat_error_cnt) > 3)
											{
												g_charge.step = CHARGE_EXIT;				
												break;
											}
										}
										else
										{
											bat_error_cnt = 0;
										}										
										
										if(g_minute_counter>360)
										{											
											g_charge.charge_mode = FINISH_CHARGE;
											break;
										}

										if(g_adc_value.System_Current<g_baselineadc)temp_charge_current=Get_Charge_Current(g_baselineadc);
										else temp_charge_current=0;										

										if(temp_bat_v > 1100)//normally increase the charge current to 600 mA
										{		
											if(tick == 0)
											{											
												if(set_current < Charge_Target_Current)set_current++;
											}
										}
										else// Over discharged recovery
										{
											set_current=50;
										}										
										
										switch(g_charge.charge_mode)
										{
											case CC_CHARGE:
																if(temp_bat_v >= 1660)
																{
																	g_charge.charge_mode = CV_CHARGE;
																	break;
																}		
																
																if(temp_charge_current > set_current)//charge current over
																{
																	if(temp_pwm > 2)temp_pwm--;									
																}
																else if(temp_charge_current < set_current)
																{ 
																	if(tick == 0)
																	{
																		if(temp_pwm < 1000)temp_pwm++;		
																	}
																}											 
																break;
											case CV_CHARGE:
																if(temp_bat_v > Battery_FullVoltage)
																{
																	stable_cnt++;
																	
																	if(stable_cnt >= 5)
																	{
																		stable_cnt = 0;
																		
																		if(temp_pwm > 0)temp_pwm --;
																	}
																	
																	if(temp_charge_current < Charge_FinishCurrent) 
																	{
																		g_charge.charge_mode = FINISH_CHARGE;
																	}
																}
																else
																{
																	stable_cnt--;
																	if(stable_cnt <= -5)
																	{
																		stable_cnt = 0;
																		if(temp_pwm < 1000)temp_pwm++;		
																	}
																}
						
																break;	
											case FINISH_CHARGE:
																temp_pwm = 0;
																g_charge.step =  CHARGE_FINISH;
																Display_SetBattery(LED_BAT_G);
																if(temp_bat_v <= 1600)
																{
																	g_minute_counter=0;
																	g_charge.charge_mode = CC_CHARGE;	
																}
																break;
											default:
																g_charge.charge_mode = FINISH_CHARGE;	
																break;
										}

										Charge_PWM = temp_pwm;//
										break;	
					
		case CHARGE_FINISH:
										Display_SetBattery(LED_BAT_G);
										break;	
		
		case CHARGE_EXIT:
										Charge_PWM = 0;
//										Work_Configuration();	
										Display_SetBattery(LED_BAT_G);
										g_charge.step = CHARGE_IDLE;
										break;			
		default:										
										g_charge.step = CHARGE_EXIT;
										break;	
	}
	
	if(g_charge.time < 0xfffffff0)g_charge.time++;
	

	if(tick == 0)
	{ 
#if 1	
		static uint8_t num=0;
		num++;
		if(num>20)
		{
			num=0;
			USPRINTF("V:%d\n", temp_bat_v);
			USPRINTF("g_baselineadc:%d\n", g_baselineadc);
			USPRINTF("now_baselineadc:%d\n", g_adc_value.System_Current);		
			USPRINTF("C:%d\n", temp_charge_current);
			USPRINTF("T:%d\n", g_minute_counter);
			USPRINTF("PWM:%d\n", Charge_PWM);		
			USPRINTF("M:%d\n\n", g_charge.charge_mode);			
		}

#endif		
	}	
}




uint16_t GetBaseLineADCV(void)
{
  uint32_t SystemVoltageSum=0;
  uint8_t temp=0;
  I_CTRL_ON;
	delay(100);
  for(temp=0;temp<10;temp++)
  {
    delay(10);
    SystemVoltageSum+=g_adc_value.System_Current;
  }
  I_CTRL_OFF; 
	SystemVoltageSum/=10;
	return (uint16_t)SystemVoltageSum;
}


uint16_t Get_Charge_Current(uint16_t BaselineADCV)//ma
{
  uint32_t temp=0;
  uint8_t count=0;
  for(count=0;count<10;count++)
  {
    temp+=g_adc_value.System_Current;
    delay(10);
  }
	temp/=10;
	if(BaselineADCV > temp)
	{
		temp=(BaselineADCV-temp)*330/4096;
		temp *=10;	
	}else
	temp = 0;
  return (uint16_t)temp;
}

uint16_t Get_Sys_Current(uint16_t BaselineADCV)
{
  uint32_t temp=0;
  uint8_t count=0;
  for(count=0;count<10;count++)
  {
    temp+=g_adc_value.System_Current;
    delay(10);
  }
	temp/=10;
	if(temp>BaselineADCV)
	{
		temp=(temp - BaselineADCV)*330/4096;
		temp *=10;
		temp +=20;
	}else
	temp = 0;
  return (uint16_t)temp;
}




