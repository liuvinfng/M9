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
#include "Include.h"
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

volatile Charge_t g_charge;


void Charge_SetSwitch(uint8_t state)
{
	g_charge.power_switch = state;
}


void Charge_Process(void) 
{
	static uint8_t bat_error_cnt = 0,tick = 0,speak_enable=0,display_cnt=0;
	static uint16_t set_current = 50,temp_pwm = 0;
	static int8_t stable_cnt = 0;
	int32_t temp_charge_current = 0;
	int16_t temp_bat_v = 0;
		
	if(g_charge.power_switch != ENABLE)
	{
		if(g_charge.step == CHARGE_IDLE)
		{
			#ifdef WIFI_TY
			if(SysConfig.Ty_CurrentState == CHARGE_STATE)
			{
				SysConfig.Ty_CurrentState = IDLE_STATE;			
			}
			#endif			
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
										speak_enable = 1;
										Charge_Configuration();
										g_charge.type = CHARGE_STATION;
										
										Display_SetBattery(LED_BAT_Y);
										User_SetQuitCheckState(DISABLE);	
										printf("%s(%d):MODE_CHARGE!\n",__FUNCTION__, __LINE__);																																						
										Time_ResetCurrentTime();		
										if(Battery_GetVoltage() < 1650)
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
										temp_bat_v = Battery_GetVoltage();
										if(temp_bat_v>=1640)
										{
											display_cnt++;
											if(display_cnt>=10)
											{
												display_cnt = 10;
												Display_SetBattery(LED_BAT_G);
											}										
										}
										else
										{
											if(display_cnt>0)display_cnt--;
											if(!display_cnt)Display_SetBattery(LED_BAT_Y);											
										}
											
										
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

										temp_charge_current=Get_Charge_Current(System_GetCurrentBaselineAdc());
																			

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
										Timer3_Configuration();	
										Display_SetBattery(LED_BAT_G);
										g_charge.step = CHARGE_IDLE;
										break;			
		default:										
										g_charge.step = CHARGE_EXIT;
										break;	
	}
	
	if(g_charge.time < 0xfffffff0)g_charge.time++;
	if(speak_enable)
	{		
		if(g_charge.time>100)
		{
			speak_enable = 0;
			Speaker(SPK_CHARGING_START);
			#ifdef WIFI_TY
			SysConfig.Ty_CurrentState = CHARGE_STATE;
			#endif
		}
	}	

//	if(tick == 0)
	{ 
#if 0	
		static uint8_t num=0;
		num++;
		if(num>10)
		{
			num=0;
			USPRINTF("V:%d\n", Battery_GetVoltage());
			USPRINTF("g_baselineadc:%d\n", System_GetCurrentBaselineAdc());
			USPRINTF("now_baselineadc:%d\n", g_adc_value.System_Current);		
			USPRINTF("C:%d\n", temp_charge_current);
			USPRINTF("T:%d\n", g_minute_counter);
			USPRINTF("PWM:%d\n", Charge_PWM);		
			USPRINTF("M:%d\n\n", g_charge.charge_mode);			
		}
#endif		
	}	
}



uint16_t Get_Charge_Current(uint16_t BaselineADCV)//ma
{
  uint32_t temp=0;
  uint8_t count=0;
  for(count=0;count<10;count++)
  {
    temp+=g_adc_value.System_Current;
    osDelay(1);
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
    osDelay(10);
  }
	temp/=10;
	if(temp>BaselineADCV)
	{
		temp=(temp - BaselineADCV)*330/4096;
		temp *=10;
		temp +=30;
	}else
	temp = 0;
  return (uint16_t)temp;
}




