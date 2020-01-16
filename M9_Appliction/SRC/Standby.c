 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Standby Fuction , Turn off all the unworking peripheral except TIM4 and 2 gpios, waiting for
	           Button function or remote code to wakeup 
	           
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


void Standby_Mode(void)
{
	uint8_t back_to_interface_flag = 0,clean_key_wake_up_flag = 0;//go_shut_down_flag = 0;
	uint16_t clean_key_cnt = 0,sleep_key_cnt = 0;	
	uint8_t  spk_dy_flag = 1;

	
	Set_LED_On_Switch(0,0,0,0,0,0);
	Set_LED_On_Blink(0,0,0,0,0,0);
	Rcon_ResetRemoteCode();	
	while(Key_GetStatus())
	{
		sleep_key_cnt++;
		if(sleep_key_cnt>200)break;
		osDelay(10);
	}
	while(Mode_GetMode() == MODE_SLEEP)
	{	 
	
		
//		Power_DisableAll();	
//		Standby_RTC();		
//		Standby_Configuration();		 			
//		Standby_EnterStopMode();	
//		Standby_ExitConfiguration();		
		
						
		osDelay(50);							
		if((Key_GetStatus() == KEY_CLEAN))
		{
			while((Key_GetStatus() == KEY_CLEAN))
			{
			  clean_key_cnt++;
				if(clean_key_cnt >= 2)//0.125s + 0.375s = 0.5s
				{		
					if(spk_dy_flag)
					{
						spk_dy_flag = 0;
						Usprintf("%s(%d):clean key wake up success!!!\n",__FUNCTION__, __LINE__); 
					}         					
					back_to_interface_flag = 1;	
          clean_key_wake_up_flag = 1;					
				}
			}				
		}	
		/*remote*/
		if(Rcon_GetRemoteCode() == Remote_Clean)
		{			
			Rcon_ResetRemoteCode();
			Usprintf("%s(%d):remote wake up success!!!\n",__FUNCTION__, __LINE__);
			back_to_interface_flag = 1;		
		}
		
		/*rtc alarm*/	
	/*if(RTC_IsAlarm())
		{			
			RTC_ResetAlarmFlag();
			Standby_WakeUpConfiguration(); 	
			Speaker_Initialize();	
			delay(100);	//for adc  ,battery led
			Usprintf("%s(%d):rtc alarm!!! battery voltage:%d\n",__FUNCTION__, __LINE__,Battery_GetVoltage());
			delay(500);	//40ms for battery voltage detect
			if(Battery_GetVoltage() <= LOW_POWER_VOLTAGE)//low power voltage
			{
				Usprintf("%s(%d):low power ,shut down!!!\n",__FUNCTION__, __LINE__);
		   	delay(100);	//for usart						
				Power_KillAllVcc();
			}
		}*/		
		/*back to user interface mode*/	
		if(back_to_interface_flag)
		{
		  back_to_interface_flag = 0;
			Standby_WakeUpConfiguration();	
			osDelay(100);				
			System_StoreMotorBaseline();
			
			Usprintf("%s(%d):remote code or clean key wake up!\n",__FUNCTION__, __LINE__);	
			
			if(clean_key_wake_up_flag)
			{
			  clean_key_wake_up_flag = 0;
				
				if(Error_GetCode() == ERROR_NONE)
				{
					Usprintf("%s(%d):speak select mode!\n",__FUNCTION__, __LINE__);	

				}			
			}					
			Mode_SetMode(MODE_USERINTERFACE);
			Set_LED_On_Switch(1,1,1,Get_Bat_Green(),Get_Bat_Red(),0);
			osDelay(500);
			break;				
		}		
	}
}
void Standby_Configuration(void)
{			
	Standby_EXTI_Configuration();	
  Standby_GPIO_Configuration();			
	Standby_NVIC_Configuration();
	Standby_ADC_Configuration();	
	Standby_RCC_Configuration();			
	Standby_ClearEXTIPendings();	
}
void Standby_ADC_Configuration(void)
{
//	ADC1->CR &=~ADC_CR_ADEN;	
//	DMA1_Channel1->CCR &=~ DMA_CCR_EN;
}
void Standby_EXTI_Configuration(void)
{							
//	SYSCFG->EXTICR[0] = 0x0133;//B2/D1/D0
//	SYSCFG->EXTICR[1] = 0x0444;//E6/E5/E4
//	SYSCFG->EXTICR[2] = 0x0000;
//	SYSCFG->EXTICR[3] = 0x0420;//E14/C13
//	
//	EXTI->IMR   = 0x00026077;
//	EXTI->EMR   = 0x00026077;
//	EXTI->RTSR  = 0x00026077;
//	EXTI->FTSR  = 0x00026077;	  	
}
void Standby_GPIO_Configuration(void)
{

}
void Standby_NVIC_Configuration(void)
{

}
void Standby_RCC_Configuration(void)
{	

}

void Standby_ExitConfiguration(void)  
{
	EXTI_Configuration();	
//	ADC1->CR |=ADC_CR_ADEN;
//	DMA1_Channel1->CCR |= DMA_CCR_EN;	
	NVIC_Configuration();
	Power_EnableAll();	
}
void Standby_WakeUpConfiguration(void) 
{	

}
void Standby_ClearEXTIPendings(void)
{
//	EXTI->PR |=0X007BFFFF;
}
void Standby_EnterStopMode(void)
{
//	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);	 
}
void Standby_RTC(void)//adjust the rtc alarm with different battery voltages
{	
//	RTC_SetCurrentTime(0,0);
//	RTC_SetWeekDay(0);
//	RTC_SetAlarmTime(0,30);	
}


