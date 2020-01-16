

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



volatile uint8_t g_alarm_flag=0;
volatile uint64_t Clock_Value=0;
volatile uint8_t Clock_Receive_Flag=0;


/*-------------------------------Alarm Flag-----------------------------------*/
void RTC_ResetAlarm(void)
{
  g_alarm_flag=0;
}
uint8_t RTC_IsAlarm(void)
{
  if(g_alarm_flag==1)return 1;
	return 0;
}


/*------------------------------Set RTC Time-----------------------------*/
void RTC_SetCurrentTime(uint32_t Hour,uint32_t Minute)
{
	uint32_t T=0;
	uint16_t RTC_S_L=0,RTC_S_H=0;
	uint32_t Waiting=0;
	
	T = Hour*3600 + Minute*60;
	RTC_S_L = T;
	T>>=16;
	RTC_S_H = T;
	
	while(!(RTC_CTL & RTC_CRL_RTOFF))
	{
		Waiting++;
		if(Waiting>100000)break;
	}
	RTC_CTL |= RTC_CRL_CNF;	
	RTC_CNTH = RTC_S_H;
	RTC_CNTL = RTC_S_L;
	RTC_CTL &= ~RTC_CRL_CNF;
	Waiting = 0;
	while(!(RTC_CTL & RTC_CRL_RTOFF))
	{
		Waiting++;
		if(Waiting>100000)break;
	}
}

void RTC_SetAlarmTime(uint32_t Hour,uint32_t Minute)
{
	uint32_t T=0;
	uint16_t RTC_S_L=0,RTC_S_H=0;
	uint32_t Waiting=0;
	
	T = Hour*3600 + Minute*60;
	RTC_S_L = T;
	T>>=16;
	RTC_S_H = T;

	while(!(RTC_CTL & RTC_CRL_RTOFF))
	{
		Waiting++;
		if(Waiting>100000)break;
	}
	RTC_CTL |= RTC_CRL_CNF;
	RTC_ALRMH = RTC_S_H;
	RTC_ALRML = RTC_S_L;		
	
	RTC_INTEN |= RTC_CRH_ALRIE;
	RTC_CTL &= ~RTC_CRL_CNF;
	Waiting = 0;
	while(!(RTC_CTL & RTC_CRL_RTOFF))
	{
		Waiting++;
		if(Waiting>100000)break;
	}	
}

void RTC_DisableAlarm(void)
{
	RTC_INTEN &= ~RTC_CRH_ALRIE;
}
void RTC_Configuration(void)
{
//	uint32_t Waiting=0;
//	RCU_APB1EN |=RCC_APB1ENR_PWREN|RCC_APB1ENR_BKPEN;
//	PMU_CTL |=PWR_CR_DBP;
//	RCU_BDCTL |=RCC_BDCR_RTCEN|RCC_BDCR_RTCSEL;

//	while(!(RTC_CTL & RTC_CRL_RTOFF))
//	{
//		Waiting++;
//		if(Waiting>100000)break;
//	}
//	RTC_CTL |= RTC_CRL_CNF;
//	RTC_PSCH = 0x0000;
//	RTC_PSCL = 0xF423;
//	RTC_CTL &=~RTC_CRL_CNF;

//	Waiting = 0;
//	while(!(RTC_CTL & RTC_CRL_RTOFF))
//	{
//		Waiting++;
//		if(Waiting>100000)break;
//	}

//	uint32_t Waiting=0;
	RCU_APB1EN |=RCC_APB1ENR_PWREN|RCC_APB1ENR_BKPEN;
	PMU_CTL |=PWR_CR_DBP;
	RCU_BDCTL &= ~RCC_BDCR_LSEON;
	RCU_BDCTL |= RCC_BDCR_LSEON;
	while(!(RCU_BDCTL&RCC_CSR_LSIRDY));
	RCU_BDCTL &= ~RCC_BDCR_RTCSEL;
	RCU_BDCTL |= RCC_BDCR_RTCSEL_0;
//	RCU_BDCTL |=RCC_BDCR_RTCEN|RCC_BDCR_RTCSEL_0;	
//	while(!(RTC_CTL & RTC_CRL_RTOFF))
//	{
//		Waiting++;
//		if(Waiting>100000)break;
//	}
//	RTC_CTL |= RTC_CRL_CNF;
//	RTC_PSCH = 0x0000;
//	RTC_PSCL = 0xF423;
//	RTC_CTL &=~RTC_CRL_CNF;

//	Waiting = 0;
//	while(!(RTC_CTL & RTC_CRL_RTOFF))
//	{
//		Waiting++;
//		if(Waiting>100000)break;
//	}
}



void Set_RemoteClock(uint64_t data)
{
	Clock_Value = data;
	Set_ClockReceived();
}
uint64_t Read_RemoteClock(void)
{
	return Clock_Value;
}
void Set_ClockReceived(void)
{
	Clock_Receive_Flag=1;
}
void Clear_ClcokReceive(void)
{
	Clock_Receive_Flag=0;
}
uint8_t Is_ClockReceived(void)
{
	return Clock_Receive_Flag;
}


uint32_t RTC_GetCurrentMinutes(void)
{
	uint32_t Se=0;

	while(!(RTC_CTL & RTC_CRL_RTOFF)){}
  RTC_CTL &=~ RTC_CRL_RSF;
	while(!(RTC_CTL & RTC_CRL_RSF)){}
	
	Se = RTC_CNTH;
	Se<<=16;
	Se |= RTC_CNTL;
	Se /= 60;
	return Se;	
}


#ifdef SCREEN_REMOTE
void Set_Remote_Schedule(void)
{
	uint32_t Alarm_Minute=0;
	uint16_t Temp_Schedule_Minute=0;
	uint16_t Temp_Alarm_Minute = 0;
	uint8_t Buffering=0;
	uint8_t Clock_Correct_Flag=0;
	uint64_t Temp_CLock_Value=0;

  Clear_ClcokReceive();
	
	Temp_CLock_Value = Read_RemoteClock();
	
	Temp_Schedule_Minute=	Temp_CLock_Value;//get least 16bit for clock
	
	Temp_CLock_Value>>=16;
	
	Temp_Alarm_Minute = Temp_CLock_Value;//get 17-32 bit for alarm
	
	Buffering = Temp_Alarm_Minute;//get alarm corrector
	Temp_Alarm_Minute>>=8;//alarm value
	Buffering=~Buffering;

	if(Temp_Schedule_Minute&0x8000)
	{
		if(Temp_Schedule_Minute&0x0001)
		{
		  Clock_Correct_Flag|=0x01;
	  }
		else
		{
			Clock_Correct_Flag = 0 ;
		}
	}
	else
	{
		if((Temp_Schedule_Minute&0x0001)==0)
		{
			Clock_Correct_Flag|=0x01;
		}
		else
		{
			Clock_Correct_Flag = 0 ;
		}
	}
	Temp_Schedule_Minute=Temp_Schedule_Minute&0x7fff;
	if(Temp_Alarm_Minute ==	Buffering)Clock_Correct_Flag|=0x02;
	if(Clock_Correct_Flag==0x03)
	{
    Alarm_Minute = Temp_Alarm_Minute*15;	
		RTC_Configuration();
		RTC_SetCurrentTime(Temp_Schedule_Minute/60,Temp_Schedule_Minute%60);
		
		if(Alarm_Minute==0)
		{
			RTC_DisableAlarm();
		}
		else
		{
			RTC_SetAlarmTime(Alarm_Minute/60,Alarm_Minute%60);
		}
		Beep(1);
		Beep(2);
		Beep(1);
	}
}
#endif



