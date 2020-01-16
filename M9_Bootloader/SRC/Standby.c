 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2018
  * @brief   Standby Fuction , Turn off all the unworking peripheral except TIM4 and 2 gpios, waiting for
	           Button function or remote code to wakeup 
	           
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/

#include "Standby.h"
#include "Movement.h"
#include "Speaker.h"
#include "USART.h"
#include "Display.h"
#include "TouchPad.h"
#include "UserInterface.h"
#include "Rcon.h"
#include "RTC.h"
#include "Home.h"
#include "Gyro.h"
#include "Charge.h"

/*----------------------------------------------------------------Standby mode---------------------------*/
void Standby_Mode(void)
{
  uint8_t time=0;
	static uint32_t Ch_WP_Counter=0;
	Set_LED_On_Blink(0,0,0,0,0,0);
	Set_LED_On_Switch(0,0,0,0,0,0);	
	time=0;
  
  USART_Print("\n\r Sleep! ",Dev_USART3);
	
//	delay(500);
	
	while(Get_Key_Press()&KEY_CLEAN)
	{
		delay(1000);
		time++;
		if(time>25)break;
	}
  Disable_PPower();
		
//	Gyro_Sleep_Cmd();
	
	#ifdef WD_ENABLE
	while(1)
	{
		if(Get_Key_Press()==KEY_CLEAN)
		{
			Ch_WP_Counter=0;
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return;
		}
		delay(1000);
		if(Is_Alarm())
		{
			Ch_WP_Counter=0;
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return;
		}
		if(Remote_Key(Remote_Clean))
		{
			Ch_WP_Counter=0;
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return;
		}
		if(Is_ChargerOn())
		{
			Ch_WP_Counter=0;
			Set_Clean_Mode(Clean_Mode_Charging);
			return;
		}
		/*-----------------Check if near the charging base-----------------------------*/
		if(Get_Rcon_Status()&Rcon_Signal_WAKE_UP)
		{
			Reset_Rcon_Status();
			Ch_WP_Counter++;
			if(Ch_WP_Counter>50)
			{
				Ch_WP_Counter=0;
				Set_Clean_Mode(Clean_Mode_GoHome);
				Enable_PPower();
				SetHomeRemote();
				Set_LED_On_Blink(0,0,0,0,0,0);
				Set_LED_On_Switch(1,0,0,0,0,0);
				Wake_Up_Adjust();
				return;
			}
		}
		delay(500);
	}
	#endif	
	#ifndef WD_ENABLE
	
//  Stanby_Configuration();	
	WIFI_POWER_OFF;
	OBS_Disable;
	
//	Enter_Stop_Mode();
	/*---------------------------------Wake Up-------------------------------*/
//	WakeUp_Configuration();
	Enable_PPower();
	Reset_Touch();
	Set_LED(0,0,0,0,0,0);
	delay(1000);

	if(!IS_KEY_CLEAN)//if(Get_Key_Press()==KEY_CLEAN)
	{
		while(!IS_KEY_CLEAN);
		Ch_WP_Counter=0;
		#ifdef CEC_ENABLE
		Reset_ChargeSleep_Flag();
		#endif
		Set_Clean_Mode(Clean_Mode_Userinterface);
		return;
	}
	
	if(Is_Alarm())
	{
		Ch_WP_Counter=0;
		#ifdef CEC_ENABLE
		Reset_ChargeSleep_Flag();
		#endif
		Set_Clean_Mode(Clean_Mode_Userinterface);
		return;
	}
	if(Remote_Key(Remote_Clean))
	{
		Ch_WP_Counter=0;
		#ifdef CEC_ENABLE
		Reset_ChargeSleep_Flag();
		#endif
		Set_Clean_Mode(Clean_Mode_Userinterface);
		return;
	} 
 	
	#ifdef CEC_ENABLE
	if(!Get_ChargeSleep_Flag())
	{
		if(Is_ChargerOn())
		{
			Ch_WP_Counter=0;
			#ifdef CEC_ENABLE
			Reset_ChargeSleep_Flag();
			#endif
			Set_Clean_Mode(Clean_Mode_Charging);
			#ifdef WIFI_EMW3081
			Period_WIFI_Report(0);
			Check_APP_Command();
			Set_Display_WIFI_LED_Status(1);
			#endif
			return;
		}
		/*-----------------Check if near the charging base-----------------------------*/
		if(Get_Rcon_Status()&Rcon_Signal_WAKE_UP)
		{
			Reset_Rcon_Status();
			Ch_WP_Counter++;
			if(Ch_WP_Counter>50)
			{
				Ch_WP_Counter=0;
				#ifdef CEC_ENABLE
				Reset_ChargeSleep_Flag();
				#endif
				Set_Clean_Mode(Clean_Mode_GoHome);
				Enable_PPower();
				SetHomeRemote();
				Set_LED_On_Blink(0,0,0,0,0,0);
				Set_LED_On_Switch(1,0,0,0,0,0);
				Wake_Up_Adjust();
				#ifdef WIFI_EMW3081
				Period_WIFI_Report(0);
				Check_APP_Command();
				Set_Display_WIFI_LED_Status(1);
				#endif
				return;
			}
		}
	}
	else
	{
		if(!Is_ChargerOn())
		{
			Ch_WP_Counter=0;
			#ifdef CEC_ENABLE
			Reset_ChargeSleep_Flag();
			#endif
			Set_Clean_Mode(Clean_Mode_Userinterface);
			#ifdef WIFI_EMW3081
			Period_WIFI_Report(0);
			Check_APP_Command();
			Set_Display_WIFI_LED_Status(1);
			#endif
			return;
		}
		if(GetBatteryVoltage()<1600)
		{
			Ch_WP_Counter=0;
			#ifdef CEC_ENABLE
			Reset_ChargeSleep_Flag();
			#endif
			Set_Clean_Mode(Clean_Mode_Userinterface);
			#ifdef WIFI_EMW3081
			Period_WIFI_Report(0);
			Check_APP_Command();
			Set_Display_WIFI_LED_Status(1);
			#endif
			return;
		}
	}
	#endif
	#ifndef	CEC_ENABLE
	if(Is_ChargerOn())
	{
		Ch_WP_Counter=0;
		Set_Clean_Mode(Clean_Mode_Charging);
		return;
	}
	/*-----------------Check if near the charging base-----------------------------*/
	if(Get_Rcon_Status()&Rcon_Signal_WAKE_UP)
	{
		Reset_Rcon_Status();
		Ch_WP_Counter++;
		if(Ch_WP_Counter>50)
		{
			Ch_WP_Counter=0;
			Set_Clean_Mode(Clean_Mode_GoHome);
			Enable_PPower();
			SetHomeRemote();
			Set_LED_On_Blink(0,0,0,0,0,0);
			Set_LED_On_Switch(1,0,0,0,0,0);
			/*Wake_Up_Adjust();*/
		  return;
		}
	}
	#endif
	#endif
	Set_Clean_Mode(Clean_Mode_Sleep);
}
void WakeUp_Configuration(void)
{
	RCC->CFGR &= ~0X0a0;	
	RCC_Configuration();
	GPIO_Configuration();
	EXTI_Configuration();	
	Timer6_Configuration();
	STK_CTRL=0X07;
	
	ADC1->CR |= ADC_CR_ADEN;
	DMA1_Channel1->CCR |= DMA_CCR_EN;											
	TIM1->CR1  |= TIM_CR1_CEN;	
	TIM2->CR1	 |= TIM_CR1_CEN;
	TIM3->CR1	 |= TIM_CR1_CEN;	
	TIM7->CR1	 |= TIM_CR1_CEN;	
	TIM15->CR1 |= TIM_CR1_CEN;	
	TIM17->CR1 |= TIM_CR1_CEN;
	
	USART1->CR1 |= USART_CR1_UE;
	USART2->CR1 |= USART_CR1_UE;
	USART3->CR1 |= USART_CR1_UE;
	
}


void Stanby_Configuration(void)
{
	ADC1->CR &= ~ADC_CR_ADEN;	
	DMA1_Channel1->CCR &=~ DMA_CCR_EN;
	TIM1->CR1  &= ~TIM_CR1_CEN;
	TIM2->CR1  &= ~TIM_CR1_CEN;
	TIM3->CR1  &= ~TIM_CR1_CEN;
	TIM7->CR1  &= ~TIM_CR1_CEN;
	TIM15->CR1 &= ~TIM_CR1_CEN;
	TIM17->CR1 &= ~TIM_CR1_CEN;
	
	USART1->CR1 &= ~USART_CR1_UE;
	USART2->CR1 &= ~USART_CR1_UE;
	USART3->CR1 &= ~USART_CR1_UE;	
	STK_CTRL=0;
	Stanby_EXTI_Configuration();
	
	RCC->APB2ENR = 0;											
	RCC->APB1ENR = RCC_APB1ENR_TIM6EN;
	RCC->CFGR |= 0X0a0;
	TIM6->PSC = (uint16_t)0x0005;	

//	GPIOA->MODER = GPIO_MODER_MODER0|GPIO_MODER_MODER1|GPIO_MODER_MODER2|GPIO_MODER_MODER3
//								|GPIO_MODER_MODER4|GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7
//								|GPIO_MODER_MODER8|GPIO_MODER_MODER9|GPIO_MODER_MODER10|GPIO_MODER_MODER11
//						  	|GPIO_MODER_MODER13_1|GPIO_MODER_MODER14_1;
//	GPIOB->MODER = GPIO_MODER_MODER0|GPIO_MODER_MODER1
//								|GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7//
//								|GPIO_MODER_MODER8
//								|GPIO_MODER_MODER13|GPIO_MODER_MODER14;
//	GPIOC->MODER = GPIO_MODER_MODER0|GPIO_MODER_MODER1|GPIO_MODER_MODER3
//								|GPIO_MODER_MODER4|GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7
//								|GPIO_MODER_MODER8;

	Clear_EXTI_Pendings();
}

/*------------------------------------------Stanby NVIC Configure---------------------------*/
/** 
  * @摘要 外部中断初始化
	* 
	* @参数 x000:PA[x]pin  x001:PB[x]pin  x010:PC[x]pin  x011:PD[x]pin  x100:PE[x]pin  x101:PF[x] pin     
	*  			EXTICR[0](GPIO[3-0])     
	*				EXTICR[1](GPIO[7-4])     
	*				EXTICR[1](GPIO[11-8])      
	*				EXTICR[1](GPIO[15-12])     
	*				A=0;B=1;C=2;D=3;E=4;F=5
	* @返回值 无 
  */ 
void Stanby_EXTI_Configuration(void)
{			
	SYSCFG->EXTICR[0] = 0x0133;//B2/D1/D0
	SYSCFG->EXTICR[1] = 0x0444;//E6/E5/E4
	SYSCFG->EXTICR[2] = 0x0000;
	SYSCFG->EXTICR[3] = 0x0420;//E14/C13
	
	EXTI->IMR   = 0x00026077;
	EXTI->EMR   = 0x00026077;
	EXTI->RTSR  = 0x00026077;
	EXTI->FTSR  = 0x00026077;	  
}

void Clear_EXTI_Pendings(void)
{
	EXTI->PR |=0X007BFFFF;
}

void Enter_Stop_Mode(void)
{

  SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk); 
   __WFI();
	
	/*PWR->CR |= PWR_CR_PDDS|PWR_CR_LPDS;	
	SCB->SCR |=SCB_SCR_SLEEPDEEP_Msk;	
	__WFI();	
	SCB->SCR &=~ SCB_SCR_SLEEPDEEP_Msk;*/
}


