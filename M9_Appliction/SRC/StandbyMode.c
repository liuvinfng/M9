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
#include "WIFI_EMW_3081.h"
#include "Gyro.h"
#include "Charge.h"

/*----------------------------------------------------------------Standby mode---------------------------*/
void Standby_Mode(void)
{
  uint8_t time=0;
	static uint32_t Ch_WP_Counter=0;

 
	time=0;
  
  USART_Print("\n\r Sleep! ",Dev_USART3);
	
	delay(500);
	
	while(Get_Key_Press()&KEY_CLEAN)
	{
		delay(1000);
		time++;
		if(time>25)break;
	}
  Disable_PPower();
	
 
 
 
 
	
	Gyro_Sleep_Cmd();
	
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
		if(Get_Rcon_Status()&0x00044f00)
		{
			Reset_Rcon_Status();
			Ch_WP_Counter++;
			if(Ch_WP_Counter>50)
			{
				Ch_WP_Counter=0;
				Set_Clean_Mode(Clean_Mode_GoHome);
				Enable_PPower();
				SetHomeRemote();
				Set_LED_On_Blink(0,0,0);
				Set_LED_On_Switch(0,1,0);
				Wake_Up_Adjust();
				return;
			}
		}
		delay(500);
	}
	#endif	
	#ifndef WD_ENABLE	
	
	
	
  Stanby_Configuration();//configure the per for stanby mode
	
	WIFI_POWER_OFF;
	OBS_Disable;
	GPIOC->BRR |= MCU_OBS_CTRL;
	/*--------------------------------ENTER LOW POWER--------------------------*/
	
	Enter_Stop_Mode();
	/*---------------------------------Wake Up-------------------------------*/
	WakeUp_Configuration();
	Reset_Touch();
	Set_LED(0,0,0);
	delay(1000);
	
	if(Get_Key_Press()==KEY_CLEAN)
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
	
	if(Is_Alarm())
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
		if(Get_Rcon_Status()&0x00044f00)
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
				Set_LED_On_Blink(0,0,0);
				Set_LED_On_Switch(0,1,0);
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
	#else	
	if(Is_ChargerOn())
	{
		Ch_WP_Counter=0;
		Set_Clean_Mode(Clean_Mode_Charging);
		#ifdef WIFI_EMW3081
		Period_WIFI_Report(0);
		Check_APP_Command();
		Set_Display_WIFI_LED_Status(1);
		#endif
		return;
	}
	/*-----------------Check if near the charging base-----------------------------*/
	if(Get_Rcon_Status()&0x00044f00)
	{
		Reset_Rcon_Status();
		Ch_WP_Counter++;
		if(Ch_WP_Counter>50)
		{
			Ch_WP_Counter=0;
			Set_Clean_Mode(Clean_Mode_GoHome);
			Enable_PPower();
			SetHomeRemote();
			Set_LED_On_Blink(0,0,0);
			Set_LED_On_Switch(0,1,0);
			Wake_Up_Adjust();
			#ifdef WIFI_EMW3081
			Period_WIFI_Report(0);
			Check_APP_Command();
			Set_Display_WIFI_LED_Status(1);
			#endif
		  return;
		}
	}
	#endif
	#endif
	Set_Clean_Mode(Clean_Mode_Sleep);

}
void WakeUp_Configuration(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	EXTI_Configuration();
	RCC->CFGR &= ~0X0a0;
	Timer6_Configuration();

	STK_CSR=0X07;
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN|RCC_APB2ENR_ADC1EN|RCC_APB2ENR_TIM1EN
									|RCC_APB2ENR_USART1EN;
									
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN|RCC_APB1ENR_TIM6EN;

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN
									|RCC_AHBENR_GPIOCEN|RCC_AHBENR_GPIODEN
									|RCC_AHBENR_DMA1EN;//|RCC_AHBENR_GPIOFEN
									
	TIM1->CR1 |= TIM_CR1_CEN;	
	TIM3->CR1	|= TIM_CR1_CEN;
	
	#ifdef BLDC_INSTALL
	TIM16->CR1 |= TIM_CR1_CEN;
	#endif

	ADC1->CR |= ADC_CR_ADEN;
	USART1->CR1 |= USART_CR1_UE;
	
	Enable_PPower();
}


void Stanby_Configuration(void)
{
	Stanby_EXTI_Configuration();
	
	GPIOC->MODER &= ~GPIO_MODER_MODER2;
	
	TIM1->CR1 &= ~TIM_CR1_CEN;
	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM14->CR1 &= ~TIM_CR1_CEN;
	TIM15->CR1 &= ~TIM_CR1_CEN;
	TIM16->CR1 &= ~TIM_CR1_CEN;
	TIM17->CR1 &= ~TIM_CR1_CEN;
	
	ADC1->CR &= ~ADC_CR_ADEN;
	USART1->CR1 &= ~USART_CR1_UE;
	
	STK_CSR=0;
	
	RCC->APB2ENR = 0;//RCC_APB2ENR_SYSCFGEN;
											
	RCC->APB1ENR = RCC_APB1ENR_TIM6EN;
	RCC->CFGR |= 0X0a0;

	TIM6->PSC = (uint16_t)0x0005;
	
	TIM6->CR1 &= ~TIM_CR1_CEN;

	GPIOA->MODER = GPIO_MODER_MODER0|GPIO_MODER_MODER1|GPIO_MODER_MODER2|GPIO_MODER_MODER3
								|GPIO_MODER_MODER4|GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7
								|GPIO_MODER_MODER8|GPIO_MODER_MODER9|GPIO_MODER_MODER10|GPIO_MODER_MODER11
						  	|GPIO_MODER_MODER13_1|GPIO_MODER_MODER14_1;
	GPIOB->MODER = GPIO_MODER_MODER0|GPIO_MODER_MODER1
								|GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7//
								|GPIO_MODER_MODER8
								|GPIO_MODER_MODER13|GPIO_MODER_MODER14;
	GPIOC->MODER = GPIO_MODER_MODER0|GPIO_MODER_MODER1|GPIO_MODER_MODER3
								|GPIO_MODER_MODER4|GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7
								|GPIO_MODER_MODER8;

	Clear_EXTI_Pendings();
}
/*------------------------------------------Stanby ADC Configure---------------------------*/
void Stanby_ADC_Configuration(void)
{
	ADC1->CR &=~ADC_CR_ADEN;										
	DMA1_Channel1->CCR &=~ DMA_CCR_EN;
}

/*------------------------------------------Stanby NVIC Configure---------------------------*/
void Stanby_EXTI_Configuration(void)
{			
	SYSCFG->EXTICR[0] = 0x0200;//C2
	SYSCFG->EXTICR[1] = 0x0001;//B4
	SYSCFG->EXTICR[2] = 0x1200;//C10/B11
	SYSCFG->EXTICR[3] = 0x2000;//C15
	
	EXTI->IMR  =  0x00028C14;//
	EXTI->EMR  =  0x00028C14;
	EXTI->RTSR  = 0x00028C14;
	EXTI->FTSR  = 0x00028C14;		  
}
/*------------------------------------------------------Stanby GPIO--------------------------------------*/
void Stanby_GPIO_Configuration(void)
{
	/*----------------------------------- GPIOA -----------------------------------------*/
	GPIOA->MODER = GPIO_MODER_MODER13_1|
	               GPIO_MODER_MODER14_1|GPIO_MODER_MODER15_0;

	/*----------------------------------- GPIOB -----------------------------------------*/
  GPIOB->MODER = GPIO_MODER_MODER8_0|GPIO_MODER_MODER11_0;
	/*----------------------------------- GPIOC -----------------------------------------*/
	GPIOC->MODER = 0;

}
/*------------------------------------------------------Stanby NVIC--------------------------------------*/
void Stanby_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI0_1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI2_3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
//	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPriority=1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
//  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority=3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/*------------------------------------------------------Stanby RCC--------------------------------------*/
void Stanby_RCC_Configuration(void)
{

}
void Disable_ADC(void)
{
	ADC1->CR &= ~ADC_CR_ADEN;
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


