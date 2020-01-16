/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2011
  * @brief   System Initialize
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __Standby_H
#define __Standby_H

#include "SysInitialize.h"


#define LOW_POWER_VOLTAGE       1300
#define LOW_POWER_RTC_VOLTAGE   1350
void Standby_GPIO_PreConfiguration(void);
void Standby_Mode(void);
void Standby_Configuration(void);
void Standby_EXTI_Configuration(void);
void Standby_GPIO_Configuration(void);
void Standby_NVIC_Configuration(void);
void Standby_RCC_Configuration(void);
void Standby_ClearEXTIPendings(void);
void Standby_EnterStopMode(void);
void Standby_WakeUpConfiguration(void);
void Standby_ADC_Configuration(void);
void Standby_ExitConfiguration(void);
void Standby_RTC(void);
#endif /*----Behaviors------*/






