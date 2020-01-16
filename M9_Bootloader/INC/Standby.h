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


#define SCB_SysCtrl              ((uint32_t)0xE000ED10)
#define SysCtrl_SLEEPDEEP_Set    ((uint32_t)0x00000004)
#define SysCtrl_SLEEPONEXIT_Set  ((uint32_t)0x00000002)

void Standby_Mode(void);
void Stanby_Configuration(void);
void Stanby_EXTI_Configuration(void);
void Stanby_GPIO_Configuration(void);
void Stanby_NVIC_Configuration(void);
void Stanby_RCC_Configuration(void);
void Disable_ADC(void);
void Clear_EXTI_Pendings(void);
void Enter_Stop_Mode(void);
void WakeUp_Configuration(void);
void Stanby_ADC_Configuration(void);


#endif /*----Behaviors------*/





