/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2011
  * @brief   Test Mode define
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __TestMode_H
#define __TestMode_H

#include "SysInitialize.h"

#define Cliff_Max           (int16_t)1000
#define Obs_Limit           (int16_t)300
#define Obs_Max             (int16_t)1000

void Test_Mode(void);
void Sound(uint8_t number);
uint32_t Get_Current_Voltage(void);
uint32_t Get_Current_Adc(void);
uint8_t Switch_Step(uint8_t Step,uint8_t dir);
void Motor_Test_Set(void);

#endif /*----Behaviors------*/





