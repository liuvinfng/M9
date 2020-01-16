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

#ifndef __RandomRuning_H
#define __RandomRuning_H

#include "SysInitialize.h"

#define Bumper_Turn_Right     0x01
#define Bumper_Turn_Left      0x02

#define Small_Wall_Distance   (int32_t)15000
#define Middle_Wall_Distance  (int32_t)20000

#define Short_Wall_Trig       11 

void Random_Running_Mode(void);
uint8_t Out_Trap_Right(void);
uint8_t Out_Trap_Left(void);
uint8_t Left_Bumper_Avoiding(void);
uint8_t Right_Bumper_Avoiding(void);
void Half_Turn_Right(uint16_t speed,uint16_t angle);
void Half_Turn_Left(uint16_t speed,uint16_t angle);
void Set_HalfTurn_Flag(void);
void Reset_HalfTurn_Flag(void);
uint8_t Is_HalfTurn_Flag(void);



#endif /*----Behaviors------*/





