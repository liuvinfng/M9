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

#ifndef __HomeStraight_H
#define __HomeStraight_H

#include "SysInitialize.h"
#include "Mymath.h"

#define HOME_WALL_DISTANCE  (int32_t)30000

void HomeStraight_Mode(void);


Point32_t Random_GetNext_target(void);
Point32_t Random_GetForward_target(void);
void Adjust_Home_Path(uint32_t Receive_Code);
void Home_Around_Station(uint8_t dir);



#endif /*----Behaviors------*/





