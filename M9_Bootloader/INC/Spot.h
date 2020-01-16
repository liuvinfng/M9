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

#ifndef __Spot_H
#define __Spot_H

#include "SysInitialize.h"

#define Spiral_Right_Out	1
#define Spiral_Right_In 	2
#define Spiral_Left_Out 	4
#define Spiral_Left_In	  8
#define First_Round       10


void Spot_Mode(void);
uint8_t Random_Dirt_Event(void);

#endif /*----Behaviors------*/





