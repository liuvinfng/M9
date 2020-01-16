/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2011
  * @brief   Movement
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __BRUSH_H
#define __BRUSH_H

#include "SysInitialize.h"


#define SIDE_BRUSH_STALL_LIMIT    			(uint16_t)40   //40mA
#define SIDE_BRUSH_STALL_NOLOAD    			(uint16_t)6    //6mA 
#define SIDE_BRUSH_STALL_SHORTCIRCUIT   (uint16_t)2482 // 2a
#define MAIN_BRUSH_STALL_LIMIT    			(uint16_t)1620  //920mA
#define MAIN_BRUSH_STALL_NOLOAD    			(uint16_t)100  //100mA 
#define MAIN_BRUSH_STALL_SHORTCIRCUIT   (uint16_t)2482 // 2a

void Brush_Main_SetPWM(uint8_t pwm);
void Brush_Side_SetPWM(uint8_t pwm);

void Side_Brush_SetCurrent(int16_t current);
int16_t Side_Brush_GetCurrent(void);
uint16_t Side_Brush_GetCurrentAdc(void);

void Main_Brush_SetCurrent(int16_t current);
int16_t Main_Brush_GetCurrent(void);
uint16_t Main_Brush_GetCurrentAdc(void);

uint8_t Side_Brush_CheckCurrent(void);
uint8_t Main_Brush_CheckCurrent(void);


#endif /* __BRUSH_H */






