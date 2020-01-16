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

#ifndef __OBSCLIFF_H
#define __OBSCLIFF_H

#include "SysInitialize.h"
#include "config.h"

#define Cliff_Limit         (int16_t)70


extern volatile int16_t g_leftwall_baseline;
extern volatile int16_t g_rightwall_baseline;
extern volatile int16_t g_xpwall_baseline;


void Set_LWall_Base(int32_t data);
int32_t Get_LWall_Base(void);

void Set_RWall_Base(int32_t data);
int32_t Get_RWall_Base(void);

void Set_Xp_RWall_Base(int32_t data);
int32_t Get_Xp_RWall_Base(void);

int32_t Get_LWall_ADC(void);
int32_t Get_RWall_ADC(void);
int32_t Get_Xp_RWall_ADC(void);
int32_t Get_LeftOBS(void);
int32_t Get_RightOBS(void);
int32_t Get_FrontOBS(void);
int16_t Get_FrontOBST_Value(void);
int16_t Get_LeftOBST_Value(void);
int16_t Get_RightOBST_Value(void);
uint8_t Get_OBS_Status(void);
uint8_t Get_Cliff_Trig(void);



#endif /* __OBSCLIFF_H */






