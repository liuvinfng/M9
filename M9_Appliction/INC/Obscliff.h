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




#define CLIFF_LIMIT          70

typedef enum
{
	CLIFF_NO_TRIG    			= 0,
	CLIFF_LEFT_TRIG  			= 0x01,
	CLIFF_RIGHT_TRIG 			= 0x02,
	CLIFF_FRONT_TRIG 			= 0x03,
	CLIFF_BACK_LEFT_TRIG  = 0x04,
	CLIFF_BACK_RIGHT_TRIG = 0x08,	
	CLIFF_ALL_TRIG   			= 0x0f,
}CliffTrig_t;

typedef enum
{
	OBS_NO_TRIG				= 0,
	OBS_LEFT_TRIG    	= 0x02,
	OBS_FRONT_TRIG    =	0x80,
	OBS_RIGHT_TRIG    =	0x20,
	OBS_ALL_TRIG			= 0xa2,
}ObsTrig_t;

extern volatile int16_t g_leftwall_baseline;
extern volatile int16_t g_rightwall_baseline;
extern volatile int16_t g_xpwall_baseline;

void Wall_DynamicProcess(FunctionalState state);
void Wall_SetDynamicState(FunctionalState state);
FunctionalState Wall_GetDynamicState(void);
void Wall_SetLeftBaseline(int32_t data);
void Wall_SetRightBaseline(int32_t data);
void Wall_SetXBaseline(int32_t data);
void Wall_AdjustBaseline(void);

int32_t Wall_GetRightBaseline(void);
int32_t Wall_GetLeftBaseline(void);
int32_t Wall_GetXBaseline(void);

void Wall_SetAverageAdcValue(int32_t left_average,int32_t right_average,int32_t x_average);
int32_t Wall_GetRightAverageAdcValue(void);
int32_t Wall_GetLeftAverageAdcValue(void);
int32_t Wall_GetXAverageAdcValue(void);


int32_t Wall_GetLeftAdcValue(void);
int32_t Wall_GetRightAdcValue(void);
int32_t Wall_GetXAdcValue(void);

void OBS_DynamicProcess(FunctionalState state);
void OBS_SetDynamicState(FunctionalState state);
FunctionalState OBS_GetDynamicState(void);
void OBS_AdjustTrigValue(void);
void OBS_ResetTrigValue(void);
int16_t Get_FrontOBST_Value(void);
int16_t Get_LeftOBST_Value(void);
int16_t Get_RightOBST_Value(void);
uint8_t OBS_GetTrigStatus(void);
uint8_t OBS_IsWallNear(void);
uint8_t OBS_IsNear(void);
uint8_t OBS_L_F_Near(void);
uint8_t OBS_R_F_Near(void);
void Set_near_Flag(uint8_t flag);
uint8_t OBS_SpotStatus(void);
void OBS_OverLimitCheck(uint8_t obs_val);


/*obs value*/
int32_t OBS_GetFrontValue(void);
int32_t OBS_GetLeftValue(void);
int32_t OBS_GetRightValue(void);

void Cliff_DetectionProcess(FunctionalState state);
void Cliff_SetDetectState(FunctionalState state);
FunctionalState Cliff_GetDetectState(void);
CliffTrig_t Cliff_GetDetectiontProcess_Result(void);
CliffTrig_t Cliff_GetInstantStatus(void);



#endif /* __OBSCLIFF_H */






