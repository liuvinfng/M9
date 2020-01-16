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

#ifndef __MOVEMENT_H
#define __MOVEMENT_H

#include "SysInitialize.h"


#define Check_Left_Wheel		0x01
#define Check_Right_Wheel 	0x02
#define Check_Main_Brush    0x08
#define Check_Vacuum   	  	0x10
#define Check_Left_Brush		0x20
#define Check_Right_Brush		0x40
#define Error_Short_Circuit 0x80

#define DIRECTION_FLAG_RIGHT        0x01
#define DIRECTION_FLAG_LEFT         0x02

#define Status_Left_Wall    (uint8_t)0x01
#define Status_Left_OBS     (uint8_t)0x02
#define Status_Front_OBS    (uint8_t)0x80
#define Status_Right_OBS    (uint8_t)0x20
#define Status_Right_Wall   (uint8_t)0x10

typedef enum{
	BATTERY_EMPTY = 0,
	BATTERY_LOW,
	BATTERY_FULL,
}Battery_t;

typedef enum 
{
	CHECK_L_WHEEL  		= 0x01,
	CHECK_R_WHEEL 		=	0x02,
	CHECK_SIDE_BRUSH  =	0x20,
	CHECK_MOBILITY    =	0x40,
	CHECK_MAIN_BRUSH  = 0x08,
	CHECK_VACUUM   	  =	0x10,
	CHECK_BUMPER      = 0x80,
	CHECK_OBS         = 0x04,
}MotorCheck_t;

typedef enum 
{
	MODE_NAVIGATION				=0,	
	MODE_WALL						  =1,	
	MODE_MOP        	    =2,	
	MODE_HOME				      =3,
	MODE_SPOT						  =4,
	MODE_USERINTERFACE    =5,	
	MODE_SINGLE_ROOM			=6,
	MODE_REMOTE						=7,		
	MODE_DEEP_CLEAN			  =8,	
			
	MODE_SLEEP		        =10,
	MODE_RANDOM				    =11,
	MODE_GYRO_TURN_TEST   =12,
	MODE_GYRO_RUN_TEST    =13,	
	MODE_NAVIGATION2      =14,	
	MODE_TEST             =15,		
}CleanMode_t;

typedef enum 
{
	IDLE_STATE						=0,	
	AUTO_STATE						=1,	
	MOP_STATE        	    =2,	
	WALL_STATE				    =3,
	HOME_STATE						=4,
	CHARGE_STATE          =5,	
	SPOT_STATE						=6,
	PAUSE_STATE						=7,		
	SINGLE_ROOM_STATE			=8,			
}CurrentState_t;

typedef enum
{
	ERROR_NONE					=0,
	ERROR_SIDE_BRUSH		=1,
	ERROR_MAIN_BRUSH		=2,	
	ERROR_LEFT_WHEEL		=3,
	ERROR_RIGHT_WHEEL		=4,
	ERROR_DUSTBIN				=5,	
	ERROR_CLIFF					=6,
	ERROR_BUMPER				=7,
	ERROR_WATER_TANK		=8,
	ERROR_GEOMAGNETIC		=9,	
	ERROR_STUCK					=10,
	ERROR_BATTERY				=11,
	ERROR_PICK_UP				=12,
	ERROR_ENCODER				=13,
	ERROR_FAN_H					=14,
	ERROR_FAN_L					=15,
	ERROR_GYRO					=16,
}Error_t;

typedef struct
{
	volatile uint32_t temporary;
	volatile int32_t cost; //6400*3600
	volatile uint8_t cnt;
	volatile uint16_t min;
	volatile uint16_t max;
	volatile uint8_t level;
}BatteryCapacity_t;

uint32_t Time_GetCurrentTime(void);
void Time_ResetCurrentTime(void);
void Time_IncreaseCurrentTime(void);

void User_SetQuitCheckState(FunctionalState state);
FunctionalState User_GetQuitCheckState(void);


Error_t Error_GetCode(void);
void Error_SetCode(Error_t Code);
void Error_SetCleanCode(Error_t Code);
void Error_ResetCode(void);
uint8_t Stuck_IsStucked(void);


void Bumper_ResetErrorCnt(void);
uint8_t Bumper_IsFailed(void);
void Bumper_CheckProcess(FunctionalState state);
void Bumper_SetCheckState(FunctionalState state);
FunctionalState Bumper_GetCheckState(void);




uint8_t Bumper_GetTrigStatus(void);
void Motor_DisableAll(void);
void Motor_WorkConfigure(void);
void Motor_HomeConfigure(void);
void Motor_SpotConfigure(void);


uint8_t System_GetRandomValue(void);



void Motor_CheckCurrentProcess(FunctionalState state);
uint8_t Motor_GetStatus(void);
void Motor_ResetStatus(void);
void Motor_SetCheckState(FunctionalState state);
FunctionalState Motor_GetCheckState(void);

void Motor_SetState(FunctionalState state);
FunctionalState Motor_GetState(void);
void Motor_SetPower(uint32_t vac_speed, int32_t brush_power, int32_t main_brush_power);
void Motor_TunePower(void);

void Set_3v3(uint8_t data);
void Set_5v(uint8_t data);
void Power_EnableAll(void);
void Power_DisableAll(void);




int32_t ABS_Minus(int32_t A,int32_t B);
 


void adjust_Short_Limit(uint32_t Bat_V);
void Adjust_RightBrush_StallLimit(int32_t Bat_V);
void Adjust_MainBrush_StallLimit(int32_t Bat_V);


void Battery_SetVoltage(uint16_t voltage);
int32_t Battery_GetVoltage(void);
uint8_t Battery_IsLow(void);
uint16_t Battery_GetAdcValue(void);


void Battery_SetState(Battery_t state);
Battery_t Battery_GetState(void);
void Battery_SetCheckState(FunctionalState state);
FunctionalState Battery_GetCheckState(void);



/*mode functions*/
void Mode_SetMode(CleanMode_t mode);
CleanMode_t Mode_GetMode(void);
void Mode_UpdateLog(CleanMode_t mode);
CleanMode_t Mode_GetLog(uint8_t idx);
void Mode_SetModeSelection(CleanMode_t mode);
CleanMode_t Mode_GetModeSelection(void);
void Mode_SetModeAndSelection(CleanMode_t mode);

void Wifi_Mode_SetMode(CleanMode_t mode);
CleanMode_t Wifi_Mode_GetMode(void);
void Wifi_Set_Mode_Flag(uint8_t flag);
uint8_t Wifi_Get_Mode_Flag(void);	

void Direction_SetLastDir(uint8_t Dir);
uint8_t Direction_GetLastDir(void);
uint8_t Direction_IsLastDirRight(void);
uint8_t Direction_IsLastDirLeft(void);


void Set_Loop_Ture(uint8_t ture);
uint8_t Get_Loop_Ture(void);

extern volatile uint8_t g_random_counter;

void Battery_AddCapacity_Current(uint16_t sys_current_adc);
void Battery_Current_Filter(void);
uint8_t Battery_Get_CapacityLevel(void);
void Battery_Capacity_Reset(void);


#endif /* __Movement_H */

