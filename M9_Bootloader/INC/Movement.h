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
#include "config.h"


#define Check_Left_Wheel		0x01
#define Check_Right_Wheel 	0x02
#define Check_Main_Brush    0x08
#define Check_Vacuum   	  	0x10
#define Check_Left_Brush		0x20
#define Check_Right_Brush		0x40
#define Error_Short_Circuit 0x80

#define Wheel_Stall_LimitH        (uint16_t)800//580
#define Wheel_Stall_LimitL        (uint16_t)10

#define SideBrush_Stall_LimitH    (uint16_t)310//310-250ma //400 // 322ma
#define SideBrush_Stall_LimitL    (uint16_t)10
#define SideBrush_Short_Circuit   (uint16_t)1600 // //400 // 322ma*4
#define SideBrush_Stall_BaseLimit (uint16_t)750

#define MainBrush_Stall_LimitH    (uint16_t)1150 //1150 //0.926A 993//0.8A 
#define MainBrush_Stall_LimitL    (uint16_t)10
#define MainBrush_Short_Circuit   (uint16_t)2482 // 2a
#define MainBrush_Stall_BaseLimit (uint16_t)2300 // 2a

#define Vacuum_Stall_LimitH       (uint16_t)1500 // 1500 //1A
#define Vacuum_Stall_LimitL       (uint16_t)10
#define Vacuum_Short_Circuit      (uint16_t)2482 // 2a



#define Status_Left_Wall    (uint8_t)0x01
#define Status_Left_OBS     (uint8_t)0x02
#define Status_Left_OBS_2   (uint8_t)0x04

#define Status_Front_OBS    (uint8_t)0x80
#define Status_Right_OBS_2  (uint8_t)0x40
#define Status_Right_OBS    (uint8_t)0x20
#define Status_Right_Wall   (uint8_t)0x10


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

uint8_t Get_Error_Code(void);
void Set_Error_Code(uint8_t Code);
void Reset_Error_Code(void);

void Reset_Bumper_Error(void);
uint8_t Is_Bumper_Fail(void);



void Turn_Left(uint16_t speed,uint16_t angle);
void Turn_Right(uint16_t speed,uint16_t angle);
void Move_Back(void);



uint8_t Get_Bumper_Status(void);
void Disable_Motors(void);


void Forward(uint8_t Speed,uint16_t Distance);


void Initialize_Motor(void);

void Set_Turn_Check_Cliff(uint8_t data);
uint8_t Get_Turn_Check_Cliff(void);

uint8_t Self_Check(uint8_t Check_Code);
uint8_t Check_Motor_Current(void);






uint8_t Get_Random_Factor(void);

uint8_t Check_Bat_SetMotors(uint32_t Vacuum_Voltage,uint32_t Side_Brush,uint32_t Main_Brush);

uint8_t Check_Battery(void);

uint16_t GetBatteryVoltage(void);



void Quick_Back(uint8_t Speed,uint16_t Distance);


void Set_SideBrush_PWM(uint16_t L,uint16_t R);

void Set_MainBrush_PWM(uint8_t Power);

void Set_Right_Brush(uint8_t R);
uint8_t Is_RightBrush_Enable(void);


uint8_t Is_Bumper_Jamed(void);



void Set_3v3(uint8_t data);
void Set_5v(uint8_t data);
void Enable_PPower(void);
void Disable_PPower(void);


uint8_t Check_LeftWheel_Current(void);
uint8_t SelfCheck_LeftWheel(void);

uint8_t Check_RightWheel_Current(void);
uint8_t SelfCheck_RightWheel(void);


void Set_LeftBrush_Stall(uint8_t L);
void Set_RightBrush_Stall(uint8_t R);
void Set_MainBrush_Stall(uint8_t M);
uint8_t Get_MainBrush_Stall(void);
uint8_t Get_LeftBrush_Stall(void);
uint8_t Get_RightBrush_Stall(void);

void Set_Main_Brush(uint8_t M);
uint8_t Is_MainBrush_Enable(void);


uint8_t Is_Vacuum_Stall(void);
uint8_t SelfCheck_Vacuum(void);
uint8_t Is_MainBrush_Stall(void);
uint8_t SelfCheck_MainBrush(void);

void Work_Motor_Configure(void);


int32_t ABS_Minus(int32_t A,int32_t B);
 

void Reset_Short_Circuit_Counter(void);
void Check_Short_Circuit(void);


void adjust_Short_Limit(uint32_t Bat_V);
void Adjust_RightBrush_StallLimit(int32_t Bat_V);
void Adjust_MainBrush_StallLimit(int32_t Bat_V);
int32_t Get_Temp_BatVoltage(void);	



/*mode functions*/
void Mode_SetMode(CleanMode_t mode);
CleanMode_t Mode_GetMode(void);
void Mode_UpdateLog(CleanMode_t mode);
CleanMode_t Mode_GetLog(uint8_t idx);
void Mode_SetModeSelection(CleanMode_t mode);
CleanMode_t Mode_GetModeSelection(void);
void Mode_SetModeAndSelection(CleanMode_t mode);











extern volatile uint8_t g_ad_finish_flag;
extern volatile uint8_t g_random_counter;


#endif /* __Movement_H */

