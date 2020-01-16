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

#ifndef __WHEEL_H
#define __WHEEL_H

#include "SysInitialize.h"
#include "Mymath.h"


#define WHEEL_STALL_LIMIT         (uint16_t)1200  //500mA
#define WHEEL_STALL_NOLOAD        (uint16_t)10	 //10mA 




typedef enum{
	WHEEL_DIR_LEFT			=	0x01,
	WHEEL_DIR_RIGHT			=	0x02,
	WHEEL_DIR_FORWARD		=	0x04,
	WHEEL_DIR_BACKWARD	=	0x08,
}WheelDir_t;

typedef enum 
{
	MOVE_ACT_NONE							=0,
	MOVE_ACT_STATIC 					=	1,
	MOVE_ACT_TURN_LEFT 				=	2,
	MOVE_ACT_TURN_RIGHT 			=	3,
	MOVE_ACT_HALF_TURN_LEFT		=	4,
	MOVE_ACT_HALF_TURN_RIGHT	= 5,
	MOVE_ACT_FORWARD 					=	6,
	MOVE_ACT_BACK 						=	7,
	MOVE_ACT_DECELERATE 			= 8,
	MOVE_ACT_BREAK						=	9,
	MOVE_ACT_HALF_STATIC			=	10,
	MOVE_ACT_TURN							=	11,
	MOVE_ACT_STRAIGHT					=	12,
	MOVE_ACT_BYPASS_LEFT			=	13,
	MOVE_ACT_BYPASS_RIGHT			=	14,
	MOVE_ACT_CURVE						=	15,
	MOVE_ACT_HEAD2COURCE			=	16,
	MOVE_ACT_MOVE2POINT				=	17,
	MOVE_ACT_WALL    					= 18,
	MOVE_ACT_HANDLER					=	19,
	MOVE_ACT_STARTUP					= 20,
	MOVE_ACT_ANTI_SLIP				= 21,
	MOVE_ACT_SEARCHWALL       = 22,
	MOVE_ACT_TURN_ROUND       = 23,
	MOVE_ACT_HOMEPATH         = 24,
	MOVE_ACT_HOMEAROUND       = 25,	
	MOVE_ACT_HEAD2BASELEFT    = 26,
	MOVE_ACT_HEAD2BASERIGHT   = 27,
	MOVE_ACT_SEARCHSTATION    = 28,
}MoveAct_t;

typedef struct
{
	MoveAct_t state;
	uint32_t step;
	uint16_t l_speed;
	uint16_t r_speed;
}Move_t;

extern volatile int32_t  g_wallfollow_right_wheel_step,g_wallfollow_left_wheel_step;
extern volatile uint32_t g_wallfollow_acccelerate;
extern volatile uint16_t g_left_wheel_speed, g_right_wheel_speed;
extern volatile uint16_t g_left_speed_cnt, g_right_speed_cnt;
extern volatile uint16_t g_right_wheel_slow, g_left_wheel_slow;
extern volatile int16_t g_temp_right_wheel_pwm, g_temp_left_wheel_pwm;
extern volatile uint32_t g_left_wheel_step, g_right_wheel_step;
extern volatile int16_t g_left_move_step, g_right_move_step;
extern volatile int32_t g_right_wheel_cnt,g_left_wheel_cnt;


void Wheel_SetLeftCurrent(int16_t current);
int16_t Wheel_GetLeftCurrent(void);
void Wheel_SetRightCurrent(int16_t current);
int16_t Wheel_GetRightCurrent(void);
uint16_t Wheel_GetLeftCurrentAdc(void);
uint16_t Wheel_GetRightCurrentAdc(void);

uint8_t Wheel_CheckLeftCurrent(void);
uint8_t Wheel_CheckRightCurrent(void);


void Wheel_TuneSpeedProcess(int8_t l_wheel_speed, int8_t r_wheel_speed);
void Wheel_SpeedAccelerationProcess(void);

void Wheel_ResetSlowCnt(void);

void Set_Dir_Forward(void);


void Wheel_SetSpeed(uint8_t Left,uint8_t Right);
void Wheel_SetLeftSpeed(uint8_t Speed);
void Wheel_SetRightSpeed(uint8_t Speed);
int8_t Wheel_GetLeftSpeed(void);
int8_t Wheel_GetRightSpeed(void);

void Wheel_SetTargetSpeed(uint8_t l_speed,uint8_t r_speed);
void Wheel_SetLeftTargetSpeed(uint8_t l_speed);
void Wheel_SetRightTargetSpeed(uint8_t r_speed);
uint8_t Wheel_GetLeftTargetSpeed(void);
uint8_t Wheel_GetRightTargetSpeed(void);


void Wheel_ResetStep(void);
uint32_t Wheel_GetLeftStep(void);
uint32_t Wheel_GetRightStep(void);

void Wheel_SetTargetStep(uint32_t l_step,uint32_t r_step);
void Wheel_SetLeftTargetStep(uint32_t step);
void Wheel_SetRightTargetStep(uint32_t step);
uint32_t Wheel_GetLeftTargetStep(void);
uint32_t Wheel_GetRightTargetStep(void);
uint8_t Wheel_LeftStepReached(int32_t step);
uint8_t Wheel_RightStepReached(int32_t step);


/*move step*/
void Wheel_ResetMoveStep(void);
int32_t Wheel_GetLeftMoveStep(void);
int32_t Wheel_GetRightMoveStep(void);


void Wheel_SetCount(int32_t left, int32_t right);
int32_t Wheel_GetLeftCount(void);
int32_t Wheel_GetRightCount(void);


void Stop_Brifly(void);
void Wheel_Stop(void);
void Speed_Stop(void);
void WHEEL_DISABLE(void);
void LW_DIR_FORWARD(void);
void LW_DIR_BACKWARD(void);
void RW_DIR_FORWARD(void);
void RW_DIR_BACKWARD(void);
void Move_Forward(uint8_t Left_Speed,uint8_t Right_Speed);


void Wheel_SetDir(WheelDir_t dir);
WheelDir_t Wheel_GetDir(void);
void Wheel_SetLeftDir(WheelDir_t dir);
void Wheel_SetRightDir(WheelDir_t dir);
WheelDir_t Wheel_GetLeftDir(void);
WheelDir_t Wheel_GetRightDir(void);



/*action functions*/
void Action_MoveToPoint(Point32_t point,uint32_t speed);
void Action_MoveForward(uint8_t l_speed, uint8_t r_speed, uint32_t step, MoveAct_t act);
void Action_Turn(uint8_t l_speed,uint8_t r_speed,uint32_t angle,MoveAct_t dir);
void Action_MoveBack(uint8_t l,uint8_t r,uint32_t step, MoveAct_t act);
void Action_Deceleration(void);
void Action_Head2Cource(void);
void Action_Break(void);
void Action_Bypass(uint8_t dir);
void Action_AntiSlip(uint8_t l,uint8_t r,uint32_t step);
void Action_StopBrifly(void);
void Action_SetMove(MoveAct_t act);
MoveAct_t Action_GetMove(void);
uint8_t Action_WaitForMoveEnded(void);
void ActList_Add_Action(MoveAct_t action);
void ActList_Add(MoveAct_t act_mode,uint32_t act_step,uint8_t l_speed,uint8_t r_speed);
Move_t ActList_GetNext(void);
void ActList_Clear(void);
uint8_t ActList_GetCnt(void);
void ActList_Switch(void);
void ActList_DecelerateToStop(void);
void ActList_BackToStop(uint16_t back_distance);
void ActList_WallOffEdgeStop(WallDir_t wall_dir);
void ActList_WallStraightAndForward(MoveAct_t decelerate, uint32_t act_step);
void ActList_WallSlipStraightAndForward(MoveAct_t action);

/*turn slip functions*/
void TurnSlip_DetectProcess(FunctionalState state);
void TurnSlip_SetCheckState(FunctionalState state);
FunctionalState TurnSlip_GetCheckState(void);
uint8_t TurnSlip_IsSlip(void);
void TurnSlip_ResetSlipCnt(void);

void Wheel_TuneLeftSpeedDir(int32_t *speed,int32_t speed_limit);
void Wheel_TuneRightSpeedDir(int32_t *speed,int32_t speed_limit);

void Action_Stop(void);

void ActList_Add_Forward(uint32_t dis,uint16_t spd_l,uint16_t spd_r);

void Wheel_Forward_PidProcess(Decelerate_Type_t slow_flag,int16_t rotate_angle,int32_t *spdL,int32_t *spdR);

#endif /* __WHEEL_H */






