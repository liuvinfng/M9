/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2018
  * @brief   Basical Movement functions
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "main.h"
#include "charge.h"
#include "projecttask.h"
#include "userinterface.h"
#include "remote_mode.h"
#include "spot.h"
#include "standby.h"
#include "wallfollow.h"
#include "homestraight.h"
#include "cormove.h"
#include "pathplanning.h"
#include "shortestpath.h"
#include "map.h"
#include "wifi.h"
#include "movement.h"
#include "display.h"
#include "usart.h"
#include "spi.h"
#include "speaker.h"
#include "rcon.h"
#include "rtc.h"
#include "touchpad.h"
#include "gyro.h"
#include "wheel.h"
#include "obscliff.h"
#include "bldc.h"
#include "brush.h"
#include "w25q16.h"
#include "mymath.h"
#include "debug.h"
#include "cmsis_os.h"
#include "config.h"

uint8_t R_F=0, L_F=0;
volatile uint8_t g_left_target_speed=0,g_right_target_speed=0;
volatile uint16_t g_left_wheel_speed=0, g_right_wheel_speed=0;
volatile uint16_t g_left_speed_cnt=0, g_right_speed_cnt=0;
volatile uint16_t g_right_wheel_slow=0, g_left_wheel_slow=0;
volatile int16_t g_temp_right_wheel_pwm=0, g_temp_left_wheel_pwm=0;
volatile uint32_t g_left_wheel_step=0, g_right_wheel_step=0;
uint32_t g_left_wheel_taget_step=0,g_right_wheel_taget_step=0;
volatile int16_t g_left_move_step=0, g_right_move_step=0;
volatile int32_t g_right_wheel_cnt=0,g_left_wheel_cnt=0;
volatile int16_t g_left_wheel_current=0,g_right_wheel_current=0;


/*wheel current*/
void Wheel_SetLeftCurrent(int16_t current)
{
	g_left_wheel_current = current;
}
int16_t Wheel_GetLeftCurrent(void)
{
	return g_left_wheel_current;
}
void Wheel_SetRightCurrent(int16_t current)
{
	g_right_wheel_current = current;
}
int16_t Wheel_GetRightCurrent(void)
{
	return g_right_wheel_current;
}
uint16_t Wheel_GetLeftCurrentAdc(void)
{
  return g_adc_value.Left_Wheel_Current;
}
uint16_t Wheel_GetRightCurrentAdc(void)
{
  return g_adc_value.Right_Wheel_Current;
}

/*check wheel cur*/
uint8_t Wheel_CheckLeftCurrent(void)
{
	static uint8_t left_wheel_overcurrent_cnt = 0;
	int16_t temp_current = 0;	
	temp_current = Wheel_GetLeftCurrent();
	if(temp_current < WHEEL_STALL_NOLOAD)
	{

	}
  else if(temp_current > WHEEL_STALL_LIMIT)
  {
	  left_wheel_overcurrent_cnt += 5;
  }
  else
  {
	  left_wheel_overcurrent_cnt = 0;
  }
	if(left_wheel_overcurrent_cnt > 250)
	{
		Usprintf("left wheel current error\n");
		left_wheel_overcurrent_cnt = 0;
		return 1;
	}
	return 0;
}
uint8_t Wheel_CheckRightCurrent(void)
{
	static uint16_t right_wheel_overcurrent_cnt = 0;
	int16_t temp_current = 0;	
	temp_current = Wheel_GetRightCurrent();
	if(temp_current < WHEEL_STALL_NOLOAD)
	{

	}
  else if(temp_current > WHEEL_STALL_LIMIT)
  {
	  right_wheel_overcurrent_cnt += 5;
  }
  else
  {
	  right_wheel_overcurrent_cnt = 0;
  }
	if(right_wheel_overcurrent_cnt > 250)
	{
		Usprintf("right wheel current error\n");
		right_wheel_overcurrent_cnt = 0;
		return 1;
	}
	return 0;
}


void Wheel_TuneSpeedProcess(int8_t l_wheel_speed, int8_t r_wheel_speed)
{
	static volatile int32_t right_proportion = 0, left_proportion = 0;	
	uint16_t proportion_biggest = 10*MUL_PWM;
		
	/*check if left wheel moving slowly*/
	if(l_wheel_speed > 15)
	{
		if(g_left_speed_cnt < 15)
		{
			if(g_left_speed_cnt < (l_wheel_speed * 3 / 5))g_left_wheel_slow++;
			else g_left_wheel_slow = 0;
			if(g_left_wheel_slow > 200)g_left_wheel_slow = 200;//5 seconds
		}
		else
		{
			g_left_wheel_slow = 0;
		}
	}
	else
	{
		g_left_wheel_slow = 0;
	}
	/*check if right wheel moving slowly*/	
	if(r_wheel_speed > 15)
	{
		if(g_right_speed_cnt < 15)
		{
			if(g_right_speed_cnt < (r_wheel_speed * 3 / 5))g_right_wheel_slow++;
			else g_right_wheel_slow = 0;
			if(g_right_wheel_slow > 200)g_right_wheel_slow = 200;
		}
		else
		{
			g_right_wheel_slow = 0;
		}
	}
	else
	{
		g_right_wheel_slow = 0;
	}


	if(l_wheel_speed > 1)
	{
		if(g_left_speed_cnt>70)g_left_speed_cnt=70;	
		left_proportion = l_wheel_speed - g_left_speed_cnt;
		if(left_proportion > proportion_biggest)left_proportion = proportion_biggest;
		g_temp_left_wheel_pwm += left_proportion;

		if(g_temp_left_wheel_pwm < 0)g_temp_left_wheel_pwm = 0;
		else if(g_temp_left_wheel_pwm > 100*MUL_PWM)g_temp_left_wheel_pwm = 100*MUL_PWM;
	}
	else
	{
		g_temp_left_wheel_pwm = 0;
	}

	if(r_wheel_speed > 1)
	{
		if(g_right_speed_cnt>70)g_right_speed_cnt=70;	
		right_proportion =  r_wheel_speed - g_right_speed_cnt	;
		if(right_proportion > proportion_biggest)right_proportion = proportion_biggest;
		g_temp_right_wheel_pwm += right_proportion;

		if(g_temp_right_wheel_pwm < 0)g_temp_right_wheel_pwm = 0;
		else if(g_temp_right_wheel_pwm > 100*MUL_PWM)g_temp_right_wheel_pwm = 100*MUL_PWM;
	}
	else
	{
		g_temp_right_wheel_pwm = 0;
	}
	
	g_left_speed_cnt = 0;
	g_right_speed_cnt = 0;

	if(L_F==1)
	{
	TIMER_CH0CV(TIMER0) = 0;
	TIMER_CH1CV(TIMER0) = (uint16_t)g_temp_left_wheel_pwm;
	}
	else if(L_F==2)
	{
	TIMER_CH1CV(TIMER0) = 0;
	TIMER_CH0CV(TIMER0) = (uint16_t)g_temp_left_wheel_pwm;
	}
	else
	{
	TIMER_CH0CV(TIMER0) = 0;
	TIMER_CH1CV(TIMER0) = 0;
	}

	if(R_F==1)
	{
	TIMER_CH2CV(TIMER0) = 0;
	TIMER_CH3CV(TIMER0) = (uint16_t)g_temp_right_wheel_pwm;
	}
	else if(R_F==2)
	{
	TIMER_CH3CV(TIMER0) = 0;
	TIMER_CH2CV(TIMER0) = (uint16_t)g_temp_right_wheel_pwm;
	}	
	else
	{
	TIMER_CH2CV(TIMER0) = 0;
	TIMER_CH3CV(TIMER0) = 0;
	}		
}

void Wheel_SpeedAccelerationProcess(void)
{
	static uint8_t left_increase_cnt = 0, right_increase_cnt = 0;
	uint8_t increase_cnt_limit = 1,speed_increase_limit = 1;
			
	if(Wheel_GetLeftSpeed() < Wheel_GetLeftTargetSpeed())
	{
		left_increase_cnt++;
		if(left_increase_cnt > increase_cnt_limit)
		{	
			left_increase_cnt = 0;
			Wheel_SetLeftSpeed(Wheel_GetLeftSpeed() + speed_increase_limit);			
		}
		if(Wheel_GetLeftSpeed() < 20)Wheel_SetLeftSpeed(Wheel_GetLeftSpeed() + speed_increase_limit);
	}
	if(Wheel_GetLeftSpeed() > Wheel_GetLeftTargetSpeed())
	{
		if(Wheel_GetLeftSpeed() > 1)Wheel_SetLeftSpeed(Wheel_GetLeftSpeed() - 1);
	}	
	
	if(Wheel_GetRightSpeed() < Wheel_GetRightTargetSpeed())
	{
		right_increase_cnt++;
		if(right_increase_cnt > increase_cnt_limit)
		{
			right_increase_cnt = 0;	
			Wheel_SetRightSpeed(Wheel_GetRightSpeed() + speed_increase_limit);					
		}
		if(Wheel_GetRightSpeed() < 20)Wheel_SetRightSpeed(Wheel_GetRightSpeed() + speed_increase_limit);
	}
	if((Wheel_GetRightSpeed() > Wheel_GetRightTargetSpeed()))
	{
		if(Wheel_GetRightSpeed() > 1)Wheel_SetRightSpeed(Wheel_GetRightSpeed() - 1);
	
	}
	
	if(Action_GetMove() == MOVE_ACT_HALF_TURN_LEFT)
	{
		if(Wheel_GetRightStep() >= Wheel_GetRightTargetStep() / 3)
		{
			if(Wheel_GetLeftTargetSpeed() > 1)
			{
				Wheel_SetLeftTargetSpeed(0);
				Wheel_SetRightTargetSpeed(MAX_SPEED);
			}
		}
	}
	if(Action_GetMove() == MOVE_ACT_HALF_TURN_RIGHT)
	{
		if(Wheel_GetLeftStep() >= Wheel_GetLeftTargetStep() / 3)
		{
			if(Wheel_GetRightTargetSpeed() > 1)
			{
				Wheel_SetRightTargetSpeed(0);
				Wheel_SetLeftTargetSpeed(MAX_SPEED);
			}
		}
	}
}

void Wheel_ResetSlowCnt(void)
{
	g_right_wheel_slow=0;
	g_left_wheel_slow=0;
}


/*Wheel Direction*/
volatile WheelDir_t g_left_wheel_dir = WHEEL_DIR_LEFT,g_right_wheel_dir = WHEEL_DIR_LEFT;
volatile WheelDir_t g_wheel_dir = WHEEL_DIR_LEFT;
void Wheel_SetDir(WheelDir_t dir)
{
	g_wheel_dir	=	dir;
	switch(g_wheel_dir)
	{
		/*Dir Forward*/
		case WHEEL_DIR_FORWARD:		if(Wheel_GetLeftDir()!=WHEEL_DIR_FORWARD)Wheel_SetLeftSpeed(0);
															if(Wheel_GetRightDir()!=WHEEL_DIR_FORWARD)Wheel_SetRightSpeed(0);
															Wheel_SetLeftDir(WHEEL_DIR_FORWARD);
															Wheel_SetRightDir(WHEEL_DIR_FORWARD);
															return;
		/*Dir Backward*/
		case WHEEL_DIR_BACKWARD:	if(Wheel_GetLeftDir()!=WHEEL_DIR_BACKWARD)Wheel_SetLeftSpeed(0);
															if(Wheel_GetRightDir()!=WHEEL_DIR_BACKWARD)Wheel_SetRightSpeed(0);
															Wheel_SetLeftDir(WHEEL_DIR_BACKWARD);
															Wheel_SetRightDir(WHEEL_DIR_BACKWARD);
															return;
		/*Dir Left*/
		case WHEEL_DIR_LEFT:			if(Wheel_GetLeftDir()!=WHEEL_DIR_BACKWARD)Wheel_SetLeftSpeed(0);
															if(Wheel_GetRightDir()!=WHEEL_DIR_FORWARD)Wheel_SetRightSpeed(0);
															Wheel_SetLeftDir(WHEEL_DIR_BACKWARD);
															Wheel_SetRightDir(WHEEL_DIR_FORWARD);
															return;
		/*Dir Right*/
		case WHEEL_DIR_RIGHT:			if(Wheel_GetLeftDir()!=WHEEL_DIR_FORWARD)Wheel_SetLeftSpeed(0);
															if(Wheel_GetRightDir()!=WHEEL_DIR_BACKWARD)Wheel_SetRightSpeed(0);
															Wheel_SetLeftDir(WHEEL_DIR_FORWARD);
															Wheel_SetRightDir(WHEEL_DIR_BACKWARD);
															return;
	}
}
WheelDir_t Wheel_GetDir(void)
{
	return g_wheel_dir;
}
void Wheel_SetLeftDir(WheelDir_t dir)
{
	if(dir == WHEEL_DIR_FORWARD)
	{
		LW_DIR_FORWARD();
	}
	else
	{
		LW_DIR_BACKWARD();
	}
	g_left_wheel_dir = dir;
}
void Wheel_SetRightDir(WheelDir_t dir)
{
	if(dir == WHEEL_DIR_FORWARD)
	{
		RW_DIR_FORWARD();
	}
	else
	{
		RW_DIR_BACKWARD();
	}
	g_right_wheel_dir = dir;
}
WheelDir_t Wheel_GetLeftDir(void)
{
	return g_left_wheel_dir;
}
WheelDir_t Wheel_GetRightDir(void)
{
	return g_right_wheel_dir;
}
void Set_Dir_Forward(void)
{
	RW_DIR_FORWARD();
	LW_DIR_FORWARD();
}


/*Wheel Speed*/
int8_t Wheel_GetLeftSpeed(void)
{
  return g_left_wheel_speed;
}
void Wheel_SetLeftSpeed(uint8_t Speed)
{
  if(Speed>100)Speed=100;
	g_left_wheel_speed = Speed;
}
int8_t Wheel_GetRightSpeed(void)
{
  return g_right_wheel_speed; 
}
void Wheel_SetRightSpeed(uint8_t Speed)
{
  if(Speed>100)Speed=100;
	g_right_wheel_speed = Speed;
}
void Wheel_SetSpeed(uint8_t Left,uint8_t Right)
{
	Wheel_SetLeftSpeed(Left);
	Wheel_SetRightSpeed(Right);
}

/*target speed*/
void Wheel_SetTargetSpeed(uint8_t l_speed,uint8_t r_speed)
{
	Wheel_SetLeftTargetSpeed(l_speed);
	Wheel_SetRightTargetSpeed(r_speed);
}
void Wheel_SetLeftTargetSpeed(uint8_t l_speed)
{
	g_left_target_speed = l_speed;
}
void Wheel_SetRightTargetSpeed(uint8_t r_speed)
{
	g_right_target_speed = r_speed;
} 
uint8_t Wheel_GetLeftTargetSpeed(void)
{
	return g_left_target_speed;
}
uint8_t Wheel_GetRightTargetSpeed(void)
{
	return g_right_target_speed;
}


/*Wheel Steps*/
void Wheel_ResetStep(void)
{
  g_left_wheel_step=0;
	g_right_wheel_step=0;
}
uint32_t Wheel_GetRightStep(void)
{
  return g_right_wheel_step;
}
uint32_t Wheel_GetLeftStep(void)
{
  return g_left_wheel_step;
}

/*target step*/
void Wheel_SetTargetStep(uint32_t l_step,uint32_t r_step)
{
	 g_left_wheel_taget_step = l_step;
	 g_right_wheel_taget_step = r_step;
}
void Wheel_SetLeftTargetStep(uint32_t step)
{
	g_left_wheel_taget_step = step;
}
void Wheel_SetRightTargetStep(uint32_t step)
{
	g_right_wheel_taget_step = step;
}
uint32_t Wheel_GetLeftTargetStep(void)
{
	return g_left_wheel_taget_step;
}
uint32_t Wheel_GetRightTargetStep(void)
{
	return g_right_wheel_taget_step;
}
uint8_t Wheel_LeftStepReached(int32_t step)
{
  if(g_left_wheel_step > step)return 1;
  return 0;
}
uint8_t Wheel_RightStepReached(int32_t step)
{
  if(g_right_wheel_step > step)return 1;
  return 0;
}

/*move step*/
void Wheel_ResetMoveStep(void)
{
  g_left_move_step = 0;
  g_right_move_step = 0;
}
int32_t Wheel_GetLeftMoveStep(void)
{
	return g_left_move_step;
}
int32_t Wheel_GetRightMoveStep(void)
{
	return g_right_move_step;
}

/*wall follow accelerate*/
volatile uint32_t g_wallfollow_acccelerate=0;
uint32_t WallFollow_GetWallAccelerate(void)  
{
  return g_wallfollow_acccelerate;
}
void WallFollow_ResetWallAccelerate(void)   
{
  g_wallfollow_acccelerate = 0;
}
/*wall follow steps*/
volatile int32_t  g_wallfollow_right_wheel_step=0,g_wallfollow_left_wheel_step=0;
int32_t WallFollow_GetLeftwheelStep(void)
{
	return g_wallfollow_left_wheel_step;
}
int32_t WallFollow_GetRightwheelStep(void)
{
	return g_wallfollow_right_wheel_step;
}
void WallFollow_ResetWheelStep(void)
{
  g_wallfollow_left_wheel_step = 0;
  g_wallfollow_right_wheel_step = 0;
}


/*wheel counts*/
void Wheel_SetCount(int32_t left, int32_t right)
{
	g_left_wheel_cnt = left;
	g_right_wheel_cnt = right;
}
int32_t Wheel_GetLeftCount(void)
{
	return g_left_wheel_cnt;
}
int32_t Wheel_GetRightCount(void)
{
	return g_right_wheel_cnt;
}




void Stop_Brifly(void)
{
	Wheel_Stop();	
}
void Wheel_Stop(void)
{
	Wheel_SetSpeed(0,0);
	Wheel_SetTargetSpeed(0,0);
  g_temp_left_wheel_pwm = 0;
	g_temp_right_wheel_pwm = 0;

	ActList_Clear();
	Action_SetMove(MOVE_ACT_STATIC);
	Wall_SetDynamicState(DISABLE);
	OBS_SetDynamicState(DISABLE);	
}

void Speed_Stop(void)
{
	Wheel_SetSpeed(0,0);
	Wheel_SetTargetSpeed(0,0);
  g_temp_left_wheel_pwm = 0;
	g_temp_right_wheel_pwm = 0;
}


void WHEEL_DISABLE(void)
{
	L_F=0;R_F=0;
	TIMER_CH0CV(TIMER0) = 0;
	TIMER_CH1CV(TIMER0) = 0;
	TIMER_CH2CV(TIMER0) = 0;
	TIMER_CH3CV(TIMER0) = 0;
//	TIMER_CHCTL2(TIMER0) &=~(TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E);
}

void LW_DIR_FORWARD(void)
{
	L_F=1;TIMER_CHCTL2(TIMER0) |=(TIM_CCER_CC1E|TIM_CCER_CC2E);
}

void LW_DIR_BACKWARD(void)
{
	L_F=2;TIMER_CHCTL2(TIMER0) |=(TIM_CCER_CC3E|TIM_CCER_CC4E);
}

void RW_DIR_FORWARD(void)
{
	R_F=1;TIMER_CHCTL2(TIMER0) |=(TIM_CCER_CC3E|TIM_CCER_CC4E);
}

void RW_DIR_BACKWARD(void)
{
	R_F=2;TIMER_CHCTL2(TIMER0) |=(TIM_CCER_CC1E|TIM_CCER_CC2E);
}

/*--------- Move Forward -----------------*/
void Move_Forward(uint8_t Left_Speed,uint8_t Right_Speed)
{
  Set_Dir_Forward();
	Wheel_SetSpeed(Left_Speed,Right_Speed);
}

/*action functions*/
#define ACTLIST_LENGTH	(uint8_t)20
volatile MoveAct_t g_action = MOVE_ACT_STATIC;
volatile Move_t g_act_list[ACTLIST_LENGTH + 1] = {MOVE_ACT_STATIC,0,0,0};
volatile uint8_t g_actlist_input_idx = 0, g_actlist_output_idx = 0, g_actlist_cnt = 0;
volatile Move_t g_actlist_buffer;
void Action_MoveToPoint(Point32_t point,uint32_t speed)
{
	Path_SetCurrentTargetPoint(point);
	Wheel_SetTargetSpeed(speed,speed);
	Wheel_SetRightTargetStep(MAX_DISTANCE);
	Wheel_SetLeftTargetStep(MAX_DISTANCE);
	Action_SetMove(MOVE_ACT_MOVE2POINT);
	Wheel_SetDir(WHEEL_DIR_FORWARD);
	OBS_SetDynamicState(ENABLE);
	Wall_SetDynamicState(ENABLE);
	Motor_SetCheckState(ENABLE);
	TurnSlip_SetCheckState(DISABLE);
	Wheel_ResetStep();
}
void Action_MoveForward(uint8_t l_speed, uint8_t r_speed, uint32_t step, MoveAct_t act)
{
	Wheel_SetTargetSpeed(l_speed, r_speed);
	Wheel_SetTargetStep(step,step);	
	Action_SetMove(act);
	Wheel_SetDir(WHEEL_DIR_FORWARD);
	Wheel_ResetStep();
	Motor_SetCheckState(ENABLE);
	TurnSlip_SetCheckState(DISABLE);
}
void Action_Turn(uint8_t l_speed, uint8_t r_speed, uint32_t angle, MoveAct_t dir)
{
	Wheel_SetTargetSpeed(l_speed,r_speed);
	if((dir == MOVE_ACT_HALF_TURN_RIGHT) || (dir == MOVE_ACT_HALF_TURN_LEFT))angle = angle*3/2;
	Wheel_SetRightTargetStep(angle);
	Wheel_SetLeftTargetStep(angle);
	Action_SetMove(dir);
	if((dir == MOVE_ACT_TURN_LEFT) || (dir == MOVE_ACT_HALF_TURN_LEFT) || (dir == MOVE_ACT_HEAD2BASELEFT))
	{
		Wheel_SetDir(WHEEL_DIR_LEFT);
		Direction_SetLastDir(DIRECTION_FLAG_LEFT);
	}
	else
	{
		Wheel_SetDir(WHEEL_DIR_RIGHT);
		Direction_SetLastDir(DIRECTION_FLAG_RIGHT);
	}
	Wheel_ResetStep();
	Motor_SetCheckState(ENABLE);
	TurnSlip_SetCheckState(ENABLE);
}
void Action_MoveBack(uint8_t l_speed, uint8_t r_speed, uint32_t step, MoveAct_t act)
{
	Wheel_SetTargetSpeed(l_speed,r_speed);
	Wheel_SetRightTargetStep(step);
	Wheel_SetLeftTargetStep(step);
	Wheel_SetSpeed(l_speed,r_speed);
	Wheel_SetDir(WHEEL_DIR_BACKWARD);
	Wheel_ResetStep();
	Action_SetMove(MOVE_ACT_BACK);
	Motor_SetCheckState(ENABLE);
	TurnSlip_SetCheckState(DISABLE);
}
void Action_Deceleration(void)
{
	Wheel_SetTargetSpeed(0,0);
	Action_SetMove(MOVE_ACT_DECELERATE);
	Wheel_ResetStep();
	TurnSlip_SetCheckState(DISABLE);
}

void Action_Head2Cource(void)
{
	CM_HeadToTarget(CM_GetCurrentTargetCnt());
	Action_SetMove(MOVE_ACT_HEAD2COURCE);
	Wheel_ResetStep();
	TurnSlip_SetCheckState(ENABLE);
	TurnSlip_ResetSlipCnt();
}

void ActList_Add_Forward(uint32_t dis,uint16_t spd_l,uint16_t spd_r)
{
	ActList_Add(MOVE_ACT_FORWARD, dis, spd_l, spd_r);
}



void Action_StopBrifly(void)
{
	Wheel_Stop();
  vTaskDelay(80 / portTICK_RATE_MS);
}
void Action_SetMove(MoveAct_t act)
{
	g_action = act;
}
MoveAct_t Action_GetMove(void)
{
	return g_action;
}
uint8_t Action_WaitForMoveEnded(void)
{
	if((Wheel_GetLeftStep() >= Wheel_GetLeftTargetStep()) || (Wheel_GetRightStep() >= Wheel_GetRightTargetStep()))
	{
		Action_SetMove(MOVE_ACT_HANDLER);
		return 1;
	}
	return 0;
}
void ActList_Add(MoveAct_t act_mode, uint32_t act_step, uint8_t l_speed, uint8_t r_speed)
{
	g_act_list[g_actlist_input_idx].state = act_mode;
	g_act_list[g_actlist_input_idx].step = act_step;
	g_act_list[g_actlist_input_idx].l_speed = l_speed;
	g_act_list[g_actlist_input_idx].r_speed = r_speed;
	
	g_actlist_input_idx++;
	if(g_actlist_input_idx > ACTLIST_LENGTH)g_actlist_input_idx = 0;
	g_actlist_cnt++;
}

void ActList_Add_Action(MoveAct_t action)
{
	g_act_list[g_actlist_input_idx].state = action;
	g_act_list[g_actlist_input_idx].step = 0;
	g_act_list[g_actlist_input_idx].l_speed = 0;
	g_act_list[g_actlist_input_idx].r_speed = 0;
	
	g_actlist_input_idx++;
	if(g_actlist_input_idx > ACTLIST_LENGTH)g_actlist_input_idx = 0;
	g_actlist_cnt++;
}

Move_t ActList_GetNext(void)
{
	Move_t Act_Buffer;
	if(g_actlist_cnt)
	{
		g_actlist_cnt--;
		Act_Buffer = g_act_list[g_actlist_output_idx];
		g_act_list[g_actlist_output_idx].state = MOVE_ACT_STATIC;
		g_actlist_output_idx++;
		if(g_actlist_output_idx > ACTLIST_LENGTH)g_actlist_output_idx = 0;
	}
	else
	{
		Act_Buffer.state = MOVE_ACT_STATIC;
	}
	return Act_Buffer;
}
void ActList_Clear(void)
{
	g_actlist_output_idx = 0;
	g_actlist_cnt = 0;
	g_actlist_input_idx = 0;
}

uint8_t ActList_GetCnt(void)
{
	return  g_actlist_cnt;
}
void ActList_Switch(void)
{
	g_actlist_buffer = ActList_GetNext();
	switch(g_actlist_buffer.state)
	{
		case MOVE_ACT_WALL:							Action_MoveForward(g_actlist_buffer.l_speed,g_actlist_buffer.r_speed,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;	
		case MOVE_ACT_FORWARD:					Action_MoveForward(g_actlist_buffer.l_speed,g_actlist_buffer.r_speed,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;
		case MOVE_ACT_STRAIGHT:					Action_MoveForward(g_actlist_buffer.l_speed,g_actlist_buffer.r_speed,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;		
		case MOVE_ACT_SEARCHWALL:				Action_MoveForward(g_actlist_buffer.l_speed,g_actlist_buffer.r_speed,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;
		case MOVE_ACT_BYPASS_LEFT:			Action_MoveForward(MAX_SPEED,MAX_SPEED/4,2200,g_actlist_buffer.state);Wheel_SetLeftSpeed(0);Wheel_SetRightSpeed(0);
																		break;
		case MOVE_ACT_BYPASS_RIGHT:			Action_MoveForward(MAX_SPEED/4,MAX_SPEED,2200,g_actlist_buffer.state);Wheel_SetLeftSpeed(0);Wheel_SetRightSpeed(0);
																		break;																
		case MOVE_ACT_SEARCHSTATION:		Action_MoveForward(g_actlist_buffer.l_speed,g_actlist_buffer.r_speed,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;
		case MOVE_ACT_HOMEPATH:					Action_MoveForward(Wheel_GetLeftTargetSpeed(),Wheel_GetRightTargetSpeed(),MAX_DISTANCE,g_actlist_buffer.state);
																		break;
		case MOVE_ACT_HOMEAROUND:				Action_MoveForward(Wheel_GetLeftTargetSpeed(),Wheel_GetRightTargetSpeed(),MAX_DISTANCE,g_actlist_buffer.state);
																		break;
		case MOVE_ACT_HALF_TURN_LEFT:		Action_MoveForward(0,MAX_SPEED,g_actlist_buffer.step,g_actlist_buffer.state);																		
																		break;	
		case MOVE_ACT_HALF_TURN_RIGHT:	Action_MoveForward(MAX_SPEED,0,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;																		
		case MOVE_ACT_HEAD2BASELEFT:		Action_Turn(g_actlist_buffer.l_speed,g_actlist_buffer.r_speed,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;
		case MOVE_ACT_HEAD2BASERIGHT:		Action_Turn(g_actlist_buffer.l_speed,g_actlist_buffer.r_speed,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;																
		case MOVE_ACT_TURN_LEFT:				Action_Turn(g_actlist_buffer.l_speed,g_actlist_buffer.r_speed,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;
		case MOVE_ACT_TURN_RIGHT:				Action_Turn(g_actlist_buffer.l_speed,g_actlist_buffer.r_speed,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;
		case MOVE_ACT_TURN_ROUND:				Action_Turn(g_actlist_buffer.l_speed,g_actlist_buffer.r_speed,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;
		case MOVE_ACT_BACK:							Action_MoveBack(g_actlist_buffer.l_speed,g_actlist_buffer.r_speed,g_actlist_buffer.step,g_actlist_buffer.state);
																		break;																																																							
		case MOVE_ACT_HEAD2COURCE:			Action_Head2Cource();
																		break;
		case MOVE_ACT_DECELERATE:				Action_Deceleration();
																		break;
		case MOVE_ACT_STATIC:						Wheel_Stop();
																		break;
		default:												Wheel_Stop();
																		break;
	}
}
void ActList_DecelerateToStop(void)
{
	ActList_Clear();
	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
	ActList_Add(MOVE_ACT_STATIC,0,0,0);
	Action_SetMove(MOVE_ACT_HANDLER);
}
void ActList_BackToStop(uint16_t back_distance)
{
	ActList_Clear();
	ActList_Add(MOVE_ACT_BACK,back_distance,BACK_SPEED,BACK_SPEED);
	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
	Action_SetMove(MOVE_ACT_HANDLER);
}
void ActList_WallOffEdgeStop(WallDir_t wall_dir)
{
	ActList_Clear();
	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
	ActList_Add(MOVE_ACT_STATIC,0,0,0);
	Action_SetMove(MOVE_ACT_HANDLER);
}

void ActList_WallStraightAndForward(MoveAct_t decelerate, uint32_t act_step)
{
	if(decelerate == MOVE_ACT_DECELERATE)ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
	ActList_Add(MOVE_ACT_STRAIGHT,act_step,WALL_WALK_SPEED,WALL_WALK_SPEED);
	ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
	Action_SetMove(MOVE_ACT_HANDLER);
}

void ActList_WallSlipStraightAndForward(MoveAct_t action)
{
	ActList_Clear();
	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
	if(action == MOVE_ACT_TURN_RIGHT)
	{
		ActList_Add(MOVE_ACT_STRAIGHT,300,WALL_WALK_SPEED,WALL_WALK_SPEED - 5);
	}	
	else if(action == MOVE_ACT_TURN_LEFT)
	{
		ActList_Add(MOVE_ACT_STRAIGHT,300,WALL_WALK_SPEED - 5,WALL_WALK_SPEED);
	}
	ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
	Action_SetMove(MOVE_ACT_HANDLER);		
}


/*turn slip functions*/
volatile FunctionalState	g_turn_slip_detect_State = DISABLE;
volatile uint8_t TurnSlip_Status = 0,g_turn_slip_cnt = 0;
void TurnSlip_DetectProcess(FunctionalState state)
{
	static int16_t slip_angle_diff = 0, slip_step_diff = 0, TurnSlip_Result = 0;
	static int32_t left_move_buffer = 0;
	static int16_t gyro_angle_buffer = 0;
	
	if(state == DISABLE)return;
	slip_step_diff = Math_Diff_int(Wheel_GetLeftMoveStep(),left_move_buffer);
	if(slip_step_diff > 20)//move 25 steps
	{
		slip_angle_diff = Math_Diff_int(Gyro_GetAngle(0),gyro_angle_buffer);
		if(slip_angle_diff > 0)
		{
			if(slip_angle_diff > 1800)slip_angle_diff = 3600 - slip_angle_diff;
			TurnSlip_Result =  (slip_step_diff*100) / slip_angle_diff;
			if(TurnSlip_Result > 250)
			{	
				g_turn_slip_cnt++;
				if(g_turn_slip_cnt > 3)
				{
					g_turn_slip_cnt = 0;
					TurnSlip_Status = 1;
				}
			}
			else
			{
				g_turn_slip_cnt = 0;
			}
		}
		else
		{
			g_turn_slip_cnt = 0;
			TurnSlip_Result = 0;
		}
	}	
	else
	{
		g_turn_slip_cnt = 0;
		TurnSlip_Result = 0;
	}
	gyro_angle_buffer = Gyro_GetAngle(0);
	left_move_buffer = Wheel_GetLeftMoveStep();
	
}
void TurnSlip_SetCheckState(FunctionalState state)
{
	g_turn_slip_detect_State = state;
	g_turn_slip_cnt = 0;
	TurnSlip_ResetSlipCnt();
}
FunctionalState TurnSlip_GetCheckState(void)
{
	return g_turn_slip_detect_State;
}
uint8_t TurnSlip_IsSlip(void)
{
	return TurnSlip_Status;
}
void TurnSlip_ResetSlipCnt(void)
{
	TurnSlip_Status = 0;
	g_turn_slip_cnt = 0;
}


void Wheel_TuneLeftSpeedDir(int32_t *speed,int32_t speed_limit)
{
	if(*speed < 0)
	{
		*speed = -*speed;
		if(*speed > speed_limit)*speed = speed_limit;
		
		if(Wheel_GetLeftSpeed() <= 1)
		{
			Wheel_SetLeftDir(WHEEL_DIR_BACKWARD);
		}
		else if(Wheel_GetLeftDir() == WHEEL_DIR_FORWARD)
		{
			*speed = 0;
		}
	}
	else
	{
		if(Wheel_GetLeftDir() != WHEEL_DIR_FORWARD)Wheel_SetLeftSpeed(0);
		Wheel_SetLeftDir(WHEEL_DIR_FORWARD);
	}
}

void Wheel_TuneRightSpeedDir(int32_t *speed,int32_t speed_limit)
{
	if(*speed < 0)
	{
		*speed = -*speed;
		if(*speed > speed_limit)*speed = speed_limit;
		if(Wheel_GetRightSpeed() <= 1)
		{
			Wheel_SetRightDir(WHEEL_DIR_BACKWARD);
		}
		else if(Wheel_GetRightDir() == WHEEL_DIR_FORWARD)
		{
			*speed = 0;
		}
	}
	else
	{
		if(Wheel_GetRightDir() != WHEEL_DIR_FORWARD)Wheel_SetRightSpeed(0);
		Wheel_SetRightDir(WHEEL_DIR_FORWARD);
	}
}

void Action_Stop(void)
{
	ActList_Clear();
	Wheel_Stop();
	Action_SetMove(MOVE_ACT_HANDLER);
	vTaskDelay(20/portTICK_RATE_MS);
	Wheel_ResetStep();
}

void Wheel_Forward_PidProcess(Decelerate_Type_t slow_flag,int16_t rotate_angle,int32_t *spdL,int32_t *spdR)
{
	static int16_t integration_value = 0;
	static int16_t Previous = 0;
//	static uint8_t add=0,add_cycle=0;
	int32_t speed_base ;
	int16_t Delta = 0,spd_l,spd_r;
	int16_t delta_spd = 0;
	
	speed_base = slow_flag ? BASE_SPEED : RUN_TOP_SPEED;
//	if(slow_flag == NULL_DECELERATE)
//	{
//		if(add<(RUN_TOP_SPEED- speed_base))
//		{
//			add_cycle++;
//			if(add_cycle>10)
//			{
//				add_cycle = 0;
//				add++;
//			}			
//		}
//		speed_base += add;
//		if(speed_base>RUN_TOP_SPEED)speed_base=RUN_TOP_SPEED;
//		if(speed_base<25)speed_base=25;
//	}
//	else
//	{
//		add = 0;
//	}	
	integration_value += rotate_angle;
	
	if (integration_value > 150) integration_value = 150;
	if (integration_value < -150) integration_value = -150;	
	
	Delta = rotate_angle - Previous;
	
	if(Delta > 50)Delta = 50;
	if(Delta < -50)Delta = -50;
	
	Previous = rotate_angle;
	
	delta_spd =  rotate_angle /9 + integration_value / 150 + Delta/3;
	
	if(delta_spd == 1)
	{
		spd_l = speed_base;      
		spd_r = speed_base + delta_spd;
	}
	else if(delta_spd == -1)
	{
		spd_l = speed_base - delta_spd;        
		spd_r = speed_base;
	}
	else
	{
		spd_l = speed_base - delta_spd;       
		spd_r = speed_base + delta_spd;
	}

	if(slow_flag)
	{
		if(spd_l > BASE_SPEED)spd_l= BASE_SPEED;
		if(spd_r > BASE_SPEED)spd_r = BASE_SPEED;
	}
	else
	{
		if(spd_l <= 15)spd_l = spd_l/2;
		if(spd_r <= 15)spd_r = spd_r/2;
	}
	
#ifdef PATH_COVER_TWOLANE	
	#if 0
	if(delta_spd > RUN_TOP_SPEED)
	{
		spd_r = 28;
	
		if((Wheel_GetDir() != WHEEL_DIR_LEFT)&&(Wheel_GetLeftSpeed() > 0))
		{
			spd_l = 0;
			
			Wheel_SetLeftSpeed(0);
		}
		else
		{
			spd_l = 10;
			Wheel_SetDir(WHEEL_DIR_LEFT);	
		}
	}
	else if(delta_spd < -(RUN_TOP_SPEED))
	{

		spd_l = 28;
		if((Wheel_GetDir() != WHEEL_DIR_RIGHT)&&(Wheel_GetRightSpeed() > 0))
		{
			spd_r = 0;
			Wheel_SetRightSpeed(0);
		}
		else
		{
			spd_r = 10;
			Wheel_SetDir(WHEEL_DIR_RIGHT);
		}
	}
	else
	{
		spd_l = Math_LimitingValue(spd_l,RUN_TOP_SPEED + 3);
		spd_r = Math_LimitingValue(spd_r,RUN_TOP_SPEED + 3);
		if(Wheel_GetDir() != WHEEL_DIR_FORWARD)
		{
			if(Wheel_GetDir() == WHEEL_DIR_LEFT)
			{
				if(Wheel_GetLeftSpeed() == 0)
				{
					Wheel_SetDir(WHEEL_DIR_FORWARD);
				}
				else
				{
					spd_l = 0;
					Wheel_SetLeftSpeed(0);
				}
			}	
			else if(Wheel_GetDir() == WHEEL_DIR_RIGHT)
			{
				if(Wheel_GetRightSpeed() == 0)
				{
					Wheel_SetDir(WHEEL_DIR_FORWARD);
				}
				else
				{
					spd_r = 0;
					Wheel_SetRightSpeed(0);
				}
			}
			else
			{
				Wheel_SetDir(WHEEL_DIR_FORWARD);
			}
		}
	}
	#endif
#else
#if 0
	if(Math_Diff_int(spd_l,spd_r)>10)
	{
		spd_l = Math_LimitingValue(spd_l,RUN_TOP_SPEED-3); 
		spd_r = Math_LimitingValue(spd_r,RUN_TOP_SPEED-3);	
	}		
	else
	{
		if(spd_l<RUN_TOP_SPEED+10)spd_l++;
		if(spd_r<RUN_TOP_SPEED+10)spd_r++;		
		spd_l = Math_LimitingValue(spd_l,RUN_TOP_SPEED+10);
		spd_r = Math_LimitingValue(spd_r,RUN_TOP_SPEED+10);
	}
	if(Wheel_GetDir()!=WHEEL_DIR_FORWARD)Wheel_SetDir(WHEEL_DIR_FORWARD);
#endif 
// if(Math_Diff_int(spd_l,spd_r)>15)
 if(Math_Abs_int(rotate_angle)>50)
 {
		if(spd_l>=0 && spd_r>=0)
		{
			if(spd_l>RUN_TOP_SPEED)spd_l=RUN_TOP_SPEED_U;
			if(spd_r>RUN_TOP_SPEED)spd_r=RUN_TOP_SPEED_U;		
			if(Wheel_GetDir()!=WHEEL_DIR_FORWARD)Wheel_SetDir(WHEEL_DIR_FORWARD);
		}
		else if(spd_l<0 && spd_r<0)
		{
			spd_l = -spd_l;
			spd_r = -spd_r;		
			if(spd_l>SPEED_DIV)spd_l=SPEED_DIV;
			if(spd_r>SPEED_DIV)spd_r=SPEED_DIV;		
			if(Wheel_GetDir()!=WHEEL_DIR_BACKWARD)Wheel_SetDir(WHEEL_DIR_BACKWARD);	
		}
		else if(spd_l>0 && spd_r<0)
		{
			spd_r = -spd_r;		
			if(spd_l>RUN_TOP_SPEED_U+5)spd_l=RUN_TOP_SPEED_U+5;
			if(spd_r>SPEED_DIV)spd_r=SPEED_DIV;		
			if(Wheel_GetDir()!=WHEEL_DIR_RIGHT)Wheel_SetDir(WHEEL_DIR_RIGHT);	
		}	
		else if(spd_l<0 && spd_r>0)
		{
			spd_l = -spd_l;		
			if(spd_l>SPEED_DIV)spd_l=SPEED_DIV;
			if(spd_r>RUN_TOP_SPEED_U+5)spd_r=RUN_TOP_SPEED_U+5;		
			if(Wheel_GetDir()!=WHEEL_DIR_LEFT)Wheel_SetDir(WHEEL_DIR_LEFT);	
		} 
	}
	else
	{
		
		if(slow_flag == NULL_DECELERATE)
		{
			if(spd_l<RUN_TOP_SPEED)spd_l+=2;
			if(spd_r<RUN_TOP_SPEED)spd_r+=2;			
		}	
		spd_l = Math_LimitingValue(spd_l,RUN_TOP_SPEED+10);
		spd_r = Math_LimitingValue(spd_r,RUN_TOP_SPEED+10);
		if(Wheel_GetDir()!=WHEEL_DIR_FORWARD)Wheel_SetDir(WHEEL_DIR_FORWARD);
	}

#endif
	
	*spdL = spd_l;
	*spdR = spd_r;
	
	if(slow_flag == FAST_DECELERATE)
	{
		if((Wheel_GetLeftSpeed() + Wheel_GetRightSpeed()) > BASE_SPEED*2)
		{
			Usprintf("%s(%d):slow(%d,%d)\n",__FUNCTION__,__LINE__,Wheel_GetLeftSpeed(),Wheel_GetRightSpeed());
			Wheel_SetSpeed(Wheel_GetLeftSpeed()/2, Wheel_GetRightSpeed()/2);
		}
	}
	
//	Usprintf("dir:%d,%d,head:%d,diff:%d,set(%d,%d),cur:(%d,%d),cur(%d,%d),i:%d,del:%d\n",Wheel_GetDir(),delta_spd,Gyro_GetAngle(0),rotate_angle,*spdL,*spdR,Wheel_GetSpdL(),Wheel_GetSpdR(),Wheel_GetTargetSpdL(),Wheel_GetTargetSpdR(),integration_value,Delta);
}	



