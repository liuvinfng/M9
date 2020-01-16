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




/*wall base line*/
volatile int16_t g_xpwall_baseline=200;
volatile int16_t g_leftwall_baseline=200;
volatile int16_t g_rightwall_baseline=200;
volatile FunctionalState g_wall_dynamic_state = DISABLE;
void Wall_DynamicProcess(FunctionalState state)
{
	static int32_t wall_r_sum_value = 0, wall_r_average_value = 0, wall_r_stable_cnt = 0, wall_r_temp_buffer = 0;
	static int32_t wall_l_sum_value = 0, wall_l_average_value = 0, wall_l_stable_cnt = 0, wall_l_temp_buffer = 0;	
	static int32_t wall_x_sum_value = 0, wall_x_average_value = 0, wall_x_stable_cnt = 0, wall_x_temp_buffer = 0;	
	
	if(state == DISABLE)return;
	//right
	wall_r_temp_buffer = Wall_GetRightAdcValue();
	wall_r_sum_value += wall_r_temp_buffer;
	wall_r_stable_cnt++;
	wall_r_average_value = wall_r_sum_value / wall_r_stable_cnt;
	
	if(Math_Diff_int(wall_r_average_value, wall_r_temp_buffer) > 10)
	{
		wall_r_average_value = 0;
		wall_r_stable_cnt = 0;
		wall_r_sum_value = 0;
	}
	if(wall_r_stable_cnt > 20)//4seconds
	{
		wall_r_average_value += Wall_GetRightBaseline();
		Wall_SetRightBaseline(wall_r_average_value);
		wall_r_average_value = 0;
		wall_r_stable_cnt = 0;
		wall_r_sum_value = 0;
	}
	//left
	wall_l_temp_buffer = Wall_GetLeftAdcValue();
	wall_l_sum_value += wall_l_temp_buffer;
	wall_l_stable_cnt++;
	wall_l_average_value = wall_l_sum_value / wall_l_stable_cnt;
	
	if(Math_Diff_int(wall_l_average_value, wall_l_temp_buffer) > 10)
	{
		wall_l_average_value = 0;
		wall_l_stable_cnt = 0;
		wall_l_sum_value = 0;
	}
	if(wall_l_stable_cnt > 20)//4seconds
	{
		wall_l_average_value += Wall_GetLeftBaseline();
		Wall_SetLeftBaseline(wall_l_average_value);
		wall_l_average_value = 0;
		wall_l_stable_cnt = 0;
		wall_l_sum_value = 0;
	}
	//x
	wall_x_temp_buffer = Wall_GetXAdcValue();
	wall_x_sum_value += wall_x_temp_buffer;
	wall_x_stable_cnt++;
	wall_x_average_value = wall_x_sum_value / wall_x_stable_cnt;
	
	if(Math_Diff_int(wall_x_average_value, wall_x_temp_buffer) > 10)
	{
		wall_x_average_value = 0;
		wall_x_stable_cnt = 0;
		wall_x_sum_value = 0;
	}
	if(wall_x_stable_cnt > 20)//4seconds
	{
		wall_x_average_value += Wall_GetXBaseline();
		Wall_SetXBaseline(wall_x_average_value);
		wall_x_average_value = 0;
		wall_x_stable_cnt = 0;
		wall_x_sum_value = 0;
	}	
}
void Wall_SetDynamicState(FunctionalState state)
{
	g_wall_dynamic_state = state;
//	printf("%s(%d): %d \n", __FUNCTION__, __LINE__,g_wall_dynamic_state);
}
FunctionalState Wall_GetDynamicState(void)
{
	return g_wall_dynamic_state;
}
void Wall_SetLeftBaseline(int32_t data)
{
	g_leftwall_baseline = data;
}
void Wall_SetRightBaseline(int32_t data)
{
	g_rightwall_baseline = data;
}
void Wall_SetXBaseline(int32_t data)
{ 
	g_xpwall_baseline = data;
}

int32_t Wall_GetLeftBaseline(void)
{
	return g_leftwall_baseline;
}
int32_t Wall_GetRightBaseline(void)
{
	return g_rightwall_baseline;
}
int32_t Wall_GetXBaseline(void)
{
	return g_xpwall_baseline;
}
void Wall_AdjustBaseline(void)
{
	Wall_SetRightBaseline(Wall_GetRightAdcValue());
	Wall_SetLeftBaseline(Wall_GetLeftAdcValue());
	Wall_SetXBaseline(Wall_GetXAdcValue());	
}

/*average*/ 
volatile int32_t g_wall_right_average = 0,g_wall_left_average = 0,g_wall_x_average = 0;
void Wall_SetAverageAdcValue(int32_t left_average,int32_t right_average,int32_t x_average)
{
	g_wall_right_average = right_average;
	g_wall_left_average  = left_average;
	g_wall_x_average     = x_average;	
}
int32_t Wall_GetRightAverageAdcValue(void)
{
//	if(g_wall_right_average>g_wall_x_average)
	{
		if(g_wall_right_average>1000)g_wall_right_average=1000;
		return g_wall_right_average;
	}
//	else
//	return g_wall_x_average;	
}
int32_t Wall_GetLeftAverageAdcValue(void)
{
	return g_wall_left_average;
}
int32_t Wall_GetXAverageAdcValue(void)
{
	return g_wall_x_average;
}


/*wall value*/
int32_t Wall_GetLeftAdcValue(void)
{
	return g_obs_adc.Left_Wall;
}
int32_t Wall_GetRightAdcValue(void)
{
	return g_obs_adc.Right_Wall;
}
int32_t Wall_GetXAdcValue(void)
{
	return g_obs_adc.Xp_Wall;
}



/*obs trig value*/
#define OBS_LEFT_TRIG_BIAS	300	
#define OBS_FRONT_TRIG_BIAS	300	
#define OBS_RIGHT_TRIG_BIAS	300	
volatile int16_t g_left_obs_trig_val=500;
volatile int16_t g_front_obs_trig_val=500;
volatile int16_t g_right_obs_trig_val=500;
volatile FunctionalState g_obs_dynamic_state = DISABLE;
void OBS_DynamicProcess(FunctionalState state)
{
	static int32_t front_obs_buffer = 0, left_obs_buffer = 0, right_obs_buffer = 0;
	static uint32_t front_obs_stable_cnt = 0, left_obs_stable_cnt=0, right_obs_stable_cnt = 0;
	static int32_t front_obs_average_value = 0, left_obs_average_value=0, right_obs_average_value = 0;
	static int32_t front_obs_sum = 0, left_obs_sum=0, right_obs_sum = 0;
	
	if(state == DISABLE)return;

	/*---------------Front-----------------------*/
	front_obs_buffer = g_obs_adc.Front_OBS;
	front_obs_sum += front_obs_buffer;
	front_obs_stable_cnt++; 
	front_obs_average_value = front_obs_sum / front_obs_stable_cnt;
	if(Math_Diff_int(front_obs_average_value, front_obs_buffer) > 50)
	{
		front_obs_stable_cnt = 0;
		front_obs_sum = 0;
	}
	if(front_obs_stable_cnt > 15)//3 seconds
	{
		front_obs_stable_cnt = 0;
		front_obs_buffer = g_front_obs_trig_val - OBS_FRONT_TRIG_BIAS;
		if(g_obs_sunlight.Front_OBS>front_obs_buffer||(front_obs_buffer<front_obs_average_value))
		{
			front_obs_buffer = (front_obs_average_value+front_obs_buffer)/2;
	  }
		else
		{
			front_obs_buffer = (front_obs_average_value+front_obs_buffer*29)/30;
		}		
		if(front_obs_buffer < 100)front_obs_buffer = 100;
		g_front_obs_trig_val = front_obs_buffer + OBS_FRONT_TRIG_BIAS;
	}
	/*---------------Left-----------------------*/
	left_obs_buffer = g_obs_adc.Left_OBS;
	left_obs_sum += left_obs_buffer;
	left_obs_stable_cnt++;
	left_obs_average_value=left_obs_sum / left_obs_stable_cnt;
	if(Math_Diff_int(left_obs_average_value, left_obs_buffer) > 50)
	{
		left_obs_stable_cnt = 0;
		left_obs_sum = 0;
	}
	if(left_obs_stable_cnt > 15)
	{
		left_obs_stable_cnt = 0;
		left_obs_buffer = g_left_obs_trig_val - OBS_LEFT_TRIG_BIAS;
		if((g_obs_sunlight.Left_OBS>left_obs_buffer)||(left_obs_buffer<left_obs_average_value))
		{
			left_obs_buffer = (left_obs_average_value+left_obs_buffer)/2;
	  }
		else
		{
			left_obs_buffer = (left_obs_average_value+left_obs_buffer*29)/30;
		}		
		if(left_obs_buffer < 100)left_obs_buffer = 100;
		g_left_obs_trig_val = left_obs_buffer + OBS_LEFT_TRIG_BIAS;
	}
	/*---------------Right-----------------------*/
	right_obs_buffer = g_obs_adc.Right_OBS;
	right_obs_sum += right_obs_buffer;
	right_obs_stable_cnt++;
	right_obs_average_value = right_obs_sum / right_obs_stable_cnt;
	if(Math_Diff_int(right_obs_average_value, right_obs_buffer) > 50)
	{
		right_obs_stable_cnt = 0;
		right_obs_sum = 0;
	}
	if(right_obs_stable_cnt > 15)
	{
		right_obs_stable_cnt = 0;
		right_obs_buffer = g_right_obs_trig_val - OBS_RIGHT_TRIG_BIAS;
		if(g_obs_sunlight.Right_OBS>right_obs_buffer)
		{
			right_obs_buffer = (right_obs_average_value+right_obs_buffer)/2;
		}
		else
		{
			right_obs_buffer = (right_obs_average_value+right_obs_buffer*29)/30;
		}		
		if(right_obs_buffer < 100)right_obs_buffer = 100;
		g_right_obs_trig_val = right_obs_buffer + OBS_RIGHT_TRIG_BIAS;
	}
}
void OBS_SetDynamicState(FunctionalState state)
{
	g_obs_dynamic_state = state;
}
FunctionalState OBS_GetDynamicState(void)
{
	return g_obs_dynamic_state;
}
void OBS_AdjustTrigValue(void)
{
	if(g_obs_adc.Left_OBS > g_left_obs_trig_val)g_left_obs_trig_val+=800;
  if(g_obs_adc.Front_OBS > g_front_obs_trig_val)g_front_obs_trig_val+=800;
  if(g_obs_adc.Right_OBS > g_right_obs_trig_val)g_right_obs_trig_val+=800;
}
void OBS_ResetTrigValue(void)
{
	g_left_obs_trig_val  = g_obs_adc.Left_OBS + OBS_LEFT_TRIG_BIAS;
	if(g_left_obs_trig_val<500)g_left_obs_trig_val=500;
	g_front_obs_trig_val = g_obs_adc.Front_OBS + OBS_FRONT_TRIG_BIAS;
	if(g_front_obs_trig_val<500)g_front_obs_trig_val=500;
	g_right_obs_trig_val = g_obs_adc.Right_OBS + OBS_RIGHT_TRIG_BIAS;
	if(g_right_obs_trig_val<500)g_right_obs_trig_val=500;
}
int16_t Get_FrontOBST_Value(void)
{
	return g_front_obs_trig_val;
}
int16_t Get_LeftOBST_Value(void)
{
	return g_left_obs_trig_val;
}
int16_t Get_RightOBST_Value(void)
{
	return g_right_obs_trig_val;
}
uint8_t OBS_GetTrigStatus(void)
{
	uint8_t Status=0;

	if(g_obs_adc.Left_OBS  >(g_left_obs_trig_val))Status|=OBS_LEFT_TRIG;

	if(g_obs_adc.Front_OBS >(g_front_obs_trig_val))Status|=OBS_FRONT_TRIG;
		
	if(g_obs_adc.Right_OBS >(g_right_obs_trig_val))Status|=OBS_RIGHT_TRIG;

	return Status; 
}

//obs near 
uint8_t g_obs_near=0;
void Set_near_Flag(uint8_t flag)
{
	g_obs_near=flag;
}
uint8_t OBS_IsWallNear(void)
{
	uint8_t ret_val = 0;
	if (g_obs_adc.Right_OBS > (g_right_obs_trig_val))ret_val  |= OBS_RIGHT_TRIG;
	if (g_obs_adc.Front_OBS > (g_front_obs_trig_val))ret_val  |= OBS_FRONT_TRIG;	
	if (g_obs_adc.Left_OBS  > (g_left_obs_trig_val))ret_val   |= OBS_LEFT_TRIG;
	return ret_val;
}
uint8_t OBS_IsNear(void)
{
//	if(g_obs_near)return 1;
  if(g_obs_adc.Left_OBS  > (g_left_obs_trig_val  - 330))return 1;
  if(g_obs_adc.Front_OBS > (g_front_obs_trig_val - 330))return 1;	
  if(g_obs_adc.Right_OBS > (g_right_obs_trig_val - 330))return 1;	
	return 0;
}
uint8_t OBS_L_F_Near(void)
{
  if(g_obs_adc.Left_OBS  > (g_left_obs_trig_val))return 1;
  if(g_obs_adc.Front_OBS > (g_front_obs_trig_val))return 1;	
	if(Wall_GetLeftAverageAdcValue()>500)return 1;
	return 0;
}
uint8_t OBS_R_F_Near(void)
{
  if(g_obs_adc.Right_OBS  > (g_right_obs_trig_val))return 1;
  if(g_obs_adc.Front_OBS > (g_front_obs_trig_val))return 1;	
	if(Wall_GetRightAverageAdcValue()>500)return 1;
	return 0;
}
uint8_t OBS_SpotStatus(void)
{
  uint8_t Status = 0;
  if(g_obs_adc.Left_OBS  > (g_left_obs_trig_val + 200))Status |= OBS_LEFT_TRIG;
  if(g_obs_adc.Front_OBS > (g_front_obs_trig_val + 200))Status |= OBS_FRONT_TRIG;
  if(g_obs_adc.Right_OBS > (g_right_obs_trig_val + 200))Status |= OBS_RIGHT_TRIG;
	return Status;
}
void OBS_OverLimitCheck(uint8_t obs_val)
{
	static uint8_t obs_over_limit_cnt = 0;
	if(obs_val)
	{
		obs_over_limit_cnt++;
		Usprintf("%s(%d):obs_over_limit_cnt = %d \n",__FUNCTION__, __LINE__,obs_over_limit_cnt);
		if(obs_over_limit_cnt > 24)//4
		{
			OBS_AdjustTrigValue();
			obs_over_limit_cnt = 0;
		}
	}
	else
	{
		obs_over_limit_cnt = 0;
	}
}



/*obs value*/
int32_t OBS_GetFrontValue(void)
{
	return g_obs_adc.Front_OBS;
}
int32_t OBS_GetLeftValue(void)
{
	return g_obs_adc.Left_OBS;
}
int32_t OBS_GetRightValue(void)
{
	return g_obs_adc.Right_OBS;
}



/*cliff*/
volatile CliffTrig_t g_cliff_status_val = CLIFF_ALL_TRIG;
volatile FunctionalState g_cliff_detect_state = DISABLE;
void Cliff_DetectionProcess(FunctionalState state)//5ms
{
	CliffTrig_t temp_status = CLIFF_NO_TRIG;
	static uint8_t left_cliff_cnt = 0,right_cliff_cnt = 0,left_back_cliff_cnt = 0,right_back_cliff_cnt = 0;
	
	if(state == DISABLE)return;
	/*---------------------------------------Left cliff-----------------------------*/
  if(g_cliff_adc.Front_Left_Cliff < CLIFF_LIMIT)
	{
		left_cliff_cnt++;
		if(left_cliff_cnt > 0)
		{
			left_cliff_cnt = 2;
			temp_status |= CLIFF_LEFT_TRIG;
		}
	}
	else
	{
		left_cliff_cnt = 0;
		temp_status &=~ CLIFF_LEFT_TRIG;
	}
	/*---------------------------------------Right cliff-----------------------------*/
	if(g_cliff_adc.Front_Right_Cliff < CLIFF_LIMIT)
	{
		right_cliff_cnt++;
		if(right_cliff_cnt > 0)
		{
			right_cliff_cnt	= 2;
			temp_status |= CLIFF_RIGHT_TRIG;
		}
	}
	else
	{
		right_cliff_cnt = 0;
		temp_status &=~ CLIFF_RIGHT_TRIG;
	}
	/*---------------------------------------Front cliff-----------------------------*/
	if((temp_status & CLIFF_FRONT_TRIG)==CLIFF_FRONT_TRIG)
	{
		temp_status |= CLIFF_FRONT_TRIG;
	}
	else
	{
//		temp_status &= ~CLIFF_FRONT_TRIG;
	}
	/*---------------------------------------back Left cliff-----------------------------*/
  if(g_cliff_adc.Back_Left_Cliff < CLIFF_LIMIT)
	{
		left_back_cliff_cnt++;
		if(left_back_cliff_cnt > 0)
		{
			left_back_cliff_cnt = 2;
			temp_status |= CLIFF_BACK_LEFT_TRIG;
		}
	}
	else
	{
		left_back_cliff_cnt = 0;
		temp_status &=~ CLIFF_BACK_LEFT_TRIG;
	}
	/*---------------------------------------back Right cliff-----------------------------*/
	if(g_cliff_adc.Back_Right_Cliff < CLIFF_LIMIT)
	{
		right_back_cliff_cnt++;
		if(right_back_cliff_cnt > 0)
		{
			right_back_cliff_cnt	= 2;
			temp_status |= CLIFF_BACK_RIGHT_TRIG;
		}
	}
	else
	{
		right_back_cliff_cnt = 0;
		temp_status &=~ CLIFF_BACK_RIGHT_TRIG;
	}
	g_cliff_status_val = temp_status;
}
void Cliff_SetDetectState(FunctionalState state)
{
	 g_cliff_detect_state = state;
}
FunctionalState Cliff_GetDetectState(void)
{
	return g_cliff_detect_state;
}
CliffTrig_t Cliff_GetDetectiontProcess_Result(void)
{
  return g_cliff_status_val;
//	return 0;
}



CliffTrig_t Cliff_GetInstantStatus(void)
{
	CliffTrig_t temp_status = CLIFF_NO_TRIG;
		/*---------------------------------------Left cliff-----------------------------*/
  if(g_cliff_adc.Front_Left_Cliff < CLIFF_LIMIT)
	{
		temp_status |= CLIFF_LEFT_TRIG;
	}
	else
	{
		temp_status &=~ CLIFF_LEFT_TRIG;
	}
	/*---------------------------------------Right cliff-----------------------------*/
	if(g_cliff_adc.Front_Right_Cliff < CLIFF_LIMIT)
	{
		temp_status |= CLIFF_RIGHT_TRIG ;
	}
	else
	{
		temp_status &=~ CLIFF_RIGHT_TRIG;
	}
	/*---------------------------------------Front cliff-----------------------------*/
	/*if(g_cliff_adc.front < CLIFF_LIMIT)*/
	if((temp_status & CLIFF_FRONT_TRIG)==CLIFF_FRONT_TRIG	)
	{
		temp_status |= CLIFF_FRONT_TRIG;
	}
	else
	{
		temp_status &=~ CLIFF_FRONT_TRIG;
	}

	if(g_cliff_adc.Back_Left_Cliff < CLIFF_LIMIT)
	{
		temp_status |= CLIFF_BACK_LEFT_TRIG ;
	}
	else
	{
		temp_status &=~ CLIFF_BACK_LEFT_TRIG;
	}
	if(g_cliff_adc.Back_Right_Cliff < CLIFF_LIMIT)
	{
		temp_status |= CLIFF_BACK_RIGHT_TRIG ;
	}
	else
	{
		temp_status &=~ CLIFF_BACK_RIGHT_TRIG;
	}	
	return temp_status;
}





