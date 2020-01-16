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
#include "Obscliff.h"





/*wall base line*/
volatile int16_t g_xpwall_baseline=200;
volatile int16_t g_leftwall_baseline=200;
volatile int16_t g_rightwall_baseline=200;
void Set_LWall_Base(int32_t data)
{
	g_leftwall_baseline = data;
}
int32_t Get_LWall_Base(void)
{
	return g_leftwall_baseline;
}

void Set_RWall_Base(int32_t data)
{
	g_rightwall_baseline = data;
}
int32_t Get_RWall_Base(void)
{
	return g_rightwall_baseline;
}
   
void Set_Xp_RWall_Base(int32_t data)
{ 
	g_xpwall_baseline = data;
}
int32_t Get_Xp_RWall_Base(void)
{
	return g_xpwall_baseline;
}

/*obs wall adc*/
int32_t Get_LWall_ADC(void)
{
	return g_obs_adc.Left_Wall;
}

int32_t Get_RWall_ADC(void)
{
	return g_obs_adc.Right_Wall;
}

int32_t Get_Xp_RWall_ADC(void)
{
	return g_obs_adc.Xp_Wall;
}

int32_t Get_LeftOBS(void)
{
	return g_obs_adc.Left_OBS;
}

int32_t Get_RightOBS(void)
{
	return g_obs_adc.Right_OBS;
}

int32_t Get_FrontOBS(void)
{
	return g_obs_adc.Front_OBS;
}


/*obs trig val*/
volatile int16_t g_left_obs_trig_val=500;
volatile int16_t g_front_obs_trig_val=500;
volatile int16_t g_right_obs_trig_val=500;
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
uint8_t Get_OBS_Status(void)
{
	uint8_t Status=0;

	if(g_obs_adc.Left_OBS  >(g_left_obs_trig_val))Status|=Status_Left_OBS;

	if(g_obs_adc.Front_OBS >(g_front_obs_trig_val))Status|=Status_Front_OBS;
		
	if(g_obs_adc.Right_OBS >(g_right_obs_trig_val))Status|=Status_Right_OBS;

	return Status; 
}


/*cliff adc*/
uint8_t Get_Cliff_Trig(void)
{
  uint8_t Temp_Status=0;
	uint8_t l_CT_Counter=0;
	uint8_t r_CT_Counter=0;
	uint8_t bl_CT_Counter=0;
	uint8_t br_CT_Counter=0;	
	uint8_t CT_Counter=0;

	while(1)
	{
		if(((g_cliff_adc.Back_Left_Cliff>Cliff_Limit)&&(g_cliff_adc.Back_Right_Cliff>Cliff_Limit)&&(g_cliff_adc.Front_Left_Cliff>Cliff_Limit)&&(g_cliff_adc.Front_Right_Cliff>Cliff_Limit)))
		{	
			Temp_Status = 0;			
			break;
		}
		else
		{
			/*---------------------------------------Left cliff-----------------------------*/  
			if(g_cliff_adc.Front_Left_Cliff<Cliff_Limit)l_CT_Counter++;
			else l_CT_Counter = 0;
			if(l_CT_Counter>9)
			{
				Temp_Status|=Status_Cliff_Left;				
			}
			/*---------------------------------------Right cliff-----------------------------*/
			if(g_cliff_adc.Front_Right_Cliff<Cliff_Limit)r_CT_Counter++;
			else r_CT_Counter = 0;
			if(r_CT_Counter>9)
			{
				Temp_Status|=Status_Cliff_Right;				
			}
			/*---------------------------------------back left cliff-----------------------------*/
			if(g_cliff_adc.Back_Left_Cliff<Cliff_Limit)bl_CT_Counter++;
			else bl_CT_Counter = 0;
			if(bl_CT_Counter>4)
			{
				Temp_Status|=Status_Cliff_Back_Left;				
			}			
			/*---------------------------------------back right cliff-----------------------------*/
			if(g_cliff_adc.Back_Right_Cliff<Cliff_Limit)br_CT_Counter++;
			else br_CT_Counter = 0;
			if(br_CT_Counter>4)
			{
				Temp_Status|=Status_Cliff_Back_Right;				
			}					
			
			/*---------------------------------------Front cliff-----------------------------*/
			/*if(g_cliff_adc.Front_Cliff<Cliff_Limit)f_CT_Counter++;
			else f_CT_Counter = 0;
			if(f_CT_Counter>4)
			{
				Temp_Status|=Status_Cliff_Front;				
			}*/
			if((Temp_Status&Status_Cliff_Front)==Status_Cliff_Front)
			{
				Temp_Status|=Status_Cliff_Front;
			}
			
			CT_Counter++;
			
			if((l_CT_Counter==0)&&(r_CT_Counter==0))
			{
				if((CT_Counter>9)||(0!=Temp_Status))
				{
					CT_Counter = 0;
					break;
				}
			}
			else
			{
				if(CT_Counter>12)
				{
					CT_Counter = 0;
					break;
				}
			}			
			delay(15);
		}
	}
	return Temp_Status;
}










