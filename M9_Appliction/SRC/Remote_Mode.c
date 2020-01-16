
 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   this mode the robot follows the command of the remote , 
	           Upkey : move forward untill stop command or obstacle event
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  
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



extern volatile uint32_t g_left_wheel_step,g_right_wheel_step;
void Remote_Mode(void)
{
	uint8_t ret_val = 0;
  uint16_t No_Command_Counter=0;
	uint8_t back_to_userinterface_flag = 0;
	uint16_t move_back_distance = 600;	
	portTickType xLastWakeTime = xTaskGetTickCount();

	Usprintf("%s(%d):remote mode Start!\n",__FUNCTION__, __LINE__);	
	Motor_WorkConfigure();
	vTaskDelay(500/portTICK_RATE_MS);
  while(Mode_GetMode() == MODE_REMOTE)
	{
		switch(Action_GetMove())
		{
			case MOVE_ACT_FORWARD:   
																if((Rcon_GetRemoteCode() == Remote_Forward) || (Wifi_GetRemoteCode() == Remote_Forward))
																{
																	Wheel_SetTargetStep(Wheel_GetLeftStep() + 150,Wheel_GetRightStep() + 150);
																	Rcon_ResetRemoteCode();
																	No_Command_Counter = 0;
																}
																else if(Rcon_GetRemoteCode())Action_SetMove(MOVE_ACT_HANDLER);
															  Action_WaitForMoveEnded();																
																break;
			case MOVE_ACT_BACK:    		if((Rcon_GetRemoteCode() == Remote_Max) || (Wifi_GetRemoteCode() == Remote_Max))
																{
																	Wheel_SetTargetStep(Wheel_GetLeftStep() + 150,Wheel_GetRightStep() + 150);
																	Rcon_ResetRemoteCode();
																	No_Command_Counter = 0;
																}
																else if(Rcon_GetRemoteCode())Action_SetMove(MOVE_ACT_HANDLER);
															  Action_WaitForMoveEnded();																
																break;													
			case MOVE_ACT_TURN_LEFT:  if((Rcon_GetRemoteCode() == Remote_Left) || (Wifi_GetRemoteCode() == Remote_Left))
																{
																	Wheel_SetTargetStep(Wheel_GetLeftStep() + 150,Wheel_GetRightStep() + 150);
																	Rcon_ResetRemoteCode();
																	No_Command_Counter = 0;
																}
																else if(Rcon_GetRemoteCode())Action_SetMove(MOVE_ACT_HANDLER);
																Action_WaitForMoveEnded();
																break;
			case MOVE_ACT_TURN_RIGHT: if((Rcon_GetRemoteCode() == Remote_Right) || (Wifi_GetRemoteCode() == Remote_Right))
																{
																	Wheel_SetTargetStep(Wheel_GetLeftStep() + 150,Wheel_GetRightStep() + 150);
																	Rcon_ResetRemoteCode();
																	No_Command_Counter = 0;
																}
																else if(Rcon_GetRemoteCode())Action_SetMove(MOVE_ACT_HANDLER);
																Action_WaitForMoveEnded();
																break;
			case MOVE_ACT_TURN_ROUND:
																if(Rcon_GetRemoteCode())
																{
																	if(Rcon_GetRemoteCode() != Remote_Max)
																	{
																		Action_SetMove(MOVE_ACT_HANDLER);
																	}	
																	else
																	{
																		Rcon_ResetRemoteCode();
																		No_Command_Counter = 0;
																	}																		
																}
																Action_WaitForMoveEnded();
																break;
			case MOVE_ACT_DECELERATE:	if((Wheel_GetLeftSpeed() <= 5)&&(Wheel_GetRightSpeed() <= 5))
																{																	
																	Action_SetMove(MOVE_ACT_HANDLER);																		
																}
																break;																
			case MOVE_ACT_HANDLER:		ActList_Switch();
																break;
			case MOVE_ACT_STATIC:     
																Wheel_Stop();
																if(back_to_userinterface_flag)
																{
																	back_to_userinterface_flag = 0;
																	Mode_SetMode(MODE_USERINTERFACE);															
																}
																break;	  																
			default:break;
		}
		if((Action_GetMove() ==  MOVE_ACT_STATIC) || (Action_GetMove() ==  MOVE_ACT_DECELERATE))
		{
			if(back_to_userinterface_flag != 1)
			{
				if((Rcon_GetRemoteCode() == Remote_Forward) || (Wifi_GetRemoteCode() == Remote_Forward))
				{
					Speaker(SPK_DON);
					ActList_Clear();
					ActList_Add(MOVE_ACT_FORWARD,200,RUN_TOP_SPEED,RUN_TOP_SPEED);
					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
					ActList_Add(MOVE_ACT_STATIC,0,0,0);																				
					Action_SetMove(MOVE_ACT_HANDLER);		
					No_Command_Counter=0;
					Rcon_ResetRemoteCode();
				}
				if((Rcon_GetRemoteCode() == Remote_Left) || (Wifi_GetRemoteCode() == Remote_Left))
				{
					Speaker(SPK_DON);
					ActList_Clear();
					ActList_Add(MOVE_ACT_TURN_LEFT,200,TURN_SPEED,TURN_SPEED);
					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
					ActList_Add(MOVE_ACT_STATIC,0,0,0);																				
					Action_SetMove(MOVE_ACT_HANDLER);		
					No_Command_Counter=0;
				}
				if((Rcon_GetRemoteCode() == Remote_Right) || (Wifi_GetRemoteCode() == Remote_Right))
				{
					Speaker(SPK_DON);
					ActList_Clear();
					ActList_Add(MOVE_ACT_TURN_RIGHT,200,TURN_SPEED,TURN_SPEED);
					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
					ActList_Add(MOVE_ACT_STATIC,0,0,0);																				
					Action_SetMove(MOVE_ACT_HANDLER);		
					No_Command_Counter=0;
				}
				if((Rcon_GetRemoteCode() == Remote_Max) || (Wifi_GetRemoteCode() == Remote_Max))
				{
					Speaker(SPK_DON);
					ActList_Clear();
					ActList_Add(MOVE_ACT_TURN_ROUND,1550,TURN_SPEED,TURN_SPEED);
					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
					ActList_Add(MOVE_ACT_STATIC,0,0,0);																				
					Action_SetMove(MOVE_ACT_HANDLER);		
					No_Command_Counter=0;
				}						
			}
		}
		
		if(Bumper_GetTrigStatus()) 
		{
			ActList_Clear();
			ActList_Add(MOVE_ACT_BACK,move_back_distance,BACK_SPEED,BACK_SPEED);
			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
			ActList_Add(MOVE_ACT_STATIC,0,0,0);																				
			Action_SetMove(MOVE_ACT_HANDLER);	
			back_to_userinterface_flag = 1;
			Usprintf("%s(%d):Bumper Event!\n",__FUNCTION__, __LINE__);
		}
		if(Cliff_GetDetectiontProcess_Result())
		{
			if(Cliff_GetDetectiontProcess_Result())
			{
				if(Cliff_GetDetectiontProcess_Result() == CLIFF_ALL_TRIG)
				{
					Error_SetCode(ERROR_PICK_UP);
				}
				else
				{
					ActList_Clear();
					ActList_Add(MOVE_ACT_BACK,move_back_distance,BACK_SPEED,BACK_SPEED);
					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
					ActList_Add(MOVE_ACT_STATIC,0,0,0);																				
					Action_SetMove(MOVE_ACT_HANDLER);	
					back_to_userinterface_flag = 1;			
					Usprintf("%s(%d):Cliff Event!\n",__FUNCTION__, __LINE__);						
				}			
			}
		}
		if(Error_GetCode() != ERROR_NONE)
		{
			Mode_SetMode(MODE_USERINTERFACE);
		  break;		  		
		}		
	  No_Command_Counter++;
    if(No_Command_Counter > 200)//10 s
    {
      No_Command_Counter=0;
			Usprintf("%s(%d):No_Command_Counter!\n",__FUNCTION__, __LINE__);
			Mode_SetMode(MODE_USERINTERFACE);
		  break;
    }
		
    if(Battery_IsLow())
		{
			Usprintf("%s(%d):low battery!\n",__FUNCTION__, __LINE__);
			Mode_SetMode(MODE_USERINTERFACE);
			break;
    }

		if(Motor_GetStatus())
		{
			Usprintf("%s(%d):motor error!\n",__FUNCTION__, __LINE__);
		  Mode_SetMode(MODE_USERINTERFACE);
			break;
		}
		if(ret_val)break;
		vTaskDelayUntil(&xLastWakeTime,50/portTICK_RATE_MS);
	}
}






