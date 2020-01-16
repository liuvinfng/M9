 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Move near the wall on the left in a certain distance
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



volatile int32_t  g_wallfollow_move_distance = 0;
volatile int32_t  g_wallfollow_distance=Wall_High_Limit;

volatile uint8_t  g_wallfollow_first_in_fag=0;
volatile uint8_t  g_wallfollow_circle_flag=0;
volatile uint32_t g_wallfollow_time=0; 	  
volatile Point16_t g_wall_cell[30] = {0};
volatile uint16_t  g_wall_idx = 0;

void WallFollow_UpdateCells(Point16_t robot_cell)
{
	static Point16_t last_cell = {0};
	static uint16_t not_moving_cnt = 0;
	if((robot_cell.X != last_cell.X)||(robot_cell.Y != last_cell.Y))
	{
		g_wall_cell[g_wall_idx] = robot_cell;
		g_wall_idx++;	
		Usprintf("%s(%d)idx : %d robot cell :(%d,%d)\n",__FUNCTION__,__LINE__,g_wall_idx,robot_cell.X,robot_cell.Y);		
		if(g_wall_idx >= 30)
		{
			g_wall_idx = 0; 
		}
		not_moving_cnt = 0;
	}
	else
	{
		not_moving_cnt++;
		Usprintf("%s(%d)wall not moving cnt :%d\n",__FUNCTION__,__LINE__,not_moving_cnt);
		if(not_moving_cnt > 20)//10s
		{
			not_moving_cnt = 0;
			Error_SetCode(ERROR_STUCK);
		}
	}
	last_cell = robot_cell;
}
void WallFollow_ResetCells(void)
{
	uint8_t i = 0;
	for(i = 0;i < g_wall_idx;i++)
	{
	  g_wall_cell[i].X = 0;
		g_wall_cell[i].Y = 0;	
	}	
	g_wall_idx = 0;
}

uint8_t WallFollow_CheckSpinningWithObs(void)
{
	uint8_t i = 0;
	Point16_t temp_robot_cell = Map_GetRobotCell();
	for(i = 0;i < g_wall_idx - 1;i++)
	{
	  if((g_wall_cell[i].X == temp_robot_cell.X)&&(g_wall_cell[i].Y == temp_robot_cell.Y))
		{
			Usprintf("%s(%d)idx : %d Spinning cell :(%d,%d)\n",__FUNCTION__,__LINE__,i,temp_robot_cell.X,temp_robot_cell.Y);	
			return 1 ;
		}
	}
  return 0 ;	
}
void Wall_Follow_Mode(void)
{	
  uint8_t wall_follow_time_cnt = 0;
  uint8_t is_bumper_triggered =0;	
	uint8_t is_cliff_triggered =0;
	uint8_t is_obs_triggered =0;	
	uint32_t wall_start_time = 0;
	int32_t wallfollow_straight_distance = 100;
	int32_t wallfollow_left_speed = 0;
	int32_t wallfollow_right_speed = 0;
	int32_t proportion = 0;
	int32_t delta = 0, cycle_r = 0;
	int32_t previous = 0;
	int16_t right_wall_buffer[3] = {0};
	uint8_t temp_motor_status = 0;
	uint16_t gyro_init_cnt = 0,gyro_error_cnt = 0;
  Point16_t home_point = {0};
	uint8_t find_wall_flag = 0;
	int32_t t_adjust = 0,last_straight_wall_time = 0,meet_big_angle_time = 0;
	uint8_t straight_wall_t_cnt = 0,big_angle_wall_cnt = 0;
	int32_t temp_wall_distance = g_wallfollow_distance;
	int32_t robot_angle_integrater = 0,robot_angle_buffer = 0;	
	uint16_t gyro_integrater_cnt = 0;
	uint16_t cliff_cnt = 0,cliff_event = 0;
	uint8_t all_cliff_trigger_flag = 0;
//	uint8_t mobility_event = 0;
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();	
	Wall_SetDynamicState(DISABLE);			

	ICM42688_Config();
	Gyro_Cmd(DISABLE);
	osDelay(300);
	osDelay(300);
	Gyro_Cmd(ENABLE);
	osDelay(300);	
	osDelay(300);
	
	Motor_WorkConfigure(); 
  Map_Initialize();	
	
	WallFollow_ResetCircleFlag();
	WallFollow_ResetWheelStep();
	Bumper_ResetErrorCnt();
	Rcon_ResetStatus();
	WallFollow_ResetSpinningTime();

	Wall_AdjustBaseline();
	if(g_obs_adc.Right_Wall < 200)
	{ 
    WallFollow_ResetRightBaseLine();
	}
	Usprintf("wall right base line:%d\n",Wall_GetRightBaseline());
	Action_SetMove(MOVE_ACT_STARTUP);
			

	WallFollow_SetDistance(60000);//60m
	WallFollow_SetTime(600);//10 min
	g_wallfollow_distance = Wall_High_Limit;
	wall_start_time = Time_GetCurrentTime();
	find_wall_flag = 1;	
	
	while(Mode_GetMode() == MODE_WALL)
	{
		switch(Action_GetMove())
		{
			case MOVE_ACT_STARTUP:	
															gyro_init_cnt++;
															if(gyro_init_cnt > 300)
															{
																if(!Gryo_GetUpdateFlag()) 
																{
																	Usprintf("%s(%d):No Gyro Data!\n",__FUNCTION__, __LINE__);
																	Gyro_Cmd(ENABLE);
																	gyro_init_cnt = 0;
																	gyro_error_cnt++;
																	if(gyro_error_cnt > 1)
																	{
																		Error_SetCode(ERROR_GYRO);
																		Mode_SetMode(MODE_USERINTERFACE);
																		break;
																	}
																}
																else
																{
																	Usprintf("%s(%d):Gryo_GetUpdateFlag = %d\n",__FUNCTION__, __LINE__,Gryo_GetUpdateFlag());     	
																	OBS_ResetTrigValue();
																	Gyro_Calibration_Cmd(DISABLE);
																	Wall_SetDynamicState(DISABLE);
																	Action_MoveForward(MAX_SPEED,MAX_SPEED,MAX_DISTANCE,MOVE_ACT_SEARCHWALL);
																	Action_SetMove(MOVE_ACT_SEARCHWALL); 																	
																}
															}				
															break;
															
      case MOVE_ACT_SEARCHWALL: 	
																if(OBS_IsNear())
																{
																	Wheel_SetTargetSpeed(RUN_SLOW_SPEED,RUN_SLOW_SPEED);
																}				
																is_obs_triggered = OBS_GetTrigStatus();
																if(is_obs_triggered)
																{
															    ActList_Clear();
																	ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
//																	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);																	
																	if(is_obs_triggered & OBS_LEFT_TRIG)
																	{
																	  ActList_Add(MOVE_ACT_TURN_LEFT,1000,TURN_SPEED,TURN_SPEED);
																	}
																	else
																	{
																	  ActList_Add(MOVE_ACT_TURN_LEFT,600,TURN_SPEED,TURN_SPEED);
																	}																	
//																	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//																	ActList_Add(MOVE_ACT_STATIC,0,0,0);
																	Action_SetMove(MOVE_ACT_HANDLER);
																	WallFollow_ResetWallAccelerate();
																	WallFollow_ResetWheelStep();
																	break;																	
																}
																if(Wall_GetRightAverageAdcValue() > 400)
																{
																	ActList_Clear();
																	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
																	ActList_Add(MOVE_ACT_STATIC,0,0,0);
																	Action_SetMove(MOVE_ACT_HANDLER);																		
																	WallFollow_ResetWallAccelerate();
																	WallFollow_ResetWheelStep();  
																}																
																if(Rcon_GetRemoteCode())
																{	
																	if(Rcon_GetRemoteCode() == Remote_Right)
																	{
																		Speaker(SPK_DON);
																		ActList_Clear();
																		ActList_Add(MOVE_ACT_TURN_RIGHT,300,TURN_SPEED,TURN_SPEED);
																		ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
																		ActList_Add(MOVE_ACT_SEARCHWALL,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);																		
																		Action_SetMove(MOVE_ACT_HANDLER);
																	}
																	if(Rcon_GetRemoteCode() == Remote_Left)
																	{
																		Speaker(SPK_DON);
																		ActList_Clear();
																		ActList_Add(MOVE_ACT_TURN_LEFT,300,TURN_SPEED,TURN_SPEED);
																		ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
																		ActList_Add(MOVE_ACT_SEARCHWALL,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
																		Action_SetMove(MOVE_ACT_HANDLER);																	
																	}
																}
																break;
																
			case MOVE_ACT_FORWARD:    Action_WaitForMoveEnded();
																if(OBS_IsNear())
																{
																	Wheel_SetTargetSpeed(RUN_SLOW_SPEED,RUN_SLOW_SPEED);
																}
																/*if(WallFollow_CheckTinterWall(&temp_wall_distance,80))//edit by vin
																{
																	g_wallfollow_distance = temp_wall_distance;
																}*/
																break;
			case MOVE_ACT_WALL: 		 																
																wall_follow_time_cnt++;
				                        if(wall_follow_time_cnt > 5)//60ms
																{
																		t_adjust++;//straight time
																		wall_follow_time_cnt = 0;																	
																		temp_wall_distance = g_wallfollow_distance;
																		if(g_wallfollow_distance >= 100)
																		{
																			right_wall_buffer[2] = right_wall_buffer[1];
																			right_wall_buffer[1] = right_wall_buffer[0];
																			right_wall_buffer[0] = Wall_GetRightAverageAdcValue();	
																			if(right_wall_buffer[0] < 100)
																			{
																				if((right_wall_buffer[1] - right_wall_buffer[0]) > (g_wallfollow_distance / 50))
																				{
																					if((right_wall_buffer[2] - right_wall_buffer[1]) > (g_wallfollow_distance / 50))
																					{
																						if(WallFollow_GetWallAccelerate() > 300)
																						{
																							if((Wheel_GetLeftSpeed() - Wheel_GetRightSpeed()) >= 3)
																							{
																								Wheel_SetTargetSpeed(19,21);
																								WallFollow_ResetWallAccelerate();
																								wallfollow_straight_distance = 200;//300
																							}
																						}
																					}
																				}
																			}
																		}																		
																		if(WallFollow_GetWallAccelerate() < wallfollow_straight_distance)
																		{
																			if(Wheel_GetLeftStep() < 500)
																			{
																				if(WallFollow_GetWallAccelerate() < 100)
																				{
																					Wheel_SetTargetSpeed(18, 18);	
																				}
																				else
																				{
																					Wheel_SetTargetSpeed(19, 19);
																				}
																			}
																			else
																			{
																				Wheel_SetTargetSpeed(RUN_SPEED_10, RUN_SPEED_10);
																			}
																		}																																			
																		else/*Main speed adjust*/
																		{				
																				proportion = Wall_GetRightAverageAdcValue();
																																								
																				proportion = proportion * 100 / g_wallfollow_distance;

																				proportion -= 100;
															 
																				delta = proportion - previous;  
																				
																				delta /= 3;																			
																				/*white wall*/
																				if (g_wallfollow_distance > 700)//over left
																				{
																					wallfollow_right_speed = RUN_SPEED_12 + proportion / 12 + delta / 5;
																					wallfollow_left_speed = RUN_SPEED_12 - proportion / 10 - delta / 5;
																					
																					if (wallfollow_left_speed > RUN_SPEED_14) {
																						wallfollow_right_speed = RUN_SPEED_4;
																						wallfollow_left_speed = RUN_SPEED_14;
																					}
																				} 
																				else
																				{
																					wallfollow_right_speed  = RUN_SPEED_10 + proportion / 15 + delta / 7;
																					wallfollow_left_speed = RUN_SPEED_10 - proportion / 12 - delta / 7;

																					if (wallfollow_left_speed > RUN_SPEED_11) {
																						wallfollow_right_speed = RUN_SPEED_3;
																						wallfollow_left_speed = RUN_SPEED_13;
																					}																		
																				}
//																				else if (g_wallfollow_distance > 200){//over left
//																					wallfollow_right_speed  = RUN_SPEED_10 + proportion / 15 + delta / 7;
//																					wallfollow_left_speed = RUN_SPEED_10 - proportion / 12 - delta / 7;

//																					if (wallfollow_left_speed > RUN_SPEED_12) {
//																						//Set_LED(0,0,0);
//																						wallfollow_right_speed = RUN_SPEED_3;
//																						wallfollow_left_speed = RUN_SPEED_13;
//																					}//else Set_LED(100,0,0);
//																				}
//																				else{
//																					wallfollow_right_speed  = RUN_SPEED_8 + proportion / 18 + delta / 10;
//																					wallfollow_left_speed = RUN_SPEED_8 - proportion / 15 - delta / 10;
//																					if(wallfollow_right_speed > RUN_SPEED_10)wallfollow_right_speed = RUN_SPEED_10;
//																					if(wallfollow_left_speed > RUN_SPEED_9) {
//																						//Set_LED(0,0,0);
//																						wallfollow_right_speed = RUN_SPEED_3;
//																						wallfollow_left_speed = RUN_SPEED_10;
//																					}
//																				}
																				/*the left wheel speed can be "-"*/
																				if(wallfollow_left_speed < 0)
																				{
																					wallfollow_left_speed = -wallfollow_left_speed;
																					if(wallfollow_left_speed > 10)wallfollow_left_speed = 10;
																					
																					if(Wheel_GetLeftSpeed() <= 1)
																					{
																						Wheel_SetLeftDir(WHEEL_DIR_BACKWARD);
																					}
																					else if(Wheel_GetLeftDir() == WHEEL_DIR_FORWARD)
																					{
																						wallfollow_left_speed = 0;
																					}
																				}
																				else
																				{
																					if(Wheel_GetLeftDir() != WHEEL_DIR_FORWARD)Wheel_SetLeftSpeed(0);
																					Wheel_SetLeftDir(WHEEL_DIR_FORWARD);
																				}																				
																				/*common speed limit*/
																				if (wallfollow_left_speed < 0)
																				{
																					wallfollow_left_speed = 0;
																				}
																				if (wallfollow_right_speed < 0)
																				{
																					wallfollow_right_speed = 0;
																				}	
																				if (wallfollow_left_speed > RUN_SPEED_15) {
																					wallfollow_left_speed = RUN_SPEED_15;
																				}																				
																				/*the obs is near the wall*/
																				if(OBS_IsNear())
																				{
																					Wheel_SetTargetSpeed(RUN_SLOW_SPEED,RUN_SLOW_SPEED);
																				}			
																				/*save current proportion and set the wall speed*/																	
																				previous = proportion;
																				Wheel_SetTargetSpeed(wallfollow_left_speed,wallfollow_right_speed); 																																																										
																				/*check if follow the straight wall*/
																				if(WallFollow_CheckStraightWall(wallfollow_left_speed,wallfollow_right_speed))
																				{
																				  straight_wall_t_cnt++;
																					if(straight_wall_t_cnt >= 10)
																					{
																					  last_straight_wall_time = t_adjust;																						
																					}
																				}
																				else
																				{
																				  straight_wall_t_cnt = 0;
																				}
																				/*check if follow the big angle wall*/
																				if(WallFollow_CheckBigAngleWall(wallfollow_left_speed,wallfollow_right_speed))
																				{
																				  big_angle_wall_cnt++;
																					if(big_angle_wall_cnt >= 2)
																					{
																					  meet_big_angle_time = t_adjust;		
//																						Usprintf("first big angle wall time :%d\n",meet_big_angle_time);
																						if((meet_big_angle_time - last_straight_wall_time) < 72)
																						{																						
																						  wallfollow_left_speed = 32;
																							wallfollow_right_speed = 4;
																						}																																												
																					}
																				}
																				else
																				{
																				  big_angle_wall_cnt = 0;
																				}																				
																				
																				/*check if the robot has wall follow for a circle*/																				
																				if(WallFollow_GetLeftwheelStep() > WallFollow_GetRightwheelStep())
																				{	
																					 cycle_r = WallFollow_GetLeftwheelStep() - WallFollow_GetRightwheelStep();
																				}
																				if(cycle_r > 7500)
																				{	
//																					if(WallFollow_CheckHome(home_point))//edit by vin
//																					{
//																						WallFollow_SetCircleFlag(); 
//																					}																					
																				}	
																		}																																																			
																}																																																													
																/*obs*/
																is_obs_triggered = OBS_GetTrigStatus();
																if(is_obs_triggered)
																{
																	WallFollow_ResetSpinningTime();						
																	if(is_obs_triggered & (OBS_LEFT_TRIG|OBS_FRONT_TRIG))
																	{
																		ActList_Clear();
																		ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
																		ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
																		ActList_Add(MOVE_ACT_TURN_LEFT,800,TURN_SPEED,TURN_SPEED);																
																		ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
																		ActList_Add(MOVE_ACT_WALL,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
																		Action_SetMove(MOVE_ACT_HANDLER);
																		g_wallfollow_distance = Wall_High_Limit;
																		WallFollow_ResetWallAccelerate();																																			
																	}	
																	break;																	
																}																
																/*remote*/
																if(Rcon_GetRemoteCode())
																{	
																	if(Rcon_GetRemoteCode() == Remote_Left)
																	{
																		Speaker(SPK_DON);
																		ActList_Clear();
																		ActList_Add(MOVE_ACT_TURN_LEFT,300,TURN_SPEED,TURN_SPEED);
																		ActList_Add(MOVE_ACT_DECELERATE,0,0,0);																		
																		ActList_Add(MOVE_ACT_WALL,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
																		Action_SetMove(MOVE_ACT_HANDLER);																	
																	}
																}
																/*finish wall follow*/
																if(WallFollow_GetRightwheelStep() >= WallFollow_GetDistance())
																{
//																	Mode_SetMode(MODE_USERINTERFACE);//edit by vin																	
																	Usprintf("distance finish!!\n");																						
																}
																break;
			case MOVE_ACT_DECELERATE:	if((Wheel_GetLeftSpeed() <= 10)&&(Wheel_GetRightSpeed() <= 10))
																{
																	Action_SetMove(MOVE_ACT_HANDLER);				
																}
																break;
			case MOVE_ACT_TURN_RIGHT: if(Rcon_GetRemoteCode() == Remote_Right)
																{
																	Rcon_ResetRemoteCode();
																	Wheel_SetTargetStep(Wheel_GetLeftStep() + 300,Wheel_GetRightStep()+300);
																}
//																if(Wall_GetRightAdcValue() > 150)
//																{
//																	g_wallfollow_distance = Wall_High_Limit - 30;
//																}
																Action_WaitForMoveEnded();																	
																break;

			case MOVE_ACT_TURN_LEFT:  if(Rcon_GetRemoteCode() == Remote_Left)
																{
																	Rcon_ResetRemoteCode();
																	Wheel_SetTargetStep(Wheel_GetLeftStep() + 300,Wheel_GetRightStep()+300);
																}
//																if(Wall_GetRightAdcValue() > 150)
//																{
//																	g_wallfollow_distance = Wall_High_Limit - 30;
//																}																
																Action_WaitForMoveEnded();															
																break;																
			case MOVE_ACT_BACK:       Action_WaitForMoveEnded();
																break;																	
			case MOVE_ACT_HANDLER:				
				                        ActList_Switch();
																break;

			case MOVE_ACT_STATIC:    
																Wheel_Stop();
																if(all_cliff_trigger_flag == 1)
																{
																	all_cliff_trigger_flag = 0;
																	if(Cliff_GetDetectiontProcess_Result() == CLIFF_ALL_TRIG)
																	{
																		Usprintf("%s(%d):Pick up!\n",__FUNCTION__, __LINE__);
																	  Error_SetCode(ERROR_PICK_UP);
																		Mode_SetMode(MODE_USERINTERFACE);
																		break;	
																	}
																}
																if(find_wall_flag == 1)
																{
																	home_point = Map_GetRobotCell();	
																	WallFollow_ResetCells();
																	Usprintf("home point x :%d  y:%d \n",home_point.X,home_point.Y)
																  find_wall_flag = 0;																  
																}
																Action_MoveForward(MAX_SPEED,MAX_SPEED,MAX_DISTANCE,MOVE_ACT_WALL);
																break;
	  																
			default:break;
		}
		if((Time_GetCurrentTime()- wall_start_time) >= WallFollow_GetTime())
		{
			Usprintf("time finish!!\n");
//			Mode_SetMode(MODE_USERINTERFACE);//edit by vin
		}		
		if(Action_GetMove() == MOVE_ACT_WALL)//checking the finish conditions
		{
			gyro_integrater_cnt++;
			if(gyro_integrater_cnt > 49)
			{
				gyro_integrater_cnt = 0;
				robot_angle_buffer -= Gyro_GetAngle(0);
				robot_angle_buffer = Math_RoundAngle(robot_angle_buffer);
				robot_angle_integrater -= robot_angle_buffer;
				Usprintf("%s(%d):intergrater = %d \n",__FUNCTION__, __LINE__,robot_angle_integrater);
				if((robot_angle_integrater <= -3600)||(robot_angle_integrater >= 3600))
				{
//					if(Path_RobotCloseToTargetCell(home_point,2))//edit by vin
//					{
//						Mode_SetMode(MODE_USERINTERFACE);
//						break;																		  
//					}									
				}
				if(robot_angle_integrater < -5400)//island
				{							
					Usprintf("%s(%d):robot trapped in a island \n",__FUNCTION__, __LINE__);
					ActList_Clear();
					ActList_Add(MOVE_ACT_BACK,200,0,BACK_SPEED);
					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
					ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,0);
					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);					
					ActList_Add(MOVE_ACT_TURN_LEFT,600,TURN_SPEED,TURN_SPEED);
					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
					ActList_Add(MOVE_ACT_SEARCHWALL,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
					Action_SetMove(MOVE_ACT_HANDLER);
					WallFollow_ResetWallAccelerate();															
				}
				if(robot_angle_integrater > 5400)//turn around for 360 degree,and 
				{
//					Mode_SetMode(MODE_USERINTERFACE);//edit by vin
//					break;										
				}
				robot_angle_buffer = Gyro_GetAngle(0);		
			}				
		}		
		if((Action_GetMove() != MOVE_ACT_BACK)&&(Action_GetMove() != MOVE_ACT_STATIC)&&(Action_GetMove() != MOVE_ACT_STARTUP))
		{		
			//cliff event
			is_cliff_triggered = Cliff_GetDetectiontProcess_Result();
			if(is_cliff_triggered)
			{
				is_cliff_triggered = Cliff_GetDetectiontProcess_Result();
				if(is_cliff_triggered)
				{
					g_wallfollow_distance = Wall_High_Limit;
					if(is_cliff_triggered == CLIFF_ALL_TRIG)
					{
						if(all_cliff_trigger_flag == 0)
						{
							all_cliff_trigger_flag = 1;
							ActList_BackToStop(500);
						}
						else
						{
							Usprintf("%s(%d):Pick up!\n",__FUNCTION__, __LINE__);
							Error_SetCode(ERROR_PICK_UP);
							Mode_SetMode(MODE_USERINTERFACE);
							break;						
						}
					}
					else
					{
						if(cliff_event == 1)
						{
							cliff_cnt++;
						}					
						if(cliff_cnt > 2)
						{
							cliff_cnt = 0;
							Error_SetCode(ERROR_CLIFF);
						}
						cliff_event = 1;
						if(is_cliff_triggered & CLIFF_LEFT_TRIG)
						{
							ActList_Clear();
							ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							ActList_Add(MOVE_ACT_TURN_LEFT,700,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							if(Action_GetMove() == MOVE_ACT_SEARCHWALL)
							{				  
								ActList_Add(MOVE_ACT_STATIC,0,0,0);
							}					
							else
							{
								ActList_Add(MOVE_ACT_FORWARD,100,WALL_WALK_SPEED,WALL_WALK_SPEED);
								ActList_Add(MOVE_ACT_WALL,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
							}
							Action_SetMove(MOVE_ACT_HANDLER);
							WallFollow_ResetWallAccelerate();	 
							WallFollow_ResetWheelStep();																																		
						}
						else if(is_cliff_triggered & CLIFF_FRONT_TRIG)
						{
							ActList_Clear();
							ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							ActList_Add(MOVE_ACT_TURN_LEFT,500,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							if(Action_GetMove() == MOVE_ACT_SEARCHWALL)
							{				  
								ActList_Add(MOVE_ACT_STATIC,0,0,0);
							}					
							else
							{
								ActList_Add(MOVE_ACT_FORWARD,100,WALL_WALK_SPEED,WALL_WALK_SPEED);
								ActList_Add(MOVE_ACT_WALL,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
							}
							Action_SetMove(MOVE_ACT_HANDLER);
							WallFollow_ResetWallAccelerate();	 
							WallFollow_ResetWheelStep();																																		
						}																	
						else if(is_cliff_triggered & CLIFF_RIGHT_TRIG)
						{
							ActList_Clear();
							ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							ActList_Add(MOVE_ACT_TURN_LEFT,400,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							if(Action_GetMove() == MOVE_ACT_SEARCHWALL)
							{				  
								ActList_Add(MOVE_ACT_STATIC,0,0,0);
							}					
							else
							{
								ActList_Add(MOVE_ACT_FORWARD,100,WALL_WALK_SPEED,WALL_WALK_SPEED);
								ActList_Add(MOVE_ACT_WALL,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
							}
							Action_SetMove(MOVE_ACT_HANDLER);
							WallFollow_ResetWallAccelerate();	 
							WallFollow_ResetWheelStep();																																		
						}																																							
					}								
				}
			}
			else
			{
				cliff_event = 0;
				cliff_cnt = 0;
			}	
			
			//station
			/*if((wall_dir == WALLDIR_WEST_LEFT) || (wall_dir == WALLDIR_EAST_LEFT))
			{
				if(Is_Left_Meet_Station())
				{
					Rcon_ResetStatus();
					ActList_Clear();
					ActList_Add(MOVE_ACT_TURN_RIGHT,300,TURN_SPEED,TURN_SPEED/2);
					ActList_WallStraightAndForward(0,300);
					Action_SetMove(MOVE_ACT_HANDLER);
				}
			}
			else
			{
				if(Is_Right_Meet_Station())
				{
					Rcon_ResetStatus();
					ActList_Clear();
					ActList_Add(MOVE_ACT_TURN_LEFT,300,TURN_SPEED/2,TURN_SPEED);
					ActList_WallStraightAndForward(0,300);
					Action_SetMove(MOVE_ACT_HANDLER);
				}			
			}*/			
			//bumper event
			is_bumper_triggered = Bumper_GetTrigStatus();
			if(is_bumper_triggered)
			{
				if(is_bumper_triggered & LeftBumperTrig)
				{
					ActList_Clear();
					ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
//					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
					ActList_Add(MOVE_ACT_TURN_LEFT,700,TURN_SPEED/2,TURN_SPEED);
//					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
					if(Action_GetMove() == MOVE_ACT_SEARCHWALL)
					{				  
						ActList_Add(MOVE_ACT_STATIC,0,0,0);
					}					
					else
					{
						ActList_Add(MOVE_ACT_FORWARD,100,WALL_WALK_SPEED,WALL_WALK_SPEED);
						ActList_Add(MOVE_ACT_WALL,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
					}				
					Action_SetMove(MOVE_ACT_HANDLER);				
					WallFollow_ResetWallAccelerate();	 
					WallFollow_ResetWheelStep();
					g_wallfollow_distance = Wall_High_Limit;				
				}		
				else//right bumper
				{
					ActList_Clear();
					if(Action_GetMove() == MOVE_ACT_SEARCHWALL)
					{
						ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
//						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
						ActList_Add(MOVE_ACT_TURN_LEFT,400,TURN_SPEED/2,TURN_SPEED);
//						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);						
					}					
					else
					{
//						if(is_bumper_triggered & FRightBumperTrig)ActList_Add(MOVE_ACT_BACK,100,BACK_SPEED,BACK_SPEED);
						if(OBS_L_F_Near())
						{
							ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
							ActList_Add(MOVE_ACT_TURN_LEFT,900,TURN_SPEED/2,TURN_SPEED);
						}
						else 
						{
							ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,0);
							ActList_Add(MOVE_ACT_FORWARD,400,WALL_WALK_SPEED,WALL_WALK_SPEED);
						}					
						ActList_Add(MOVE_ACT_WALL,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
					}
					Action_SetMove(MOVE_ACT_HANDLER);
					WallFollow_ResetWallAccelerate();	 
					WallFollow_ResetWheelStep();
					g_wallfollow_distance = g_wallfollow_distance - 30;
					if(g_wallfollow_distance < Wall_Low_Limit)
					{
						g_wallfollow_distance = Wall_Low_Limit;
					}				
				}		
			}
		

			
		}													
		/*abnormal event*/
		temp_motor_status = Motor_GetStatus();
		if(temp_motor_status)
		{
			Motor_ResetStatus();
			if(!((temp_motor_status == CHECK_MOBILITY)&&CM_GetMobilityIgnoreFlag()))
			{
			  CM_AbnormalHandler(temp_motor_status);					 
				 Action_MoveForward(MAX_SPEED,MAX_SPEED,MAX_DISTANCE,MOVE_ACT_WALL);		
			}	
		}			
		/*Error Code*/
		if(Error_GetCode())
		{			
			Mode_SetMode(MODE_USERINTERFACE);
		}		
		vTaskDelayUntil(&xLastWakeTime,10/portTICK_RATE_MS);
	}
}
/*wall follow distance : wd mm*/
void WallFollow_SetDistance(uint32_t wd)
{		
  g_wallfollow_move_distance = wd*8; 
}
int32_t WallFollow_GetDistance(void)
{
  return  g_wallfollow_move_distance;
}
void WallFollow_ResetDistance(void)
{
  g_wallfollow_move_distance = 0;
}
/*wall follow circle*/
void WallFollow_SetCircleFlag(void)
{
  g_wallfollow_circle_flag = 1;
}
void WallFollow_ResetCircleFlag(void)
{
  g_wallfollow_circle_flag = 0;
}
uint8_t WallFollow_GetCircleFlag(void)
{
  return g_wallfollow_circle_flag;
}
/*wall follow time*/
void WallFollow_SetTime(uint32_t wft)  
{
  g_wallfollow_time = wft;
}
void WallFollow_ResetTime(uint32_t wft)  
{
  g_wallfollow_time = 0;
}
uint32_t WallFollow_GetTime(void)
{
  return g_wallfollow_time;
}



/*reset the right wall baseline*/
void WallFollow_ResetRightBaseLine(void)
{
  Wall_SetRightBaseline(g_obs_adc.Right_Wall);
}
/*check if back home*/
uint8_t WallFollow_CheckHome(Point16_t home_point)
{
  Point16_t temp_point  = Map_GetRobotCell(); 
	
	uint8_t reval = 0;	
	
	if((temp_point.X > home_point.X - 2)&&(temp_point.X < home_point.X + 2))
	{
	  if((temp_point.Y > home_point.Y - 2)&&(temp_point.Y < home_point.Y + 2))
		{
		  reval = 1;  
		}		
	}
	
	return reval;
}

/*check if follow the straight wall*/
uint8_t WallFollow_CheckStraightWall(uint16_t l_speed,uint16_t r_speed)
{
	uint8_t reval = 0;
	
	if(l_speed >= r_speed)	
	{
    if((l_speed - r_speed)<=1)
		{
		  reval = 1;		
		}
	}
	else
	{
    if((r_speed - l_speed)<=1)
		{
		  reval = 1;		
		}		
	}
		
	return reval;
}
/*check if follow the big angle wall*/
uint8_t WallFollow_CheckBigAngleWall(uint16_t l_speed,uint16_t r_speed)
{
	uint8_t reval = 0;
	
	if(l_speed >= r_speed)	
	{
    if((l_speed - r_speed) >= 20)
		{
		  reval = 1;		
		}
	}
	
	return reval;
}

/*check if meet the tinter color wall*/
uint8_t WallFollow_CheckTinterWall(int32_t *wall_distance,int32_t wall_adc_distance_diff)
{	
	uint8_t reval = 0;
	static uint8_t cnt = 0;
		
	if(Wall_GetRightAverageAdcValue() > (*wall_distance))
  {
	  if((Wall_GetRightAverageAdcValue() - (*wall_distance)) > wall_adc_distance_diff)     
		{
			cnt++;
			if(cnt > 2)
			{
				if(Wall_GetRightAverageAdcValue() < Wall_High_Limit)
				{
				  *wall_distance = Wall_GetRightAverageAdcValue()- 50;
				}
				else
				{
				  *wall_distance = Wall_High_Limit;
				}
				reval = 1;
				Usprintf("meet tinter color wall , new wall distance:%d\n",*wall_distance);			
			}
		}
		else
		{
		  cnt = 0;
		}
	}	
	else
	{
	  cnt = 0;
	}
	
	return reval;
	
}
/*check spinning*/
volatile uint16_t g_spinning_time = 0;
uint8_t WallFollow_CheckSpinning(uint16_t l_speed,uint16_t r_speed)
{
	uint8_t reval = 0;	
	if((l_speed - r_speed) > 20)
	{
		g_spinning_time++;
		if(g_spinning_time >= 320)
		{
			g_spinning_time = 0;
			Usprintf("wall follow spinning!!!\n")
		  reval = 1;   
		}	
	}
	else
	{
	  g_spinning_time = 0;
	}
	
	return reval;
}

void WallFollow_ResetSpinningTime(void) 
{
  g_spinning_time = 0; 
}
