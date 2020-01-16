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


#define Robot_Home_DistanceFar	  0x01
#define Robot_Home_DistanceNear   0x0
volatile uint8_t Position_Far=0,Robot_Near_Cnt=0;

void HomeStraight_Mode(void)
{
 	portTickType xLastWakeTime = xTaskGetTickCount();	
	volatile uint32_t Receive_Code = 0,left_wheel_step_buffer=0;
	volatile uint8_t dir = 0,path_bumper_counter=0,around_no_signal_counter=0,just_homepath_cycle=0,home_around_cycle=0;
	volatile uint8_t stuck = 0,bumper_counter=0,wall_bumper_factor=0,wall_bumper_counter=0,wall_small_counter=0,wall_mid_counter=0;
	static uint8_t path_no_signal_counter=0,head2base_no_signal_counter=0,signal_counter=0,forward_no_signal_counter=0;
	uint8_t start_up_time_cnt = 0,fl_fr_flag=0,fl_fr_counter=0;
  uint8_t is_bumper_triggered =0;	
	uint8_t is_cliff_triggered =0;
	uint8_t is_obs_triggered =0;	
	
	wall_bumper_factor = System_GetRandomValue()/15;//random num
	Position_Far = Robot_Home_DistanceFar;//set robot home distance far
	Robot_Near_Cnt = 0;//clear Near counter
	
	Motor_HomeConfigure();
	Action_SetMove(MOVE_ACT_STARTUP);
	vTaskDelay(1300/portTICK_RATE_MS);//wait cliff sensors get data
	Usprintf("%s(%d):Go Home Start!!\n",__FUNCTION__, __LINE__);
	
	while(Mode_GetMode()==MODE_HOME)
	{
		/*reset home remote code for half turn and move act by pass*/
		if(Rcon_GetRemoteCode()==Remote_Home)Rcon_ResetRemoteCode();
		if(Rcon_GetRemoteCode()==Remote_Max)Rcon_ResetRemoteCode();
		if(Rcon_GetRemoteCode()==Remote_Forward)Rcon_ResetRemoteCode();
				
	  switch(Action_GetMove())
		{
			case MOVE_ACT_STARTUP:							                        
																	start_up_time_cnt++;
																	Rcon_ResetStatus();
																	Receive_Code=0;
																	if(start_up_time_cnt > 1)
																	{
																		start_up_time_cnt = 0;
																		Action_MoveForward(MAX_SPEED,MAX_SPEED,MAX_DISTANCE,MOVE_ACT_SEARCHSTATION);
																		Action_SetMove(MOVE_ACT_SEARCHSTATION); 
																		OBS_ResetTrigValue();
																	}
																	break;	
																	
		  case MOVE_ACT_FORWARD:       
																	Action_WaitForMoveEnded();		     				
																	break;
		
		  case MOVE_ACT_BACK:
																	Action_WaitForMoveEnded();																
																	break;							
			case MOVE_ACT_BYPASS_RIGHT: 			
																	if((!Bumper_GetTrigStatus()) && (!Cliff_GetDetectiontProcess_Result()) && (!Rcon_GetRemoteCode()))
																	{
																		Action_WaitForMoveEnded();	
																	}
																	else
																	{
																		Action_SetMove(MOVE_ACT_HANDLER);
																	}																															
																	break;
			case MOVE_ACT_BYPASS_LEFT:  			
																	if((!Bumper_GetTrigStatus()) && (!Cliff_GetDetectiontProcess_Result()) && (!Rcon_GetRemoteCode()))
																	{
																		Action_WaitForMoveEnded();	
																	}
																	else
																	{
																		Action_SetMove(MOVE_ACT_HANDLER);
																	}																															
																	break;
			case MOVE_ACT_HALF_TURN_LEFT: 		
																	if((!Bumper_GetTrigStatus()) && (!Cliff_GetDetectiontProcess_Result()) && (!Rcon_GetRemoteCode()))
																	{
																		Action_WaitForMoveEnded();	
																	}
																	else
																	{
																		Action_SetMove(MOVE_ACT_HANDLER);
																	}																															
																	break;
			case MOVE_ACT_HALF_TURN_RIGHT:  	
																	if((!Bumper_GetTrigStatus()) && (!Cliff_GetDetectiontProcess_Result()) && (!Rcon_GetRemoteCode()))
																	{
																		Action_WaitForMoveEnded();	
																	}
																	else
																	{
																		Action_SetMove(MOVE_ACT_HANDLER);
																	}																															
																	break;
			case MOVE_ACT_TURN_LEFT:  
//																	if(Bumper_GetTrigStatus())
//																	{
//																		Action_SetMove(MOVE_ACT_HANDLER);
//																	}				
																	if(Rcon_RemoteKey(Remote_Left))
																	{																	
																		Wheel_SetTargetStep(Wheel_GetLeftStep() + 300,Wheel_GetRightStep()+300);
																	}															
																	Action_WaitForMoveEnded();															
																	break;																	
			case MOVE_ACT_TURN_RIGHT: 	
//																	if(Bumper_GetTrigStatus())
//																	{
//																		Action_SetMove(MOVE_ACT_HANDLER);
//																	}
																	if(Rcon_RemoteKey(Remote_Right))
																	{																		
																		Wheel_SetTargetStep(Wheel_GetLeftStep() + 300,Wheel_GetRightStep()+300);
																	}
																	Action_WaitForMoveEnded();																	
																	break;
      case MOVE_ACT_DECELERATE:	
																	if((Wheel_GetLeftSpeed() <= 10) && (Wheel_GetRightSpeed()) <= 10)
																	{															
																		Action_SetMove(MOVE_ACT_HANDLER);	
																	}  
																	break;		

			case MOVE_ACT_STATIC:       
																	if(ActList_GetCnt() != 0)
																	{
																		ActList_Switch();
																	}
																	break;
      case MOVE_ACT_SEARCHSTATION: 
			{
				path_bumper_counter=0;
				if(OBS_IsNear())
				{
					Wheel_SetTargetSpeed(RUN_SLOW_OBS,RUN_SLOW_OBS);															
				}							
				else if(Rcon_GetStatus()&(RconFL_LEFT|RconFR_LEFT|RconL_LEFT))
				{
					Rcon_ResetStatus();
					Wheel_SetTargetSpeed(Wheel_GetLeftTargetSpeed(),Wheel_GetLeftTargetSpeed()/2);//Wheel_GetLeftTargetSpeed(),Wheel_GetRightTargetSpeed()/2
				}
				else if(Rcon_GetStatus()&(RconFL_RIGHT|RconFR_RIGHT|RconR_RIGHT))
				{
					Rcon_ResetStatus();
					Wheel_SetTargetSpeed(Wheel_GetRightTargetSpeed()/2,Wheel_GetRightTargetSpeed());
				}				
				else
				{
					forward_no_signal_counter++;
					if(forward_no_signal_counter>10)
					{
						forward_no_signal_counter=0;
						Wheel_SetTargetSpeed(MAX_SPEED,MAX_SPEED);
					}					
				}																					
																																																			
				if(Bumper_GetTrigStatus() & LeftBumperTrig)
				{
					left_wheel_step_buffer=Wheel_GetLeftStep();
					if(Wheel_GetLeftStep()>14000)//2.64m
					{
						wall_bumper_counter+=2;
					}
					else
					{
						wall_bumper_counter+=3;
					}	
					is_bumper_triggered = Bumper_GetTrigStatus();	
					ActList_Clear();
					ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
					stuck++;
					/*if(stuck>7)
					{
						ActList_Add(MOVE_ACT_TURN_RIGHT,300,TURN_SPEED,TURN_SPEED);
						//Wall_Follow(WALL_TYPE_LEFT,WALL_HOME_MODE,300);//3m
						Wheel_SetDirFlag(WHEEL_DIR_RIGHT);
					}
					else*/
					{
						/*if((wall_small_counter>30)&&(Wheel_GetLeftMoveStep()<300000))
						{
							ActList_Add(MOVE_ACT_TURN_RIGHT,400,TURN_SPEED,TURN_SPEED);
							//Wall_Follow(WALL_TYPE_LEFT,WALL_HOME_MODE,300);//3m
							stuck=0;
							wall_bumper_counter=0;
							wall_small_counter=0;
							wall_mid_counter=0;
							Wheel_ResetMoveStep();
						}
						else if((wall_mid_counter>40)||(Wheel_GetLeftMoveStep()>460000))
						{
							ActList_Add(MOVE_ACT_TURN_RIGHT,400,TURN_SPEED,TURN_SPEED);
							//Wall_Follow(WALL_TYPE_LEFT,WALL_HOME_MODE,500);//5m
							stuck=0;
							wall_bumper_counter=0;
							wall_mid_counter=0;
							wall_small_counter=0;
							Wheel_ResetMoveStep();
						}
						else if((wall_bumper_counter>(SHORT_WALL_TRIG+wall_bumper_factor))&&(System_GetRandomValue()<25))
						{
							ActList_Add(MOVE_ACT_TURN_RIGHT,400,TURN_SPEED,TURN_SPEED);
							//Wall_Follow(WALL_TYPE_LEFT,WALL_HOME_MODE,Get_Average_Move()/53);//53/cm
							stuck=0;
							wall_bumper_counter=0;
							wall_mid_counter=0;
							wall_small_counter=0;
						}
						else*/
						{
							Usprintf("%s(%d):left_stp = %d \n",__FUNCTION__, __LINE__, Wheel_GetLeftStep());
							if(left_wheel_step_buffer<1000)
							{
								if(Direction_IsLastDirRight())
								{
									ActList_Add(MOVE_ACT_TURN_RIGHT,660,TURN_SPEED,TURN_SPEED);
								}
								else
								{
									ActList_Add(MOVE_ACT_TURN_LEFT,660,TURN_SPEED,TURN_SPEED);
								}																				
							}
							else
							{
								if(is_bumper_triggered == AllBumperT)
								{
									ActList_Add(MOVE_ACT_TURN_RIGHT,(800+System_GetRandomValue()*7)/2,TURN_SPEED,TURN_SPEED);
									ActList_Add(MOVE_ACT_HALF_TURN_RIGHT,800+System_GetRandomValue()*7,0,0);
								}
								else
								{
									if(System_GetRandomValue()<60)//60
									{
										ActList_Add(MOVE_ACT_TURN_RIGHT,800,TURN_SPEED,TURN_SPEED);
										ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
										ActList_Add(MOVE_ACT_BYPASS_RIGHT,0,0,0);																							
									}
									else
									{
										ActList_Add(MOVE_ACT_TURN_RIGHT,(700+System_GetRandomValue()*9)/2,TURN_SPEED,TURN_SPEED);
										ActList_Add(MOVE_ACT_HALF_TURN_RIGHT,700+System_GetRandomValue()*9,0,0);																							
									}
								}
							}
						}																				
					}
					bumper_counter++;
					wall_small_counter++;
					wall_mid_counter++;	
					ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
					Action_SetMove(MOVE_ACT_HANDLER);																		
					break;
				}
				
				if(Bumper_GetTrigStatus() & RightBumperTrig)
				{
					left_wheel_step_buffer=Wheel_GetLeftStep();
					if(Wheel_GetLeftStep()>14000)
					{
						wall_bumper_counter+=1;
					}
					else
					{
						wall_bumper_counter+=2;
					}	
					is_bumper_triggered = Bumper_GetTrigStatus();	
					ActList_Clear();
					ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);																		
					stuck++;
					/*if(stuck>7)
					{
						ActList_Add(MOVE_ACT_TURN_LEFT,300,TURN_SPEED,TURN_SPEED);
						//Wall_Follow(WALL_TYPE_RIGHT,WALL_HOME_MODE,300);//3m
						Wheel_SetDirFlag(WHEEL_DIR_LEFT);
					}
					else*/
					{
						/*if((wall_small_counter>30)&&(Wheel_GetLeftMoveStep()<300000))
						{
							ActList_Add(MOVE_ACT_TURN_LEFT,400,TURN_SPEED,TURN_SPEED);
							//Wall_Follow(WALL_TYPE_RIGHT,WALL_HOME_MODE,300);//3m
							stuck=0;
							wall_bumper_counter=0;
							wall_small_counter=0;
							wall_mid_counter=0;
							Wheel_ResetMoveStep();
						}
						else if((wall_mid_counter>40)||(Wheel_GetLeftMoveStep()>460000))
						{
							ActList_Add(MOVE_ACT_TURN_LEFT,400,TURN_SPEED,TURN_SPEED);
							//Wall_Follow(WALL_TYPE_RIGHT,WALL_HOME_MODE,500);//5m
							stuck=0;
							wall_bumper_counter=0;
							wall_mid_counter=0;
							wall_small_counter=0;
							Wheel_ResetMoveStep();
						}
						else if((wall_bumper_counter>(SHORT_WALL_TRIG+wall_bumper_factor))&&(System_GetRandomValue()<25))
						{
							ActList_Add(MOVE_ACT_TURN_LEFT,400,TURN_SPEED,TURN_SPEED);
							//Wall_Follow(WALL_TYPE_RIGHT,WALL_HOME_MODE,Get_Average_Move()/53);//53/cm
							stuck=0;
							wall_bumper_counter=0;
							wall_mid_counter=0;
							wall_small_counter=0;
						}
						else*/
						{
							Usprintf("%s(%d):left_stp = %d \n",__FUNCTION__, __LINE__, Wheel_GetLeftStep());
							if(left_wheel_step_buffer<2000)
							{
								if(Direction_IsLastDirLeft())
								{
									ActList_Add(MOVE_ACT_TURN_LEFT,660,TURN_SPEED,TURN_SPEED);									
								}
								else
								{
									ActList_Add(MOVE_ACT_TURN_RIGHT,660,TURN_SPEED,TURN_SPEED);
								}																				
							}
							else
							{
								if(is_bumper_triggered == AllBumperT)
								{
									ActList_Add(MOVE_ACT_TURN_LEFT,(800+System_GetRandomValue()*6)/2,TURN_SPEED,TURN_SPEED);
									ActList_Add(MOVE_ACT_HALF_TURN_LEFT,800+System_GetRandomValue()*6,0,0);
								}
								else
								{
									if(System_GetRandomValue()<60)//60
									{
										ActList_Add(MOVE_ACT_TURN_LEFT,800,TURN_SPEED,TURN_SPEED);
										ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
										ActList_Add(MOVE_ACT_BYPASS_LEFT,0,0,0);																							
									}
									else
									{
										ActList_Add(MOVE_ACT_TURN_LEFT,(700+System_GetRandomValue()*8)/2,TURN_SPEED,TURN_SPEED);
										ActList_Add(MOVE_ACT_HALF_TURN_LEFT,700+System_GetRandomValue()*8,0,0);																							
									}
								}
							}
						}																				
					}
					bumper_counter++;
					wall_small_counter++;
					wall_mid_counter++;		
					ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
					Action_SetMove(MOVE_ACT_HANDLER);
					break;																		
				}																																														
				/*cliff*/
				is_cliff_triggered = Cliff_GetDetectiontProcess_Result();
				if(is_cliff_triggered)
				{
//					osDelay(200);
//					if(Cliff_GetDetectiontProcess_Result())
					{
						is_cliff_triggered = Cliff_GetDetectiontProcess_Result();
						if(is_cliff_triggered & CLIFF_FRONT_TRIG)
						{
							ActList_Clear();
							ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							ActList_Add(MOVE_ACT_TURN_LEFT,1600,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
							Action_SetMove(MOVE_ACT_HANDLER);																																		
						}						
						if(is_cliff_triggered & CLIFF_LEFT_TRIG)
						{
							ActList_Clear();
							ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							ActList_Add(MOVE_ACT_TURN_RIGHT,500,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
							Action_SetMove(MOVE_ACT_HANDLER);	 																																		
						}																
						if(is_cliff_triggered & CLIFF_RIGHT_TRIG)
						{
							ActList_Clear();
							ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							ActList_Add(MOVE_ACT_TURN_LEFT,500,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
							Action_SetMove(MOVE_ACT_HANDLER);																																	
						}					
					}																																				
				}				
				/*obs*/
				is_obs_triggered = OBS_GetTrigStatus();
				if(is_obs_triggered)
				{
					is_obs_triggered = OBS_GetTrigStatus();
					left_wheel_step_buffer=Wheel_GetLeftStep();
					if(Wheel_GetLeftStep()>14000)
					{
						wall_bumper_counter+=1;
					}
					else
					{
						wall_bumper_counter+=2;
					}	
												
					if(is_obs_triggered == OBS_FRONT_TRIG)
					{
						ActList_Clear();
						ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
						if(System_GetRandomValue()<60)//60
						{
							ActList_Add(MOVE_ACT_TURN_RIGHT,1300,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);																						
						}
						else
						{
							ActList_Add(MOVE_ACT_TURN_LEFT,(700+System_GetRandomValue()*8)/2,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_HALF_TURN_LEFT,700+System_GetRandomValue()*8,0,0);																							
						}											
						ActList_Add(MOVE_ACT_FORWARD,100,BASE_SPEED,BASE_SPEED);
						ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
						Action_SetMove(MOVE_ACT_HANDLER);
						break;
					}
					else if((is_obs_triggered == OBS_ALL_TRIG) || (is_obs_triggered == (OBS_LEFT_TRIG|OBS_RIGHT_TRIG)))
					{
						ActList_Clear();
						ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
						if(System_GetRandomValue()<60)//60
						{
							ActList_Add(MOVE_ACT_TURN_RIGHT,1500,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);																						
						}
						else
						{
							ActList_Add(MOVE_ACT_TURN_LEFT,(700+System_GetRandomValue()*8)/2,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_HALF_TURN_LEFT,700+System_GetRandomValue()*8,0,0);																							
						}													
						ActList_Add(MOVE_ACT_FORWARD,100,BASE_SPEED,BASE_SPEED);
						ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
						Action_SetMove(MOVE_ACT_HANDLER);	
						break;
					}																	
					else if((is_obs_triggered == OBS_LEFT_TRIG) || (is_obs_triggered == (OBS_LEFT_TRIG|OBS_FRONT_TRIG)))
					{
						ActList_Clear();
						ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
						if(System_GetRandomValue()<60)//60
						{
							ActList_Add(MOVE_ACT_TURN_RIGHT,500,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);																						
						}
						else
						{
							ActList_Add(MOVE_ACT_TURN_LEFT,(700+System_GetRandomValue()*8)/2,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_HALF_TURN_LEFT,700+System_GetRandomValue()*8,0,0);																							
						}						
						ActList_Add(MOVE_ACT_FORWARD,100,BASE_SPEED,BASE_SPEED);
						ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
						Action_SetMove(MOVE_ACT_HANDLER);
						break;
					}		
					else if((is_obs_triggered == OBS_RIGHT_TRIG) || (is_obs_triggered == (OBS_RIGHT_TRIG|OBS_FRONT_TRIG)))
					{
						ActList_Clear();
						ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
						if(System_GetRandomValue()<60)//60
						{
							ActList_Add(MOVE_ACT_TURN_LEFT,1300,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);																						
						}
						else
						{
							ActList_Add(MOVE_ACT_TURN_RIGHT,(700+System_GetRandomValue()*8)/2,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_HALF_TURN_RIGHT,700+System_GetRandomValue()*8,0,0);																							
						}						
						ActList_Add(MOVE_ACT_FORWARD,100,BASE_SPEED,BASE_SPEED);
						ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);																	
						Action_SetMove(MOVE_ACT_HANDLER);
						break;
					}
				}
				/*speed up*/
				if(wall_mid_counter>70)
				{
					wall_mid_counter=0;
					Wheel_ResetMoveStep();
				}   
				if(Wheel_GetLeftMoveStep()>(29000))
				{				
					Wheel_ResetMoveStep();																
				}
				if(Wheel_GetLeftMoveStep()>(24000))
				{
					wall_small_counter=0;
				}
				if(Wheel_GetLeftMoveStep()>(2500))
				{
					stuck=0;
					//Reset_Bumper_Error();
					//if(Get_LeftBrush_Stall())Set_LeftBrush_Stall(0);
					//if(Get_RightBrush_Stall())Set_RightBrush_Stall(0);
				}
				else if(Wheel_GetLeftMoveStep()>(750))
				{
					//Base_Wall_On=1;
					//Set_Left_Brush(ENABLE);
					//Set_Right_Brush(ENABLE);
				}																
				/*remote*/
				if(Rcon_GetRemoteCode())
				{	
					if(Rcon_RemoteKey(Remote_Right))
					{
						ActList_Clear();
						ActList_Add(MOVE_ACT_TURN_RIGHT,450,TURN_SPEED,TURN_SPEED);
						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
						ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);																		
						Action_SetMove(MOVE_ACT_HANDLER);
					}
					if(Rcon_RemoteKey(Remote_Left))
					{
						ActList_Clear();
						ActList_Add(MOVE_ACT_TURN_LEFT,450,TURN_SPEED,TURN_SPEED);
						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
						ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
						Action_SetMove(MOVE_ACT_HANDLER);																	
					}
				}
				/*rcon*/					
				if(Rcon_IsGotStationSignal())
				{
					Receive_Code = Rcon_GetStatus();
					Rcon_ResetStatus();
					Rcon_ResetRemoteCode();																	
					ActList_Clear();
//					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);	
/*--------------------------HomeT-----------------*/
					if(Receive_Code&RconL_HomeT)// L  H_T  turn right
					{
						dir = 1;
//						ActList_Add(MOVE_ACT_TURN_RIGHT,400,TURN_SPEED,TURN_SPEED);//1300
						ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,0,0);																
						Action_SetMove(MOVE_ACT_HANDLER);																	
						break;
					}
					if(Receive_Code&RconR_HomeT)// R  H_T  turn left
					{
						dir = 0;
//						ActList_Add(MOVE_ACT_TURN_LEFT,400,TURN_SPEED,TURN_SPEED);//1300
						ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,0,0);																
						Action_SetMove(MOVE_ACT_HANDLER);
						break;
					}																	
					if(Receive_Code&RconFL_HomeT)//FL H_T  turn right
					{
						dir = 1;
//						ActList_Add(MOVE_ACT_TURN_RIGHT,600,TURN_SPEED,TURN_SPEED);//1000
						ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,0,0);																
						Action_SetMove(MOVE_ACT_HANDLER);																	
						break;
					}
					if(Receive_Code&RconFR_HomeT)//FR H_T  turn left
					{
						dir = 0;
//						ActList_Add(MOVE_ACT_TURN_LEFT,600,TURN_SPEED,TURN_SPEED);//1000
						ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,0,0);																
						Action_SetMove(MOVE_ACT_HANDLER);
						break;
					}
					/*--------------BL BR---------------------*/	
					if(Receive_Code&(RconBR_HomeL|RconBR_HomeR))//BR H_R  turn right 
					{
//						dir = 1;
						ActList_Add(MOVE_ACT_TURN_RIGHT,1800,TURN_SPEED,TURN_SPEED);
//						ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,0,0);
						ActList_Add(MOVE_ACT_HOMEPATH,MAX_DISTANCE,0,0);																
						Action_SetMove(MOVE_ACT_HANDLER);																	
						break;
					}

																
					if(Receive_Code&RconFL_HomeR)//FL H_R  turn_left
					{
						dir = 0;
//						ActList_Add(MOVE_ACT_TURN_LEFT,500,TURN_SPEED,TURN_SPEED);
						ActList_Add(MOVE_ACT_HOMEPATH,MAX_DISTANCE,0,0);//
						Action_SetMove(MOVE_ACT_HANDLER);
						break;
					}
					if(Receive_Code&RconFR_HomeL)//FR H_L  turn_right
					{
						dir = 1;
//						ActList_Add(MOVE_ACT_TURN_RIGHT,500,TURN_SPEED,TURN_SPEED);
						ActList_Add(MOVE_ACT_HOMEPATH,MAX_DISTANCE,0,0);						
						Action_SetMove(MOVE_ACT_HANDLER);																	
						break;
					}																	
					if((Receive_Code&RconFL_HomeL) || (Receive_Code&RconFR_HomeR))//FL H_L  turn_right
					{
						ActList_Add(MOVE_ACT_HOMEPATH,MAX_DISTANCE,0,0);
						Action_SetMove(MOVE_ACT_HANDLER);																	
						break;
					}
					
					
					//------------------------------
					if(Receive_Code&RconL_HomeL)// L  H_L
					{
						dir = 1;
//						ActList_Add(MOVE_ACT_TURN_RIGHT,400,TURN_SPEED,TURN_SPEED);						
						ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,0,0);																
						Action_SetMove(MOVE_ACT_HANDLER);																	
						break;
					}
					if(Receive_Code&RconR_HomeR)// R  H_R
					{
						dir = 0;
//						ActList_Add(MOVE_ACT_TURN_LEFT,400,TURN_SPEED,TURN_SPEED);						
						ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,0,0);																
						Action_SetMove(MOVE_ACT_HANDLER);																	
						break;
					}
																																																
				}			
			}
			break;
			
      case MOVE_ACT_HOMEAROUND:
			{					
				home_around_cycle++; 
				if(home_around_cycle >= 15)//15-300ms
				{
					home_around_cycle=0;
					Receive_Code = Rcon_GetStatus();
					Rcon_ResetRemoteCode();
					if(Receive_Code)
					{
						around_no_signal_counter=0;
						Rcon_ResetStatus();
					}
					else
					{
						around_no_signal_counter++;
						if(around_no_signal_counter>=10)//10-3s
						{
							ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
							Action_SetMove(MOVE_ACT_HANDLER);
							break;
						}
					}
					
					if(dir == 1)//anticlockwise
					{
						if(Receive_Code&RconL_HomeT)//L_T
						{
							Wheel_SetTargetSpeed(22,12);//ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,32,22);
//							ActList_Clear();
//							ActList_Add(MOVE_ACT_FORWARD,600,22,12);
//							ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);																
//							Action_SetMove(MOVE_ACT_HANDLER);							
						}
						else if(Receive_Code&RconL_HomeL)//L_L//else 
						{
							forward_no_signal_counter++;
							if(forward_no_signal_counter>5)
							{
								forward_no_signal_counter=0;
								Wheel_SetTargetSpeed(18,12);//ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,28,22);
//								ActList_Clear();
//								ActList_Add(MOVE_ACT_FORWARD,600,18,12);
//								ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);																
//								Action_SetMove(MOVE_ACT_HANDLER);								
							}
							else
							{
								Wheel_SetTargetSpeed(12,18);
							}
						}																				
						else if(Receive_Code & (RconFL_HomeL|RconFR_HomeL))//else 
						{				
							ActList_Clear();
							ActList_Add(MOVE_ACT_TURN_RIGHT,600,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,0,0);																
							Action_SetMove(MOVE_ACT_HANDLER);
							break;
						}																		
						else
						{
							Wheel_SetTargetSpeed(6,22);//ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,14,31);//18-30
						}	

						if(Receive_Code & (RconFL_HomeR|RconFR_HomeR))
						{
							ActList_Clear();
							Action_SetMove(MOVE_ACT_HOMEPATH);
							break;								
						}
						if(Receive_Code & RconL_HomeR)
						{	
							signal_counter++;
							if(signal_counter>3)
							{
								signal_counter=0;
								ActList_Clear();
								Action_SetMove(MOVE_ACT_HEAD2BASELEFT);
								break;								
							}																
						}						
					}
					else//clockwise
					{
						if(Receive_Code&RconR_HomeT)//R_T
						{
							Wheel_SetTargetSpeed(12,22);//ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,22,32);
						}
						else if(Receive_Code&RconR_HomeR)//R_R
						{
							Wheel_SetTargetSpeed(12,18);//ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,22,28);
						}		
						else if(Receive_Code & (RconFR_HomeR|RconFL_HomeR))
						{				
							ActList_Clear();
							ActList_Add(MOVE_ACT_TURN_LEFT,400,TURN_SPEED,TURN_SPEED);
							ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
							ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,0,0);																
							Action_SetMove(MOVE_ACT_HANDLER);
							break;																		
						}	
						else
						{
							Wheel_SetTargetSpeed(22,6);//ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,31,14);30-18
						}	
						
						if(Receive_Code & (RconFL_HomeL|RconFR_HomeL))
						{
							ActList_Clear();
							Action_SetMove(MOVE_ACT_HOMEPATH);
							break;								
						}
						if(Receive_Code & RconR_HomeL)
						{		
							signal_counter++;
							if(signal_counter>5)
							{
								signal_counter=0;
								ActList_Clear();
								Action_SetMove(MOVE_ACT_HEAD2BASERIGHT);
								break;									
							}																								
						}						
					}	

																		
				}																																		
				/*cliff*/																																																														
				if(Cliff_GetDetectiontProcess_Result())
				{
					osDelay(50);
					if(Cliff_GetDetectiontProcess_Result())
					{
						Usprintf("%s(%d):cliff_result:%x\n",__FUNCTION__, __LINE__,Cliff_GetDetectiontProcess_Result());
						Action_SetMove(MOVE_ACT_SEARCHSTATION);
						break;																	
					}
				}																	
																
				/*bumper*/
				if(Bumper_GetTrigStatus())
				{
					path_bumper_counter++;
					ActList_Clear();
					ActList_Add(MOVE_ACT_BACK,100,BACK_SPEED,BACK_SPEED);
					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);																	
					if(dir)//anticlockwise
					{
						ActList_Add(MOVE_ACT_TURN_LEFT,400,TURN_SPEED,TURN_SPEED);
						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);	
						ActList_Add(MOVE_ACT_HOMEAROUND,0,0,0);
					}
					else
					{
						ActList_Add(MOVE_ACT_TURN_RIGHT,400,TURN_SPEED,TURN_SPEED);
						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
						ActList_Add(MOVE_ACT_HOMEAROUND,0,0,0);
					}
//					dir = 1-dir;
//					if(path_bumper_counter>1)
//					{
//						path_bumper_counter=0;
//						ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
//						Action_SetMove(MOVE_ACT_HANDLER);
//					}
//					ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
					Action_SetMove(MOVE_ACT_HANDLER);
					break;
				}	
			}
																	break;
			case MOVE_ACT_HEAD2BASELEFT:
																	Receive_Code = Rcon_GetStatus();
																	Rcon_ResetStatus();
																	if(Receive_Code&(RconFR_HomeL |  RconFR_HomeR))
																	{
																		head2base_no_signal_counter=0;
																		ActList_Clear();
																		Wheel_SetTargetSpeed(10,10);
																		ActList_Add(MOVE_ACT_HOMEPATH,0,0,0);																																		
																	}
																	else
																	{
																		head2base_no_signal_counter++;
																		ActList_Add(MOVE_ACT_HEAD2BASELEFT,100,BASE_SPEED,BASE_SPEED);
																		if(head2base_no_signal_counter >= 200)
																		{
																			head2base_no_signal_counter=0;
																			ActList_Clear();
																			ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
																		}																				
																	}
																	Action_SetMove(MOVE_ACT_HANDLER);
																	break;
			case MOVE_ACT_HEAD2BASERIGHT:				
																	Receive_Code = Rcon_GetStatus();
																	Rcon_ResetStatus();
																	if(Receive_Code&(RconFL_HomeR | RconFL_HomeL))
																	{
																		head2base_no_signal_counter=0;
																		ActList_Clear();
																		Wheel_SetTargetSpeed(10,10);
																		ActList_Add(MOVE_ACT_HOMEPATH,0,0,0);	
																	}
																	else
																	{
																		head2base_no_signal_counter++;
																		ActList_Add(MOVE_ACT_HEAD2BASERIGHT,100,BASE_SPEED,BASE_SPEED);
																		if(head2base_no_signal_counter >= 200)
																		{
																			head2base_no_signal_counter=0;
																			ActList_Clear();
																			ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,MAX_SPEED,MAX_SPEED);
																		}																				
																	}
																	Action_SetMove(MOVE_ACT_HANDLER);
																	break;																		
      case MOVE_ACT_HOMEPATH: 	
																just_homepath_cycle++;
																if(just_homepath_cycle >= 18)//6
																{
																	just_homepath_cycle=0;
																	Receive_Code = Rcon_GetStatus();
																	Rcon_ResetRemoteCode();
																	Rcon_ResetStatus();																																	
																	Adjust_Home_Path(Receive_Code);	
																	if((Receive_Code&(RconFL_HomeR | RconFL_HomeL | RconFR_HomeL |  RconFR_HomeR)) == 0)
																	{
																		path_no_signal_counter++;
																		if(path_no_signal_counter >= 5)//20
																		{
																			path_no_signal_counter=0;
																			ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,BASE_SPEED,BASE_SPEED);
																			Action_SetMove(MOVE_ACT_HANDLER);																		
																		}															
																	}
																	else
																	{
																		path_no_signal_counter=0;
																		if(Receive_Code & (RconFL_HomeR|RconFR_HomeR))//fl_fr_flag
																		{
																			fl_fr_flag = 1;
																		}
																		if(fl_fr_flag==1)
																		{
																			if(Receive_Code & (RconFL_HomeL|RconFL_HomeL))
																			{
																				fl_fr_flag = 2;
																			}
																		}
																		if(fl_fr_flag==2)
																		{
																			fl_fr_counter=0;
																			fl_fr_flag=0;
																		}
																		else
																		{
																			fl_fr_counter++;
																		}
																		if(fl_fr_counter>5)
																		{			
																			if(Receive_Code & Rcon_Signal_LRFLFR_T)
																			{
																				fl_fr_counter=200;
																			}
																			else
																			{
																			fl_fr_counter=0;
																			}
																			if(fl_fr_flag == 1)
																			{
																				dir=0;
																				ActList_Add(MOVE_ACT_TURN_LEFT,500+2*fl_fr_counter,TURN_SPEED,TURN_SPEED);
																			}
																			else
																			{
																				dir=1;
																				ActList_Add(MOVE_ACT_TURN_RIGHT,500+2*fl_fr_counter,TURN_SPEED,TURN_SPEED);
																			}																			
																			ActList_Add(MOVE_ACT_HOMEAROUND,MAX_DISTANCE,0,0);						
																			Action_SetMove(MOVE_ACT_HANDLER);	
																			path_no_signal_counter=0;
																			fl_fr_counter=0;
																			fl_fr_flag=0;																				
																		}
																	}																	
																}																															
																if(Cliff_GetDetectiontProcess_Result())
																{
																	osDelay(50);
																	if(Cliff_GetDetectiontProcess_Result())
																	{
																		Usprintf("%s(%d):cliff_result:%x\n",__FUNCTION__, __LINE__,Cliff_GetDetectiontProcess_Result());
																		Action_SetMove(MOVE_ACT_SEARCHSTATION);
																		break;																	
																	}																	
																}
																if(Bumper_GetTrigStatus())
																{	
																  ActList_Clear();
//																	ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
																	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
																	ActList_Add(MOVE_ACT_TURN_LEFT,100,5,5);
																	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
																	ActList_Add(MOVE_ACT_TURN_RIGHT,200,5,5);
																	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
																	ActList_Add(MOVE_ACT_TURN_LEFT,100,5,5);
																	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);																		
																	ActList_Add(MOVE_ACT_BACK,2000,15,15);
																	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//																	ActList_Add(MOVE_ACT_SEARCHSTATION,MAX_DISTANCE,BASE_SPEED,BASE_SPEED);
																	ActList_Add(MOVE_ACT_HOMEPATH,MAX_DISTANCE,0,0);
																	Action_SetMove(MOVE_ACT_HANDLER);
																	break;
																}
															
																break;	

																	
			case MOVE_ACT_HANDLER:
																	ActList_Switch();
																	break;						
		  default:
																	break;
		}
		
		is_cliff_triggered = Cliff_GetDetectiontProcess_Result();
		if(is_cliff_triggered == CLIFF_ALL_TRIG)
		{
			osDelay(50);
			if(Cliff_GetDetectiontProcess_Result()==CLIFF_ALL_TRIG)
			{
				Usprintf("%s(%d):Pick up! cliff_state:%x\n",__FUNCTION__, __LINE__,is_cliff_triggered);
				Mode_SetMode(MODE_USERINTERFACE);
				break;			
			}
		}
		
		/*abnormal event*/
		if(Motor_GetStatus())
		{
			Usprintf("%s(%d):error!:%d\n",__FUNCTION__, __LINE__,Motor_GetStatus());
			Mode_SetMode(MODE_USERINTERFACE);		
		}			
		vTaskDelayUntil(&xLastWakeTime,20/portTICK_RATE_MS);	
	}
	Wheel_Stop();
}




void Adjust_Home_Path(uint32_t Receive_Code)
{
	static uint32_t temp_code=0;
	static uint32_t NoSignal_Counter=0;
	Usprintf("\n Code =0x: %x ",Receive_Code);
	temp_code = Receive_Code;
	temp_code&=0x000330ff;
	/*if(Action_GetMove()!=MOVE_ACT_FORWARD)return;*/
	if(Receive_Code)
	{
		if(Receive_Code & Rcon_Signal_LRFLFR_T)
		{
			Robot_Near_Cnt++;
			if(Robot_Near_Cnt>0)
			{
				Position_Far=Robot_Home_DistanceNear;
			}
		}
		else
		{
			Robot_Near_Cnt=0;
		}
		Rcon_ResetStatus();
	}
	else
	{
		Robot_Near_Cnt=0;
		NoSignal_Counter++;
	} 
	
	if(Position_Far==Robot_Home_DistanceFar)
	{
		Usprintf(" Robot_Home_DistanceFar =0x: %x",temp_code);
		switch(temp_code)
		{
			case 0x0000003c:Wheel_SetTargetSpeed(14,14);break;//00 11 11 00
			case 0x00000024:Wheel_SetTargetSpeed(13,13);break;//00 10 01 00
			
			case 0x000000ff:Wheel_SetTargetSpeed(12,12);break;//11 11 11 11*
			
			case 0x000000fc:Wheel_SetTargetSpeed(8,15);break;//11 11 11 00*
			case 0x0000003f:Wheel_SetTargetSpeed(15,8);break;//00 11 11 11*
			
			case 0x000000fd:Wheel_SetTargetSpeed(8,15);break;//11 11 11 01*
			case 0x000000bf:Wheel_SetTargetSpeed(15,8);break;//10 11 11 11*
			
			case 0x000000f0:Wheel_SetTargetSpeed(8,15);break;//11 11 00 00*
			case 0x0000000f:Wheel_SetTargetSpeed(15,8);break;//00 00 11 11*
				
			case 0x00000034:Wheel_SetTargetSpeed(10,13);break;//00 11 01 00
			case 0x00000014:Wheel_SetTargetSpeed(10,15);break;//00 01 01 00
			case 0x00000010:Wheel_SetTargetSpeed(5,15);break; //00 01 00 00
			
			case 0x00000074:Wheel_SetTargetSpeed(11,15);break;//01 11 01 00*
			case 0x000000f4:Wheel_SetTargetSpeed(11,15);break;//11 11 01 00*
			case 0x000000d4:Wheel_SetTargetSpeed(10,15);break;//11 01 01 00*
			case 0x000000d0:Wheel_SetTargetSpeed(3,8);break; //11 01 00 00*

			case 0x0000002c:Wheel_SetTargetSpeed(13,10);break;//00 10 11 00
			case 0x00000028:Wheel_SetTargetSpeed(15,10);break;//00 10 10 00
			case 0x00000008:Wheel_SetTargetSpeed(15,5);break; //00 00 10 00
			
			case 0x0000002e:Wheel_SetTargetSpeed(15,11);break;//00 10 11 10*
			case 0x0000002f:Wheel_SetTargetSpeed(15,11);break;//00 10 11 11*
			case 0x0000002b:Wheel_SetTargetSpeed(15,10);break;//00 10 10 11*
			case 0x0000000b:Wheel_SetTargetSpeed(8,3);break; //00 00 10 11*			
			
			case 0x00000001:Wheel_SetTargetSpeed(15,0);break;      //00 00 00 01
			case 0x00000003:Wheel_SetTargetSpeed(15,0);break;
			case 0x00000080:Wheel_SetTargetSpeed(0,15);break;
			case 0x000000c0:Wheel_SetTargetSpeed(0,15);break;
			
			/*----------------------*/  
			case 0x00000009:Wheel_SetTargetSpeed(15,8);break; //00 00 10 01
			case 0x00000090:Wheel_SetTargetSpeed(8,15);break; //10 01 00 00

			case 0x00000094:Wheel_SetTargetSpeed(8,13);break; //10 01 01 00
			case 0x00000029:Wheel_SetTargetSpeed(13,8);break; //00 01 10 01

			case 0x0000000d:Wheel_SetTargetSpeed(14,10);break; //00 00 11 01
			case 0x000000b0:Wheel_SetTargetSpeed(10,14);break; //10 11 00 00

			case 0x0000000A:Wheel_SetTargetSpeed(15,8);break; //00 00 10 10
			case 0x00000050:Wheel_SetTargetSpeed(8,15);break; //01 01 00 00

			case 0x0000002A:Wheel_SetTargetSpeed(15,8);break; //00 10 10 10
			case 0x00000054:Wheel_SetTargetSpeed(8,15);break; //01 01 01 00

			case 0x0000002D:Wheel_SetTargetSpeed(12,9);break; //00 10 11 01
			case 0x000000B4:Wheel_SetTargetSpeed(9,12);break; //10 11 01 00

			case 0x00000041:Wheel_SetTargetSpeed(15,9);break; //01 00 00 01
			case 0x00000082:Wheel_SetTargetSpeed(9,15);break; //10 00 00 10

			case 0x0000000c:Wheel_SetTargetSpeed(12,8);break; //00 00 11 00
			case 0x00000030:Wheel_SetTargetSpeed(8,12);break; //00 11 00 00

			case 0x00000005:Wheel_SetTargetSpeed(15,4);break; //00 00 01 01
			case 0x000000a0:Wheel_SetTargetSpeed(4,15);break; //10 10 00 00

			case 0x00000004:Wheel_SetTargetSpeed(8,15);break; //00 00 01 00*
			case 0x00000020:Wheel_SetTargetSpeed(15,8);break; //00 10 00 00*

			case 0x00000095:Wheel_SetTargetSpeed(11,13);break;//10 01 01 01
			case 0x000000A9:Wheel_SetTargetSpeed(13,11);break;//10 10 10 01

			case 0x000000B5:Wheel_SetTargetSpeed(12,14);break;//10 11 01 01
			case 0x000000AD:Wheel_SetTargetSpeed(14,12);break;//10 10 11 01

			case 0x0000001c:Wheel_SetTargetSpeed(10,8);break; //00 01 11 00
			case 0x00000038:Wheel_SetTargetSpeed(8,10);break; //00 11 10 00
			
			
			case 0x00002000:Wheel_SetTargetSpeed(10,0);break; //00 00 00 00 00 01
			case 0x00010000:Wheel_SetTargetSpeed(0,10);break; //10 00 00 00 00 00
			

			default: Wheel_SetTargetSpeed(12,15);         
							 break;
		 } 
	 }
	 else //near
	 {    			
			temp_code&=0x000000ff;		
			Usprintf(" Robot_Home_DistanceNear =0x: %x ",temp_code);		 
			switch(temp_code)
			{
				case 0x0000003c:Wheel_SetTargetSpeed(10,10);break;// 00 11 11 00
				case 0x00000024:Wheel_SetTargetSpeed(9,9);break;// 00 10 01 00
				
				case 0x000000ff:Wheel_SetTargetSpeed(8,8);break;//11 11 11 11*
				
				case 0x000000fc:Wheel_SetTargetSpeed(3,7);break;//11 11 11 00*
				case 0x0000003f:Wheel_SetTargetSpeed(7,3);break;//00 11 11 11*
				
				case 0x000000fd:Wheel_SetTargetSpeed(5,7);break;//11 11 11 01*
				case 0x000000bf:Wheel_SetTargetSpeed(7,5);break;//10 11 11 11*
				
				case 0x000000f0:Wheel_SetTargetSpeed(3,7);break;//11 11 00 00*
				case 0x0000000f:Wheel_SetTargetSpeed(7,3);break;//00 00 11 11*			
					
				case 0x00000034:Wheel_SetTargetSpeed(5,7);break;// 00 11 01 004-7
				case 0x00000014:Wheel_SetTargetSpeed(2,6);break;// 00 01 01 00
				case 0x00000010:Wheel_SetTargetSpeed(2,6);break;// 00 01 00 00
				
				
				case 0x00000074:Wheel_SetTargetSpeed(4,7);break;//01 11 01 00*
				case 0x000000f4:Wheel_SetTargetSpeed(4,7);break;//11 11 01 00*
				case 0x000000d4:Wheel_SetTargetSpeed(6,8);break;//11 01 01 00*
				case 0x000000d0:Wheel_SetTargetSpeed(3,8);break; //11 01 00 00*

				case 0x0000002c:Wheel_SetTargetSpeed(7,5);break; // 00 10 11 00
				case 0x00000028:Wheel_SetTargetSpeed(6,2);break; // 00 10 10 00
				case 0x00000008:Wheel_SetTargetSpeed(6,2);break;// 00 00 10 00
				
				case 0x0000002e:Wheel_SetTargetSpeed(7,5);break;//00 10 11 10*
				case 0x0000002f:Wheel_SetTargetSpeed(7,4);break;//00 10 11 11*
				case 0x0000002b:Wheel_SetTargetSpeed(8,6);break;//00 10 10 11*
				case 0x0000000b:Wheel_SetTargetSpeed(8,3);break; //00 00 10 11*

				case 0x00000001:Wheel_SetTargetSpeed(12,0);break; // 00 00 00 01#
				case 0x00000080:Wheel_SetTargetSpeed(0,12);break;
				
				
				/*----------------------*/  
				case 0x00000009:Wheel_SetTargetSpeed(10,3);break;// 00 00 10 01
				case 0x00000090:Wheel_SetTargetSpeed(3,10);break;// 10 01 00 00

				case 0x00000094:Wheel_SetTargetSpeed(4,8);break;// 10 01 01 00
				case 0x00000029:Wheel_SetTargetSpeed(8,4);break;// 00 10 10 01

				case 0x0000000d:Wheel_SetTargetSpeed(12,7);break; // 00 00 11 01
				case 0x000000b0:Wheel_SetTargetSpeed(7,12);break; // 10 11 00 00

				case 0x0000000A:Wheel_SetTargetSpeed(12,5);break; // 00 00 10 10
				case 0x00000050:Wheel_SetTargetSpeed(5,12);break; // 01 01 00 00

				case 0x0000002A:Wheel_SetTargetSpeed(8,7);break; // 00 10 10 10
				case 0x00000054:Wheel_SetTargetSpeed(7,8);break; // 01 01 01 00

				case 0x0000002D:Wheel_SetTargetSpeed(9,5);break;  // 00 10 11 01
				case 0x000000B4:Wheel_SetTargetSpeed(5,9);break;  // 10 11 01 00

				case 0x00000041:Wheel_SetTargetSpeed(9,5);break; // 01 00 00 01
				case 0x00000082:Wheel_SetTargetSpeed(5,9);break; // 10 00 00 10

				case 0x0000000c:Wheel_SetTargetSpeed(9,7);break; // 00 00 11 00
				case 0x00000030:Wheel_SetTargetSpeed(6,8);break;// 00 11 00 00

				case 0x00000005:Wheel_SetTargetSpeed(11,6);break;// 00 00 01 01
				case 0x000000a0:Wheel_SetTargetSpeed(6,11);break; // 10 10 00 00

				case 0x00000004:Wheel_SetTargetSpeed(7,9);break; // 00 00 01 00*
				case 0x00000020:Wheel_SetTargetSpeed(8,6);break; // 00 10 00 00*

				case 0x00000095:Wheel_SetTargetSpeed(7,8);break; // 10 01 01 01
				case 0x000000A9:Wheel_SetTargetSpeed(8,7);break; // 10 10 10 01

				case 0x000000B5:Wheel_SetTargetSpeed(7,8);break; // 10 11 01 01
				case 0x000000AD:Wheel_SetTargetSpeed(8,7);break; // 10 10 11 01

				case 0x0000001c:Wheel_SetTargetSpeed(7,8);break;// 00 01 11 00*
				case 0x00000038:Wheel_SetTargetSpeed(8,7);break;// 00 11 10 00*

				default: Wheel_SetTargetSpeed(7,8);
								break;
			} 
	} 
}




