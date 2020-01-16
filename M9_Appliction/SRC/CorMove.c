/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version Ver 0918
  * @date    17-Nov-2018
  * @brief   Main program body
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */
#include "include.h"
#include "math.h"
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




extern xSemaphoreHandle g_binary_wallcalculate_found;
extern xSemaphoreHandle g_binary_wallcalculate_start;
extern volatile uint8_t g_reset_robot_not_moving ;
volatile uint8_t g_shortpath=0,g_reset_all_stuck_cnt_flag = 0,g_reset_wall_stuck_cnt_flag = 0,g_str_lan=0,g_next_t=0,g_from_station=0;
/*
*@ CM_MapTouring() 
*@ perform the navigation function
*@
*@
*@
*/
void CM_MapTouring(void)
{
	CMState_t cm_state = CM_STATE_GYROINIT;
	uint8_t gyro_init_cnt = 0,gyro_error_cnt = 0,path_list_cnt_temp=0;
  uint8_t wall_around_finish=0;//wall_next_lane_flag = 0,

	uint8_t pathlist_state = 0;
	Point16_t temp_pos_cell, temp_robot_cell;
	WallDir_t wall_dir = WALLDIR_NONE;
	PathList_t pathlist_point;
	PathState_t path_status = PATH_STATE_NORMAL_CLEAN;
	
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	ICM42688_Config();
	Gyro_Cmd(DISABLE);	
	osDelay(300);
	osDelay(300);		
	Gyro_Cmd(ENABLE);
	osDelay(300);
	
	Motor_WorkConfigure();
	g_from_station = 0;
	if(Is_ChargerOn())
	{
		g_from_station = 1;
		CM_Move_Back(BACK_SPEED,COR_BACK_400MM);
		Wheel_SetSpeed(0,0);
		Wheel_SetTargetSpeed(0,0);
		osDelay(50);		
		CM_Move_Turn(ROTATE_TOP_SPEED,1800);
		Wheel_SetSpeed(0,0);
		Wheel_SetTargetSpeed(0,0);
		osDelay(100);
	}	
	

  Map_Initialize();
	if(CM_GetCMType() == CM_NORMAL)
	{
		pathlist_point.cell_pos.X = 0;
	}
	else
	{
		pathlist_point.cell_pos.X = Map_GetBoundaryWest()+1;
	}
  
	
  pathlist_point.cell_pos.Y = 0;
	g_reset_robot_not_moving = 1;
	g_reset_all_stuck_cnt_flag = 1;
	g_reset_wall_stuck_cnt_flag = 1;
	Map_SetHomeCell(pathlist_point.cell_pos);
	Map_SetRobotCell(pathlist_point.cell_pos);
	CM_ResetWallOutTrapCnt();
	Wall_AdjustBaseline();
	CM_SetBackToStartPointFlag(0);
	Usprintf("%s(%d):Wall ADC = %d\n",__FUNCTION__, __LINE__,Wall_GetRightAdcValue());
	PathList_Clear();
	TargetList_Clear();
	PathPoint_ClearAllPoints();
	Rcon_ResetStatus();
	if(g_from_station)
	{
		#ifdef WIFI_TY
		AC_SetFlag(0);
		#endif
		CM_SetBackBlock(Map_GetHomeCell());
		#ifdef WIFI_TY
		AC_SetFlag(1);
		#endif		
	}
	while(Mode_GetMode() != MODE_USERINTERFACE)
	{
		switch(cm_state)
		{
			case	CM_STATE_GYROINIT: 
						gyro_init_cnt++;
						if(gyro_init_cnt > 60)
						{
							if(!Gryo_GetUpdateFlag()) //check gyro started or not, other way will need to restart gyro
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
								cm_state = CM_STATE_TOURING;       
								OBS_ResetTrigValue();
								Gyro_Calibration_Cmd(DISABLE);
							}
						}

						break;
			case	CM_STATE_DRYING:
															Wheel_SetSpeed(0,0);
															Wheel_SetTargetSpeed(0,0);						
															break;
			case	CM_STATE_TOURING:    

						pathlist_state = PathList_Out(&pathlist_point);
						Map_AdjustBoundary();
						if(!pathlist_state)//no pathlist out
						{						
							Gyro_Calibration_Cmd(ENABLE);
																					
							if(CM_GetBackToStartPointFlag()==1)
							{
								if(Path_RobotCloseToTargetCell(Map_GetHomeCell(),6))//enlarge the home area from 2 to 5,avoid bumping the wall over and over again
								{									
									CM_Drying();
									Mode_SetMode(MODE_USERINTERFACE);//Mode_SetMode(MODE_HOME);//								
									break;
								}
							}							
							path_status =  Path_Next_V2(1,Map_GetRobotCell());

							
							Gyro_Calibration_Cmd(DISABLE);
							
							if(CM_GetBackToStartPointFlag()==2)//edit by vin
							{		
								Usprintf("%s(%d):CM_GetBackToStartPointFlag()==2 \n",__FUNCTION__, __LINE__);									
								path_status = PATH_STATE_NOTARGET;														
							}
							if(path_status == PATH_STATE_NOTARGET)
							{
								
								if(CM_GetBackToStartPointFlag()==3 || wall_around_finish)
								{
									Usprintf("%s(%d):CM_GetBackToStartPointFlag()==3 \n",__FUNCTION__, __LINE__);	
									Usprintf("wall_around_finish:%d",wall_around_finish);
									CM_Drying();
									Mode_SetMode(MODE_USERINTERFACE);//Mode_SetMode(MODE_HOME);
									break;
								}
								#ifdef WALL_CYCLE								
								Speaker(SPK_WALLFOLLOW_MODE_START);
								CM_SetBackToStartPointFlag(0);
								CM_WallToMap(Map_GetRobotCell(),wall_dir, WALL_AROUND);
								#endif							
								CM_SetBackToStartPointFlag(3);
								Path_BlockAllTargetPoints();
								Path_SetHomeCellEmpty();
								wall_around_finish=1;
								break;
							}
							else if(path_status == PATH_STATE_UNREACHABLE)
							{	
								Usprintf("%s(%d):No Direct Way can access \n",__FUNCTION__, __LINE__);								
								Usprintf("%s(%d):Map_GetCellAcreage=%d \n",__FUNCTION__, __LINE__,Map_GetCellAcreage());
								Usprintf("%s(%d):PathLine_GetAcreage=%d \n",__FUNCTION__, __LINE__,PathLine_GetAcreage());								
								if((PathLine_GetAcreage() * 100 / Map_GetCellAcreage()) > 100)//75 edit by vin
								{
									if(CM_GetBackToStartPointFlag()==1)
									{
										CM_Drying();
										Mode_SetMode(MODE_USERINTERFACE);//Mode_SetMode(MODE_HOME);//
									}
									else
									{
										Path_BlockAllTargetPoints();
										Path_SetHomeCellEmpty();
										CM_SetBackToStartPointFlag(1);
									}
									break;
								}
								else
								{
									//trapped
									//should try to untrap
									Usprintf("%s(%d):robot trapped!\n",__FUNCTION__, __LINE__);	
									Map_FillGap();
									PathList_Clear();
									pathlist_state = 1;
									pathlist_point.status = NORMAL_CLEAN;
									pathlist_point.cell_pos = Map_GetRobotCell();

                  wall_dir = CM_SetTrapWallDir(wall_dir);
									CM_SetWallTrapFlag(1);
									if(CM_WallToMap(Map_GetRobotCell(),wall_dir, WALL_TRAPPED) == MT_ARRIVED)
									{
										
									}
									CM_SetWallTrapFlag(0);
									break;
								}
							}
							else
							{
								Usprintf("%s(%d):Shortest Path Found, go go go! \n",__FUNCTION__, __LINE__);								
								break;
							}
						}
						
						if(pathlist_state)
						{
							//PathList_ShowAll();
							if(CM_GetBackToStartPointFlag()==1)
							{
								pathlist_point.status = SHORT_PATH;
							}

							if(pathlist_point.status == SHORT_PATH)
							{							
								temp_pos_cell = PathList_ReadPath(1).cell_pos;
								temp_robot_cell = Map_GetRobotCell();
								wall_dir = WALLDIR_NONE;
								if((!CM_GetBackToStartPointFlag()))
								{//(PathList_GetCnt() < 3) && 
									if(temp_pos_cell.X !=  temp_robot_cell.X)
									{
										if(Math_Diff_int(temp_pos_cell.Y,temp_robot_cell.Y) < 4)
										{
											wall_dir = Path_GetShortWallDir(temp_pos_cell, temp_robot_cell);
										}
									}
								}
								else
								{
									Usprintf("%s(%d):PathList_GetCnt()= %d\n",__FUNCTION__, __LINE__, PathList_GetCnt());
								}
								
								if(wall_dir != WALLDIR_NONE)
								{
									
									if(CM_WallToMap(Map_GetRobotCell(), wall_dir, WALL_NORMAL) == MT_ARRIVED)
									{
										PathList_Clear();
										CM_WaitForGyroCal();
									}
									break;
								}
								else
								{									
									g_str_lan=0;
									if(CM_MoveToMap(pathlist_point.cell_pos,pathlist_point.status) == MT_OBSTACLE)
									{
//										wall_next_lane_flag = 1;
										PathList_Clear();
										CM_WaitForGyroCal();
										break;
									}
									g_str_lan=0;
//									wall_next_lane_flag = 0;
								}
							}
							else
							{									
								if(pathlist_point.cell_pos.X == Map_GetRobotCellX())// && wall_next_lane_flag//same x means go wallfollow to next lane
								{
									Usprintf("%s(%d):pathlist_point:(%d %d)\n",__FUNCTION__, __LINE__, pathlist_point.cell_pos.X, pathlist_point.cell_pos.Y);
									g_str_lan=0;
																		
//									CM_HeadToTarget(Map_CellToPoint(pathlist_point.cell_pos));
									wall_dir = Path_GetNormalWallDir(pathlist_point.cell_pos);														
									if(Map_GetCell(pathlist_point.cell_pos.X,pathlist_point.cell_pos.Y) == UNCLEAN)
									{
										CM_UpdateStrBlock(pathlist_point.cell_pos,wall_dir);																			
										PathList_Clear();									
										Path_Next_V2(1,pathlist_point.cell_pos);									
									}
									else 
									{										
										PathList_Clear();
										path_status =  Path_Next_V2(0,Map_GetRobotCell());
										PathList_Out(&pathlist_point);
										CM_UpdateStrBlock(pathlist_point.cell_pos,wall_dir);										
									}

									
									if(PathList_GetCnt()>=1)
									{
										path_list_cnt_temp=PathList_GetCnt()-1;
									}else	path_list_cnt_temp=0;
									
									if(CM_MoveToMap(pathlist_point.cell_pos,pathlist_point.status)==MT_OBSTACLE)
									{
										if(PathList_GetCnt()>path_list_cnt_temp)
										{
											CM_WallToMap(Map_GetRobotCell(), wall_dir, WALL_NORMAL);
										}										
										PathList_Clear();										
										CM_WaitForGyroCal();
									}									
									g_str_lan=0;
//									wall_next_lane_flag = 0;
									break;																			
								}
								//straight x lane
//								else
								{									
									if(CM_MoveToMap(pathlist_point.cell_pos,pathlist_point.status)==MT_OBSTACLE)
									{
//										wall_next_lane_flag = 1;
										PathList_Clear();
										CM_WaitForGyroCal();
									}
									else
									{
//										wall_next_lane_flag = 0;
									}								
								}

								break;															
							}
						}
						break;                 
		}
		vTaskDelayUntil(&xLastWakeTime,50/portTICK_RATE_MS);
	}
	Gyro_Cmd(DISABLE);
}

/*
   move to target position by cell
*/
MapTouring_t CM_MoveToMap(Point16_t target_cell,uint8_t move_type)
{
	Point32_t temp;
	temp = Map_CellToPoint(target_cell);
	Usprintf("%s(%d):cell X : %d Y: %d\n",__FUNCTION__, __LINE__, target_cell.X, target_cell.Y);
  Usprintf("%s(%d):count X : %d Y: %d\n",__FUNCTION__, __LINE__, temp.X, temp.Y);
  return CM_MoveToPosition(temp, (WheelDir_t)0, move_type);
}

/*
   move to target position by count
*/
MapTouring_t CM_MoveToPosition(Point32_t target_cnt,WheelDir_t move_dir,uint8_t move_type)
{
	MapTouring_t retval = MT_NONE, arrived_flag = MT_NONE;
	uint16_t gyro_error_timer = 0;
	uint8_t temp_motor_status =0;
  uint8_t bumper_trigger_status = 0,bumper_trigger_buffer = 0;
	uint8_t obs_trigger_buffer = 0;//obs_trigger_status = 0,
	uint8_t cliff_trigger_status = 0,cliff_trigger_buffer = 0;
	int32_t cm_left_wheel_speed = 0,cm_right_wheel_speed = 0;
	int32_t rotate_angle = 0;
//	uint16_t integration_cycle = 0;
//	int32_t integration_value = 0;
//	uint32_t move_speed_base = 20;
	int32_t diff = 0,dis = 0;
	uint8_t robot_slow_flag = 0;
	static uint8_t cliff_trig_cnt = 0;
	uint8_t all_cliff_trigger_flag = 0;
	uint8_t cycle_cnt = 0;//,slip_cnt = 0
//	int16_t angle_diff = 0,gyro_angle_buffer = 0;
	Point32_t temp_target = target_cnt;//target_cnt_buffer = target_cnt,
	uint8_t  turn_u_flag = 0,delay_cnt = 0;//near_target_flag = 0,new_cell = 0,	
	Point16_t cur_cell;//,pre_cell
	
			
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	Usprintf("%s(%d):Target(%d,%d) Heading(%d)\n", __FUNCTION__, __LINE__,target_cnt.X,target_cnt.Y,Path_GetRobotHeading8());
  ActList_Clear();
	CM_StoreCurrentTargetCnt(target_cnt);
	CM_SetNearTargetFlag(0);
	CM_HeadToTarget(target_cnt);
	Action_SetMove(MOVE_ACT_HEAD2COURCE);
	TurnSlip_SetCheckState(ENABLE);
	Wall_SetDynamicState(ENABLE);
	gyro_error_timer = 0;
	Rcon_ResetStatus();	
	while((Mode_GetMode() != MODE_USERINTERFACE))
	{
		cur_cell = Map_GetRobotCell();
//		new_cell = Map_IsReach_NewCell(cur_cell,&pre_cell)? 1: 0;		
		switch(Action_GetMove())
		{
			case MOVE_ACT_FORWARD:
																	Action_WaitForMoveEnded();
																	break;
			case MOVE_ACT_HEAD2COURCE:	
																	cycle_cnt++;
																	if(cycle_cnt > 2)//20ms
																	{
																		cycle_cnt = 0;
																		diff = Math_Diff_int(Heading_GetTargetAngle(), Gyro_GetAngle(0));
																		diff = diff > 1800 ? 3600 - diff : diff;
																		diff = Math_Abs_int(diff);
																		if (diff < 350)//20 
																		{
																			Usprintf("%s(%d):Angle: %d\tGyro: %d\tDiff: %d\n", __FUNCTION__, __LINE__, Heading_GetTargetAngle(), Gyro_GetAngle(0), diff);
																			Usprintf("%s(%d):Gyro Error timer = %d \n",__FUNCTION__, __LINE__,gyro_error_timer);
//																			ActList_DecelerateToStop();//edit by vin
																			ActList_Clear();
																			Action_SetMove(MOVE_ACT_MOVE2POINT);
																			break;
																		}
//																		else if((diff < 400))//100
//																		{
//																			diff /= 20;
//																			if(diff < 6)diff = 6;
//																			Wheel_SetRightTargetSpeed(diff);//edit by vin
//																			Wheel_SetLeftTargetSpeed(diff);//edit by vin
//																		}
																		else
																		{
																			Wheel_SetTargetSpeed(ROTATE_TOP_SPEED,ROTATE_TOP_SPEED);//ROTATE_TOP_SPEED
																		}
										
																		if(TurnSlip_IsSlip()||(Motor_GetStatus()&(CHECK_L_WHEEL|CHECK_R_WHEEL)))//when the edge of robot is stuck by something ,move forward a small distance
																		{	
																			
																			Usprintf("%s(%d):Turn Slip!\n",__FUNCTION__, __LINE__);
																			TurnSlip_ResetSlipCnt();
																			ActList_Clear();
																			ActList_Add(MOVE_ACT_FORWARD,300,MAX_SPEED,MAX_SPEED);																	
																			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
																			ActList_Add(MOVE_ACT_STATIC,0,0,0);
																			Action_SetMove(MOVE_ACT_HANDLER);
																			break;																			
																		}																																				
																		gyro_error_timer ++;
																		if(gyro_error_timer > 1000)//20s
																		{
																			Usprintf("%s(%d):Gyro Error!\n",__FUNCTION__, __LINE__);
																			retval = MT_ARRIVED;
																			Error_SetCode(ERROR_GYRO);
																			Mode_SetMode(MODE_USERINTERFACE);
																			break;
																		}
																  }
																	break;
			case MOVE_ACT_MOVE2POINT: 
																	cycle_cnt++;
																	
																	Rcon_ResetStatus();
																	if(cycle_cnt > 0)//4-25ms
																	{	
																		cycle_cnt = 0;	
																		robot_slow_flag = NULL_DECELERATE;																		
																		rotate_angle = Math_TwoPoint_Angle(Map_GetRobotCount(),target_cnt);
																		turn_u_flag = (Math_Abs_int(rotate_angle) < 150)? FALSE: TRUE ;
																		dis = Math_TwoCell_Dis(Map_PointToCell(target_cnt),Map_GetRobotCell());
																		if(Path_RobotCloseToTargetCount(target_cnt))
																		{																			
																			if(PathList_GetCnt())
																			{																																																													
//																				if((Math_Abs_int(rotate_angle) < 250)||Math_TwoCell_Equal(Map_GetRobotCell(),Map_PointToCell(target_cnt)))
																				if(1)
																				{//(delay_cnt >= 8)||
																					/*if(CM_IsTouring_State(TOURING)&&(PathList_ReadPreviousPath().status == NORMAL_CLEAN))
																					{
																						if((Math_Abs_int(rotate_angle) < 150) && (Math_Diff_Int(Zone_GetFeature(LANE_PATHNEXT),Map_GetRobotCellY()) > 0))
																						{
																							Usprintf("%s(%d):check previous lane\n",__FUNCTION__,__LINE__);
																							if(Path_IsLastLaneUnCleaned(Map_PointToCell(target)))
																							{
																								Usprintf("%s(%d):last lane is unclean,change the target cell\n",__FUNCTION__,__LINE__);
																								Zone_SetFeature(CALCPATH_CONTINUE,TRUE);
																							}		
																						}
																					}*/
																					
																					Map_SetTarget_Cleaned(Map_CountToCell(target_cnt.X),Map_CountToCell(target_cnt.Y));
																					PathList_GetNextPathListPoint(&temp_target);
																			
																					delay_cnt = 0 ;
																					target_cnt = temp_target;
																					
																					/*Heading_Reset_Integral(INTEGRAL_MOVETOPOSITION);*/
																					
																					rotate_angle = Math_Abs_int(Math_TwoPoint_Angle(Map_GetRobotCount(),target_cnt));
																					
																					Usprintf("%s(%d):cur cell:(%d,%d),get next point:(%d,%d),diff angle:%d\n",__FUNCTION__,__LINE__,Map_GetRobotCellX(),Map_GetRobotCellY(),Map_CountToCell(target_cnt.X),Map_CountToCell(target_cnt.Y),rotate_angle);
																					
																					if((Math_Abs_int(rotate_angle) > 1400)&&Math_IsTwoCell_Orthogonal(Map_GetRobotCell(),Map_PointToCell(target_cnt)))//1450Angle wrong direction
																					{
																						Usprintf("%s(%d):rotate angle too large to stop111:%d\n",__FUNCTION__,__LINE__,rotate_angle);
//																						if((Wheel_GetLeftSpeed() > 15)&&(Wheel_GetRightSpeed() > 15))
//																						{
//																							ActList_Clear();
//																							ActList_Add_Forward(280,RUN_TOP_SPEED,RUN_TOP_SPEED); //350
//																							ActList_Add_Action(MOVE_ACT_DECELERATE);//deceleration
//																							ActList_Add_HeadingToTarget(target_cnt);
//																							Action_SetMove(MOVE_ACT_HANDLER);		
//																						}
//																						else
																						{
																							Speaker(26);
																							Action_Stop();
																							ActList_Add_HeadingToTarget(target_cnt);
																							Action_SetMove(MOVE_ACT_HANDLER);		
																						}
																						break;
																					}
//																					else if(((rotate_angle >= 1300)&&Math_IsTwoCell_Orthogonal(Map_GetRobotCell(),Map_PointToCell(target_cnt)))||((rotate_angle >= 400)&&(Math_TwoPoint_Dis(Map_GetRobotCount(),target_cnt) < CELL_COUNT_MUL_3_2)))
//																					{
//																						Speaker(28);
//																						Usprintf("%s(%d):rotate angle is too small222,so forward a distance,angle:%d\n",__FUNCTION__,__LINE__,rotate_angle);
//																						ActList_Clear();
//																						ActList_Add_Forward(280,RUN_TOP_SPEED,RUN_TOP_SPEED); //350
//																						Action_SetMove(MOVE_ACT_HANDLER);
//																						Wheel_SetDir(WHEEL_DIR_FORWARD);
//																						break;
//																					}
																					else
																					{
																						continue;
																					}
																				}
																				else
																				{
																					delay_cnt++;
																				}
																			}
																			else //Last point is null
																			{	
//																				if(move_type == NORMAL_CLEAN)Speaker(28);																		
																				if(Math_Abs_int(rotate_angle) > 500)
																				{
																					/*target_cell = Map_GridToCell(Map_CountToCell(target_cnt.X),Map_CountToCell(target_cnt.Y));*/

																					if(Math_TwoCell_Equal(Map_GetRobotCell(),Map_PointToCell(target_cnt)))
																					{
																						Usprintf("%s(%d):reach last point ,stop it,robot heading:%d,rotate:%d\n",__FUNCTION__, __LINE__,Gyro_GetAngle(0),rotate_angle);
																						Action_Stop();	
																						arrived_flag = MT_ARRIVED;
																						break;
																					}
																				}	
																				else if(Path_IsOnPosition(Map_GetRobotCount(),target_cnt))
																				{
																					Usprintf("%s(%d),target position arrived\n",__FUNCTION__, __LINE__);
																					Action_Stop();	
																					arrived_flag = MT_ARRIVED;
																					break;
																				}
																				else if((Wheel_GetLeftSpeed() > BASE_SPEED) && (Wheel_GetRightSpeed() > BASE_SPEED)&&Math_Abs_int(rotate_angle) < 200)
																				{
																					robot_slow_flag = SLOW_DECELERATE;
																				}
																			}
																		}
																		
																		if(Is_Meet_Station())
																		{
																				Speaker(28);
																				Rcon_ResetStatus();	
																				CM_UpdateMapObs(OBS_FRONT_TRIG);
																				arrived_flag=MT_ARRIVED;
																				ActList_Clear();
																				
																				ActList_Add(MOVE_ACT_STATIC,0,0,0);
																				Action_SetMove(MOVE_ACT_HANDLER);
																				PathList_Clear();
																				break;						
																		}
																		if(Map_ReachBoundary())
																		{
																			Usprintf("%s(%d):Map_ReachBoundary!\n",__FUNCTION__, __LINE__);
																			arrived_flag=MT_ARRIVED;
																			ActList_DecelerateToStop();
																			PathList_Clear();
																			break;
																		}
																		
																		if((turn_u_flag == FALSE) && (dis > 2) && Path_IsCell_ReachBlocked(move_type,cur_cell,Gyro_GetAngle(0)))
																		{
																			robot_slow_flag = SLOW_DECELERATE;
																		}
					
																		if((turn_u_flag == FALSE)&&OBS_IsNear())
																		{
																			robot_slow_flag = SLOW_DECELERATE;//FAST_DECELERATE
																		}
																																			
																		Wheel_Forward_PidProcess(robot_slow_flag,rotate_angle,&cm_left_wheel_speed,&cm_right_wheel_speed);
																		if(robot_slow_flag != NULL_DECELERATE)
																		{
//																			cm_left_wheel_speed  = Math_GetMax(cm_left_wheel_speed,cm_right_wheel_speed);
//																			cm_right_wheel_speed = cm_left_wheel_speed;
																		}
																		Wheel_SetTargetSpeed(cm_left_wheel_speed,cm_right_wheel_speed);	
																		


																		
																		/*if(Map_ReachBoundary())
																		{
																			Usprintf("%s(%d):Map_ReachBoundary!\n",__FUNCTION__, __LINE__);
																			arrived_flag=MT_ARRIVED;
																			ActList_DecelerateToStop();
																			PathList_Clear();
																			break;
																		}
																																			
																		if(PathList_GetCnt() && (g_str_lan==0))//still have another point robot should go
																		{
																			if(Path_RobotCloseToTargetCount(target_cnt))
																			{
																				target_cnt_buffer = target_cnt;
																				near_target_flag = 1;
																				if(!PathList_GetNextPathListPoint(&target_cnt))
																				{
																					arrived_flag=MT_ARRIVED;
																					ActList_DecelerateToStop();
																					Usprintf("%s(%d):No PathPoints %d\n",__FUNCTION__, __LINE__,PathList_GetCnt());
																					break;
																				}
																				if(move_type == SPOT_CLEAN)//check if robot reached spot area limit
																				{
																					if((Math_Abs_int(Map_CountToCell(target_cnt.X)) >= 4) 
																						|| (Math_Abs_int(Map_CountToCell(target_cnt.Y)) >= 4))
																					{
																						Spot_SetSpiralDir(SPOT_DIR_IN);//spiral in
																					}
																				}
																			}
																		}
																		else
																		{
																			if(Math_Abs_int(rotate_angle) > 500)
																			{
																				if(Math_TwoCell_Equal(Map_GetRobotCell(),Map_PointToCell(target_cnt)))
																				{
																					Usprintf("%s(%d):Target position Arrived,robot cell is the same as target!\n",__FUNCTION__, __LINE__);
																					arrived_flag = MT_ARRIVED;
																					ActList_DecelerateToStop();
																					Usprintf("%s(%d):PathList_Cnt=%d\n",__FUNCTION__, __LINE__,PathList_GetCnt());
																					break;																																									
																				}																																								
																			}
																			else if(Path_IsOnPosition(Map_GetRobotCount(),target_cnt))
																			{
																				Usprintf("%s(%d):Target position Arrived!\n",__FUNCTION__, __LINE__);
																				arrived_flag=MT_ARRIVED;
																				ActList_DecelerateToStop();
																				Usprintf("%s(%d):PathList_Cnt=%d\n",__FUNCTION__, __LINE__,PathList_GetCnt());										
																				break;
																			}	
																		}
																		if(near_target_flag)//near the target point ,turn off the pump
																		{
																			if(Path_RobotCloseToTargetCount(target_cnt_buffer))
																		  {																			
																				if(CM_GetNearTargetFlag() == 0)
																				{
																					Usprintf("%s(%d):set near target flag!\n",__FUNCTION__, __LINE__)
																					CM_SetNearTargetFlag(1);
																				}																			  
																			}
																			else
																			{
																				Usprintf("%s(%d):reset near target flag!-\n",__FUNCTION__, __LINE__)
																				CM_SetNearTargetFlag(0);
																			  near_target_flag = 0; 
																			}																		
																		}
																		
																		rotate_angle = Math_Course2Dest(Map_GetRobotCountX(), Map_GetRobotCountY(), target_cnt.X, target_cnt.Y) - Gyro_GetAngle(0);
																		
																		rotate_angle = Math_RoundAngle(rotate_angle);
																																				
																		angle_diff = Math_Diff_int(Gyro_GetAngle(0),gyro_angle_buffer);
																		if(angle_diff > 1800)angle_diff = 3600 - angle_diff;																
																																				
																		if(Math_Diff_int(cm_left_wheel_speed,cm_right_wheel_speed) > 20)
																		{
																			if(angle_diff < 10)
																			{
																				slip_cnt++;
																				if(slip_cnt > 20)
																				{
																					slip_cnt = 0;
																					ActList_Clear();
																					ActList_Add(MOVE_ACT_FORWARD,300,MAX_SPEED,MAX_SPEED);																					
																					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);//deceleration
																					ActList_Add(MOVE_ACT_STATIC,0,0,0);
																					Action_SetMove(MOVE_ACT_HANDLER);
																					break;																					
																				}
																			}	
																			else
																			{
																				slip_cnt = 0;
																			}
																		}
																		else
																		{
																			slip_cnt = 0;
																		}
																		
																		if((Math_Abs_int(rotate_angle) > 900) && (!PathList_GetCnt()))//Angle wrong direction
																		{
																			if(Path_RobotCloseToTargetCount(target_cnt))
																			{																			
																				Usprintf("%s(%d):MoveToPoint Angle Differ %d\n", __FUNCTION__, __LINE__,Math_Abs_int(rotate_angle));
																				Usprintf("RobotPos(%d,%d),TargetPos(%d,%d)\n", Map_GetRobotCountX(),Map_GetRobotCountY(),target_cnt.X, target_cnt.Y);
																				CM_HeadToTarget(target_cnt);
																				Action_SetMove(MOVE_ACT_HEAD2COURCE);
																				TurnSlip_SetCheckState(ENABLE);
																				ActList_Clear();
																				gyro_error_timer = 0;
																				break;
																			}
																		}
																		
																		if(Math_Abs_int(rotate_angle)>1350)//Angle wrong direction
																		{
																			Usprintf("%s(%d):MoveToPoint Angle Differ %d\n", __FUNCTION__, __LINE__,Math_Abs_int(rotate_angle));
																			Usprintf("RobotPos(%d,%d),TargetPos(%d,%d)\n", Map_GetRobotCountX(),Map_GetRobotCountY(),target_cnt.X, target_cnt.Y);
																			CM_HeadToTarget(target_cnt);
																			Action_SetMove(MOVE_ACT_HEAD2COURCE);
																			TurnSlip_SetCheckState(ENABLE);
																			ActList_Clear();
																			gyro_error_timer = 0;
																			break;
																		}
																		
																		
																		rotate_angle = Math_RoundAngle(rotate_angle);																	
																		integration_cycle++;
																		if (integration_cycle > 4)//10 
																		{
																			integration_cycle = 0;
																			integration_value += rotate_angle;
																			if (integration_value > 150) 
																			{
																				integration_value = 150;
																			} 
																			else if (integration_value < -150) 
																			{
																				integration_value = -150;
																			}
																		}
																		robot_slow_flag = 0;
																		if(OBS_GetTrigStatus())robot_slow_flag |=1;
																		if(OBS_IsNear())robot_slow_flag |=1;																																				
																		if(Path_RobotNearPosCount(target_cnt))
																		{
																			robot_slow_flag |=1;
																		}

																		
																		if(robot_slow_flag)
																		{
																			
																			move_speed_base = RUN_SLOW_OBS;
																		}
																		else if(move_speed_base<RUN_TOP_SPEED)
																		{
																			
																			move_speed_base+=1;
																		}					
																		

																		cm_left_wheel_speed  = move_speed_base - rotate_angle / 10 - integration_value / 150; 
																		cm_right_wheel_speed = move_speed_base + rotate_angle / 10 + integration_value / 150; 

																		Wheel_TuneLeftSpeedDir(&cm_left_wheel_speed,RUN_SIDE_BACK_SPEED);
																		Wheel_TuneRightSpeedDir(&cm_right_wheel_speed,RUN_SIDE_BACK_SPEED);
																								
																		cm_left_wheel_speed = Math_LimitingValue(cm_left_wheel_speed, RUN_ADJUST_TOP_SPEED);
																		cm_right_wheel_speed = Math_LimitingValue(cm_right_wheel_speed, RUN_ADJUST_TOP_SPEED);
																		
																		if((Math_Abs_int(rotate_angle)>=100)&& (g_str_lan==0))
																		{
																			move_speed_base=BASE_SPEED;
																			if(cm_left_wheel_speed>(28))cm_left_wheel_speed=28;
																			if(cm_right_wheel_speed>(28))cm_right_wheel_speed=28;
																		}
																		if(g_str_lan==1)
																		{
																			if(Math_Diff_int(cm_left_wheel_speed,cm_right_wheel_speed)>2)
																			{
																				cm_left_wheel_speed = (cm_left_wheel_speed+cm_right_wheel_speed)/2;
																				cm_right_wheel_speed = cm_left_wheel_speed;
																			}
																		}																			
																		Wheel_SetTargetSpeed(cm_left_wheel_speed,cm_right_wheel_speed);	
																		gyro_angle_buffer = Gyro_GetAngle(0);*/
																	}
																	break;
			case MOVE_ACT_STATIC:      {
																	/*picked up events*/
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
																		else
																		{
																			CM_UpdateMapBumper(ACTION_NONE, AllBumperT);
																			arrived_flag = MT_OBSTACLE;																			
																		}																		
																	}
																	/*cliff events happen continuously*/
																	if(Cliff_GetDetectiontProcess_Result() != CLIFF_NO_TRIG)
																	{
																		cliff_trig_cnt++;
																		Usprintf("cliff_trig_cnt:%d\n",cliff_trig_cnt);
																		if(cliff_trig_cnt >= 10)
																		{
																			cliff_trig_cnt = 0;
																		  Mode_SetMode(MODE_USERINTERFACE);
																	   	Error_SetCode(ERROR_CLIFF);
																		  break;																																				
																		}
																	}	
																	else
																	{
																		cliff_trig_cnt = 0;																	  
																	}
																	//on target?																	
																	Usprintf("%s(%d):Arrive_flag =%d!\n",__FUNCTION__, __LINE__,arrived_flag);
																	if(arrived_flag == MT_ARRIVED)
																	{
																		retval = MT_ARRIVED;
																		Usprintf("%s(%d):Arrive_flag == MT_ARRIVED robotposcount(%d,%d)!\n",__FUNCTION__, __LINE__,Map_GetRobotCountX(),Map_GetRobotCountY());
																		break;
																	}
																	if(arrived_flag == MT_OBSTACLE)
																	{
																		retval = MT_OBSTACLE;
																		Usprintf("%s(%d):bumper_trigger_buffer =%d!\n",__FUNCTION__, __LINE__,bumper_trigger_buffer);
																		if (bumper_trigger_buffer)CM_UpdateMapBumper(ACTION_NONE, bumper_trigger_buffer);
																		if(obs_trigger_buffer)CM_UpdateMapObs((ObsTrig_t)obs_trigger_buffer);
																		if(cliff_trigger_buffer)
																		{
																		  if (cliff_trigger_buffer == CLIFF_LEFT_TRIG)CM_UpdateMapBumper(ACTION_NONE, LeftBumperTrig);
																			if (cliff_trigger_buffer == CLIFF_RIGHT_TRIG)CM_UpdateMapBumper(ACTION_NONE, RightBumperTrig);
																			if (cliff_trigger_buffer == CLIFF_FRONT_TRIG)CM_UpdateMapBumper(ACTION_NONE, LeftBumperTrig|RightBumperTrig);																		
																		}
																		break;
																	}
																	ActList_Clear();
																	Action_SetMove(MOVE_ACT_MOVE2POINT);
																	Action_MoveToPoint(target_cnt,RUN_TOP_SPEED);																
																}
																break;
			case MOVE_ACT_DECELERATE:	if((Wheel_GetLeftSpeed() <= 10)&&(Wheel_GetRightSpeed() <= 10))
																{		
																	Usprintf("%s(%d):Deceleration Finished!!\n",__FUNCTION__, __LINE__);
																	ActList_Switch();
																}
																break;
			case MOVE_ACT_BACK:       //Action_WaitForMoveEnded();
																if((Wheel_GetLeftStep() >= Wheel_GetLeftTargetStep()) || (Wheel_GetRightStep() >= Wheel_GetRightTargetStep()))
																{
																	if(Cliff_GetDetectiontProcess_Result() != CLIFF_NO_TRIG)
																	{
																		if(Cliff_GetDetectiontProcess_Result() == CLIFF_ALL_TRIG)
																		{
																			Usprintf("move back ,cliff_all_trig\n");
																		  Error_SetCode(ERROR_PICK_UP);
																			break;
																		}
																		Usprintf("cliff_trig_cnt:%d\n",cliff_trig_cnt);
																		cliff_trig_cnt++;
																		if(cliff_trig_cnt >= 10)
																		{
																			cliff_trig_cnt = 0;
																			Mode_SetMode(MODE_USERINTERFACE);
																			Error_SetCode(ERROR_CLIFF);
																			break;																																			
																		}
																	}
																	else
																	{
																		cliff_trig_cnt = 0;																
																	}	
																	Action_SetMove(MOVE_ACT_HANDLER);																	
																}					
																break;															
			case MOVE_ACT_HANDLER: 		ActList_Switch();
																break;
			default:break;
		}
		/*bumper events or cliff events*/		
		if(Action_GetMove() != MOVE_ACT_BACK)
		{
			bumper_trigger_status = Bumper_GetTrigStatus();
			if (bumper_trigger_status) 
			{			
				bumper_trigger_buffer =  bumper_trigger_status;
				if(Action_GetMove() == MOVE_ACT_HEAD2COURCE)
				{
					ActList_Clear();
					ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
					ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
					ActList_Add(MOVE_ACT_HEAD2COURCE,0,ROTATE_TOP_SPEED,ROTATE_TOP_SPEED);
					Action_SetMove(MOVE_ACT_HANDLER);				
				}
				else
				{
					CM_UpdateMapObs((ObsTrig_t)OBS_GetTrigStatus());
					ActList_BackToStop(150);
//					if(Mode_GetMode() == MODE_SPOT)
					{
						
						CM_UpdateMapBumper(ACTION_NONE, bumper_trigger_buffer);
					}
//					else
//					{
//						CM_UpdateMapObs(OBS_ALL_TRIG);
//						CM_UpdateMapBumper(ACTION_NONE, FrontBumperT);
//					}
					arrived_flag = MT_OBSTACLE;
					Usprintf("%s(%d):Bumper Event!!!\n",__FUNCTION__, __LINE__);				
				}
			}
			if(Path_RobotCloseToTargetCount(target_cnt))
			{
				if((Wheel_GetLeftTargetSpeed()>BASE_SPEED) && (Wheel_GetRightTargetSpeed()>BASE_SPEED))
				{
//					Wheel_SetTargetSpeed(Wheel_GetLeftTargetSpeed()-2,Wheel_GetRightTargetSpeed()-2);
				}			
			}
			if(OBS_IsNear())
			{
				if(Action_GetMove() != MOVE_ACT_HEAD2COURCE)
				{
					if((Wheel_GetLeftTargetSpeed()>BASE_SPEED) && (Wheel_GetRightTargetSpeed()>BASE_SPEED))
					{
						Wheel_SetTargetSpeed(BASE_SPEED,BASE_SPEED);
						if(OBS_SpotStatus())Wheel_SetTargetSpeed(RUN_SLOW_OBS,RUN_SLOW_OBS);
					}				
				}				
			}


//			if (OBS_SpotStatus()) 
//			{								
//				CM_UpdateMapObs((ObsTrig_t)OBS_GetTrigStatus());
//				ActList_BackToStop(200);
//				arrived_flag = MT_OBSTACLE;
//				Usprintf("%s(%d):Bumper Event!!!\n",__FUNCTION__, __LINE__);
//			}			
			cliff_trigger_status = Cliff_GetDetectiontProcess_Result();
			if(cliff_trigger_status)
			{
				Speed_Stop();
//				osDelay(200);
//				cliff_trigger_status = Cliff_GetDetectiontProcess_Result();
				if(cliff_trigger_status)
				{
					if(cliff_trigger_status == CLIFF_ALL_TRIG)
					{
						if(all_cliff_trigger_flag == 0)
						{
							all_cliff_trigger_flag = 1;
							ActList_BackToStop(500);
						}						
					}
					else
					{
						if((Action_GetMove() == MOVE_ACT_MOVE2POINT)||(Action_GetMove() == MOVE_ACT_HEAD2COURCE)||(Action_GetMove() == MOVE_ACT_DECELERATE)||(Action_GetMove() == MOVE_ACT_FORWARD))
						{							
							ActList_BackToStop(600);
							CM_UpdateMapBumper(ACTION_NONE, AllBumperT);
							arrived_flag = MT_OBSTACLE;
							Usprintf("%s(%d):Cliff Event!!! act:%d\n",__FUNCTION__, __LINE__,Action_GetMove());								
						}						
					}					
				}
			}
		}	
		/*remote event*/
		if(Mode_GetMode() == MODE_NAVIGATION || Mode_GetMode() == MODE_NAVIGATION2)		
		{
			if(Rcon_RemoteKey(Remote_Home))
			{
				CM_SetBackToStartPointFlag(1);
				arrived_flag = MT_ARRIVED;
				ActList_DecelerateToStop();
				Rcon_ResetRemoteCode();
				Usprintf("%s(%d):remote home \n",__FUNCTION__, __LINE__);
			}
		}				
		/*abnormal event*/
		temp_motor_status = Motor_GetStatus();
		if(temp_motor_status)
		{
			Motor_ResetStatus();						
			CM_AbnormalHandler(temp_motor_status);
			retval = MT_OBSTACLE;
			if(temp_motor_status == CHECK_MOBILITY)CM_UpdateMapBumper(ACTION_NONE, AllBumperT);						
		}	
		/*error codes*/
		if(Error_GetCode())
		{
			if(Error_GetCode() == ERROR_BATTERY)
			{
					if(Mode_GetMode() == MODE_SPOT)
					{
						Mode_SetMode(MODE_USERINTERFACE);
						break;					  				
					}
					if((Action_GetMove() == MOVE_ACT_MOVE2POINT) && (move_type != SHORT_PATH))
					{
						CM_SetBackToStartPointFlag(1);
						arrived_flag = MT_ARRIVED;
						ActList_DecelerateToStop();
						Usprintf("%s(%d):Water tank!\n",__FUNCTION__, __LINE__);
					}			
			}
			else
			{
				Usprintf("%s(%d):Error_GetCode()\n",__FUNCTION__, __LINE__);
				Mode_SetMode(MODE_USERINTERFACE);
				break;
			}								
		}
		
		/*return*/
		if(retval != MT_NONE)
		{		
			break;
		}
		
		vTaskDelayUntil(&xLastWakeTime,10/portTICK_RATE_MS);
	}
  Usprintf("%s(%d):Jump off CM_MoveToPosition\n\n",__FUNCTION__, __LINE__);
	if(CM_GetBackToStartPointFlag()==1)
	{
		Path_BlockAllTargetPoints();
		Path_SetHomeCellEmpty();
		PathList_Clear();
	}		
	Wall_SetDynamicState(DISABLE);
	return retval;
}

MapTouring_t CM_WallToMap(Point16_t target_cell, WallDir_t wall_dir ,WallTravel_t travel_state)
{
	Point32_t temp;
	temp = Map_CellToPoint(target_cell);
  Usprintf("%s(%d):count X : %d Y: %d\n",__FUNCTION__, __LINE__,temp.X, temp.Y);
  return CM_WallFollowByCnt(temp, wall_dir, travel_state);
}
/* 
  following wall to a lane count
*/
MapTouring_t CM_WallFollowByCnt(Point32_t start_cnt, WallDir_t wall_dir, WallTravel_t travel_state)
{
  static int32_t left_wall_speed = 0, right_wall_speed = 0;
	static int32_t wall_proportion = 0;
	static volatile int32_t wall_delta = 0;
	static volatile int32_t wall_previous = 0;
  MapTouring_t retval = MT_NONE, arrived_flag = MT_NONE;
  static int32_t wall_distance = WALL_DEFAULT_DISTANCE;
  uint8_t bumper_trigger_status = 0;
	uint8_t obs_trigger_status = 0;
  uint16_t robot_dir_buffer = 0;
	uint8_t wall_cross_status = 0;
	uint16_t small_turn=200,wall_str=300;
  Point32_t robot_pos_buffer = Map_GetRobotCount();
	Point16_t temp_robot_cell;
	uint8_t wall_find_way_state = 1;
	uint8_t wall_track_state = 0;
	uint8_t wall_track_cnt = 0;
	CliffTrig_t cliff_buffer = CLIFF_NO_TRIG,cliff_status = CLIFF_NO_TRIG;
	uint8_t obs_near_value = 0;
	uint8_t wall_no_move_cnt = 0;
	uint8_t cliff_trig_cnt = 0;
	Point16_t start_cell,wall_around_start_point=Map_GetRobotCell();
	int32_t robot_angle_integrater = 0,robot_angle_buffer = 0;
	uint8_t wall_obstacle_update_cnt = 0;
	uint8_t wall_bumper_cnt = 0;
	int16_t wall_adc_buffer[4] = {0};
	uint8_t spinning_cnt = 0;
	uint8_t wall_follow_cycle_cnt = 0;
  robot_dir_buffer = Path_GetRobotDirection();
	uint8_t all_cliff_trigger_flag = 0;
	uint8_t spot_start_idx = Spot_GetPointLaneIdx(Map_GetRobotCell());
	uint8_t temp_motor_status = 0;
	uint8_t bumper_cliff_event = 0;
	uint8_t first_turn_flag = 0,wall_around_flag=1,wall_around_result=0;	
	uint8_t cliff_event_flag = 0,cycle_cnt=0;
	static uint32_t l_slip_time = 0,r_slip_time = 0;
	static uint8_t l_slip_stuck_cnt = 0,r_slip_stuck_cnt = 0;
	static uint8_t back_home_trap_cnt = 0;
	uint16_t gyro_error_timer = 0;
	int32_t diff = 0;
	
  portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	if(g_reset_wall_stuck_cnt_flag)
	{
		g_reset_wall_stuck_cnt_flag = 0;
		l_slip_time = 0;
		r_slip_time = 0;
		l_slip_stuck_cnt = 0;
		r_slip_stuck_cnt = 0;
		back_home_trap_cnt = 0;
	}
		
	robot_angle_buffer = Gyro_GetAngle(0);
  
  Usprintf("%s(%d):start_cnt(%d,%d) Heading : %d \n",__FUNCTION__, __LINE__,start_cnt.X,start_cnt.Y,Path_GetRobotHeading8());
	
	if(Cliff_GetDetectiontProcess_Result() != CLIFF_NO_TRIG)
	{
		Usprintf("cliff trigger when static!!\n");
		cliff_trig_cnt++;
	}																		
  start_cell = Map_PointToCell(start_cnt);
  wall_distance = WALL_DEFAULT_DISTANCE;
  Usprintf("%s(%d):Dir = %d \n",__FUNCTION__, __LINE__,wall_dir);
  Map_WallTrackClear();
	Wall_SetDynamicState(DISABLE);
	/*------------actions initialize-----------------*/
  ActList_Clear();
	if((travel_state == WALL_NORMAL) || (travel_state == WALL_SPOT) || (travel_state == WALL_RANDOM))// || (travel_state == WALL_AROUND)
	{
		first_turn_flag = 1;
//		if((wall_dir == WALLDIR_EAST_LEFT)||(wall_dir == WALLDIR_WEST_LEFT))
//		{
//			ActList_Add(MOVE_ACT_TURN_RIGHT,300,TURN_SPEED,TURN_SPEED/2);				
//		}
//		else
//		{
//			ActList_Add(MOVE_ACT_TURN_LEFT,300,TURN_SPEED/2,TURN_SPEED);
//		}
	}
	else if(travel_state == WALL_TRAPPED)
	{
		if(CM_GetBackToStartPointFlag()==1)
		{
		  back_home_trap_cnt++;
			if(back_home_trap_cnt >= 3)
			{
				back_home_trap_cnt = 0;
				Mode_SetMode(MODE_USERINTERFACE);
			}
		}
	}
//	else if(travel_state == WALL_AROUND)
//	{
//		ActList_Add(MOVE_ACT_STRAIGHT,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
//	}
	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
  ActList_Add(MOVE_ACT_STRAIGHT,3*CELL_COUNT_MUL,WALL_WALK_SPEED,WALL_WALK_SPEED);
	ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
	Action_SetMove(MOVE_ACT_HANDLER);
	
	if(g_str_lan)
	{
		ActList_Clear();
		Action_SetMove(MOVE_ACT_HEAD2COURCE);
	}
	Map_WallTrackClear();//edit by vin	
	Rcon_ResetStatus();
	/*------------------------------------------------*/
  while((Mode_GetMode() != MODE_USERINTERFACE))
  {
		switch(Action_GetMove())
		{
			case MOVE_ACT_HEAD2COURCE:
//																if(Bumper_GetTrigStatus())
//																{
//																	ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
//																	Action_SetMove(MOVE_ACT_HANDLER);
//																	break;																	
//																}		
																cycle_cnt++;
																if(cycle_cnt > 2)//50ms
																{
																	cycle_cnt = 0;
																	diff = Math_Diff_int(Heading_GetTargetAngle(), Gyro_GetAngle(0));
																	diff = diff > 1800 ? 3600 - diff : diff;
																	diff = Math_Abs_int(diff);
																	if (diff < 10)//20 
																	{
																		Usprintf("%s(%d):Angle: %d\tGyro: %d\tDiff: %d\n", __FUNCTION__, __LINE__, Heading_GetTargetAngle(), Gyro_GetAngle(0), diff);
																		Usprintf("%s(%d):Gyro Error timer = %d \n",__FUNCTION__, __LINE__,gyro_error_timer);
																		ActList_Clear();
																		ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
																		ActList_Add(MOVE_ACT_STRAIGHT,600,WALL_WALK_SPEED,WALL_WALK_SPEED);
																		ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
																		Action_SetMove(MOVE_ACT_HANDLER);																		
																		break;
																	}
																	else if((diff < 200))//100
																	{
																		diff /= 10;
																		if(diff < 6)diff = 6;
																		Wheel_SetRightTargetSpeed(diff);//diff
																		Wheel_SetLeftTargetSpeed(diff);//diff
																	}
																	if(TurnSlip_IsSlip())//when the edge of robot is stuck by something ,move forward a small distance
																	{	
																		Usprintf("%s(%d):Turn Slip!\n",__FUNCTION__, __LINE__);
																		TurnSlip_ResetSlipCnt();
																		ActList_Clear();
																		ActList_Add(MOVE_ACT_FORWARD,300,MAX_SPEED,MAX_SPEED);																	
																		ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
																		ActList_Add(MOVE_ACT_STATIC,0,0,0);
																		Action_SetMove(MOVE_ACT_HANDLER);
																		break;																			
																	}																																				
																	gyro_error_timer ++;
																	if(gyro_error_timer > 500)//25s
																	{
																		Usprintf("%s(%d):Gyro Error!\n",__FUNCTION__, __LINE__);
																		retval = MT_ARRIVED;
																		Error_SetCode(ERROR_GYRO);
																		Mode_SetMode(MODE_USERINTERFACE);
																		break;
																	}																																
																}
																break;			
			case MOVE_ACT_FORWARD: 		Action_WaitForMoveEnded();							                  
																break;
			case MOVE_ACT_DECELERATE:	if((Wheel_GetLeftSpeed() <= 10) && (Wheel_GetRightSpeed() <= 10))
																{
																	if( arrived_flag ==  MT_ARRIVED)
																	{
																		Usprintf("%s(%d):arrived_flag1 !\n",__FUNCTION__, __LINE__);
																		retval = MT_ARRIVED;
																		break;
																	}
																	ActList_Switch();
																}															
																break;
			case MOVE_ACT_TURN_RIGHT:
																if(Bumper_GetTrigStatus())
																{
//																	ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
//																	ActList_Add(MOVE_ACT_TURN_RIGHT,200,TURN_SPEED,TURN_SPEED);
//																	ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
//																	Action_SetMove(MOVE_ACT_HANDLER);	
//																	break;
																}
																if(Action_WaitForMoveEnded())
																{
																	if(first_turn_flag)
																  {
																	  first_turn_flag = 0;
																		obs_near_value = OBS_IsWallNear(); 
																		if(obs_near_value & OBS_LEFT_TRIG)																	
																		{
//																		  ActList_Clear();
//																			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);	
//																			ActList_Add(MOVE_ACT_TURN_RIGHT,400,TURN_SPEED,TURN_SPEED);
//																			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//																			ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
//																			Action_SetMove(MOVE_ACT_HANDLER);																			
																		}
																	}																		
																}
//																if(TurnSlip_IsSlip())
//																{
//																	TurnSlip_ResetSlipCnt();
//																	if((Time_GetCurrentTime() - r_slip_time) <= 5)
//																	{
//																		r_slip_stuck_cnt++;
//																		if(r_slip_stuck_cnt >= 3)
//																		{
//																			r_slip_stuck_cnt = 0;
//																			Error_SetCode(ERROR_STUCK);
//																		}
//																	}
//																	else
//																	{
//																		if(r_slip_stuck_cnt >= 1)
//																		{
//																			r_slip_stuck_cnt--;
//																		}																						
//																	}
//																	r_slip_time = Time_GetCurrentTime();																		
//																	ActList_WallSlipStraightAndForward(MOVE_ACT_TURN_RIGHT);
//																	Usprintf("%s(%d):Turn Slip\n",__FUNCTION__, __LINE__);
//																}
																break;
			case MOVE_ACT_TURN_LEFT:  
																if(Bumper_GetTrigStatus())
																{
//																	ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
//																	ActList_Add(MOVE_ACT_TURN_LEFT,200,TURN_SPEED,TURN_SPEED);
//																	ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
//																	Action_SetMove(MOVE_ACT_HANDLER);
//																	break;																	
																}
																if(Action_WaitForMoveEnded())
																{
																	if(first_turn_flag)
																  {
																	  first_turn_flag = 0;
																		obs_near_value = OBS_IsWallNear(); 
																		if(obs_near_value & OBS_RIGHT_TRIG)																	
																		{
//																		  ActList_Clear();
//																			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);	
//																			ActList_Add(MOVE_ACT_TURN_LEFT,400,TURN_SPEED,TURN_SPEED);
//																			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//																			ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
//																			Action_SetMove(MOVE_ACT_HANDLER);																			
																		}
																	}																		
																}
//																if(TurnSlip_IsSlip())
//																{
//																	TurnSlip_ResetSlipCnt();
//																	ActList_WallSlipStraightAndForward(MOVE_ACT_TURN_LEFT);
//																	if((Time_GetCurrentTime() - l_slip_time) <= 5)
//																	{
//																		l_slip_stuck_cnt++;
//																		if(l_slip_stuck_cnt >= 3)
//																		{
//																			l_slip_stuck_cnt = 0;
//																			Error_SetCode(ERROR_STUCK);
//																		}
//																	}
//																	else
//																	{
//																		if(l_slip_stuck_cnt >= 1)
//																		{
//																			l_slip_stuck_cnt--;
//																		}																						
//																	}
//																	l_slip_time = Time_GetCurrentTime();																		
//																	Usprintf("%s(%d):Turn Slip\n",__FUNCTION__, __LINE__);
//																}
																break;
			case MOVE_ACT_BACK:       
																if((Wheel_GetLeftStep() >= Wheel_GetLeftTargetStep()) || (Wheel_GetRightStep() >= Wheel_GetRightTargetStep()))
																{
																	if(Cliff_GetDetectiontProcess_Result() != CLIFF_NO_TRIG)
																	{
																		if(Cliff_GetDetectiontProcess_Result() == CLIFF_ALL_TRIG)
																		{
																			Usprintf("move back ,cliff_all_trig\n");
																		  Error_SetCode(ERROR_PICK_UP);
																			break;
																		}																		
																		Usprintf("cliff_trig_cnt:%d\n",cliff_trig_cnt);
																		cliff_trig_cnt++;
																		if(cliff_trig_cnt >= 10)
																		{
																			cliff_trig_cnt = 0;
																			Mode_SetMode(MODE_USERINTERFACE);
																			Error_SetCode(ERROR_CLIFF);
																			break;																																			
																		}
																	}
																	else
																	{
																		cliff_trig_cnt = 0;																
																	}	
																	Action_SetMove(MOVE_ACT_HANDLER);																	
																}																
																break;
			case MOVE_ACT_CURVE:      Action_WaitForMoveEnded();
																break;
			case MOVE_ACT_STRAIGHT:   if(!Action_WaitForMoveEnded())
																{
																	if(Bumper_GetTrigStatus())small_turn=400;
																	else small_turn=200;
																}
																if(1)
																{
																	if(travel_state == WALL_NORMAL)
																	{					
																		wall_cross_status = CM_IsBackCrossing(wall_dir,robot_pos_buffer);
																		if(wall_cross_status)
																		{
																			Usprintf("%s(%d):Have Return to same lane!\n",__FUNCTION__, __LINE__);
																			ActList_WallOffEdgeStop(wall_dir);
																			arrived_flag = MT_HALFARRIVED;					
																		}
																		
																		/*check if go to a new lane*/
																		if(Math_Diff_int(Map_GetRobotCountY(),robot_pos_buffer.Y) > CELL_COUNT_MUL_2)//CELL_COUNT_MUL_2
																		{
																			wall_cross_status = CM_IsCrossLane(wall_dir);
																			if(wall_cross_status == 1)//crossing to new lane
																			{
																				Usprintf("%s(%d):Crossing to target lane!\n",__FUNCTION__, __LINE__);
																				if(robot_dir_buffer == EAST)
																				{
																					Usprintf("hhhhhhh:(%d,%d)\n",Map_GetRobotCellX() + 2,Map_GetRobotCellY());
																					Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() + 2,Map_GetRobotCellY()-1, BLOCKED_BUMPER);
																					Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() + 2,Map_GetRobotCellY(), BLOCKED_BUMPER);
																					Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() + 2,Map_GetRobotCellY()+1, BLOCKED_BUMPER);
																				}
																				else if(robot_dir_buffer == WEST)
																				{
																					Usprintf("aaaaaaa:(%d,%d)\n",Map_GetRobotCellX() - 2,Map_GetRobotCellY());
																					Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() - 2,Map_GetRobotCellY()-1, BLOCKED_BUMPER);
																					Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() - 2,Map_GetRobotCellY(), BLOCKED_BUMPER);
																					Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() - 2,Map_GetRobotCellY()+1, BLOCKED_BUMPER);
																				}
																				ActList_WallOffEdgeStop(wall_dir);
																				arrived_flag = MT_HALFARRIVED;
																				Usprintf("%s(%d):arrived_flag = MT_HALFARRIVED222!!!\n",__FUNCTION__, __LINE__);
																			}
																			else   //moving parrallel to the lane direction
																			{
																				if(Path_OverLane(Map_GetRobotCountY(),robot_pos_buffer.Y))//over cross 1.5 lane
																				{
																					Usprintf("%s(%d):Crossing over 1/2 lane!\n",__FUNCTION__, __LINE__);
																					if(robot_dir_buffer == EAST)
																					{
																						Map_SetCell(Map_GetRobotCellX() + 2,Map_GetRobotCellY(),BLOCKED_BUMPER);
																					}
																					else if(robot_dir_buffer == WEST)
																					{
																						Map_SetCell(Map_GetRobotCellX() - 2,Map_GetRobotCellY(),BLOCKED_BUMPER);
																					}
																					ActList_WallOffEdgeStop(wall_dir);
																					arrived_flag = MT_HALFARRIVED;	
																					Usprintf("%s(%d):arrived_flag = MT_HALFARRIVED333!!!\n",__FUNCTION__, __LINE__);								
																				}
																			}
																		}
																	}																
																
																}

																break;
			case MOVE_ACT_HANDLER: 		ActList_Switch();
																break;
			case MOVE_ACT_STATIC:     
																/*pick up events*/
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
																	else
																	{
																		ActList_Clear();
																		ActList_Add(MOVE_ACT_BACK,400,BACK_SPEED,BACK_SPEED);
																		if((wall_dir == WALLDIR_WEST_LEFT) || (wall_dir == WALLDIR_EAST_LEFT))//left wall follow																		{
																		{	
																			ActList_Add(MOVE_ACT_TURN_RIGHT,900,TURN_SPEED,TURN_SPEED);																			
																		}
																		else //right wall follow
																		{
																			ActList_Add(MOVE_ACT_TURN_LEFT,900,TURN_SPEED,TURN_SPEED);
																		}		
																		ActList_WallStraightAndForward(MOVE_ACT_DECELERATE,400);																		
																	}																		
																}
															  /*cliff events happen continuously*/
																if(cliff_event_flag != 0)
																{
																	if(Cliff_GetDetectiontProcess_Result() != CLIFF_NO_TRIG)
																	{
																		Usprintf("cliff_trig_cnt:%d\n",cliff_trig_cnt);
																		cliff_trig_cnt++;
																		if(cliff_trig_cnt >= 10)
																		{
																			cliff_trig_cnt = 0;
																			Mode_SetMode(MODE_USERINTERFACE);
																			Error_SetCode(ERROR_CLIFF);
																			break;																																			
																		}
																	}
																	else
																	{
																		cliff_trig_cnt = 0;																
																	}
																	if(cliff_event_flag == 1)//left wall follow
																	{
																		ActList_Clear();
																		if(cliff_buffer & CLIFF_RIGHT_TRIG)
																		{
																			ActList_Add(MOVE_ACT_TURN_RIGHT,1350,TURN_SPEED,TURN_SPEED);
																		}
																		else if(cliff_buffer & CLIFF_FRONT_TRIG)
																		{
																			ActList_Add(MOVE_ACT_TURN_RIGHT,900,TURN_SPEED,TURN_SPEED);
																		}
																		else if(cliff_buffer & CLIFF_LEFT_TRIG)
																		{
																			ActList_Add(MOVE_ACT_TURN_RIGHT,450,TURN_SPEED,TURN_SPEED);
																		}
																		ActList_WallStraightAndForward(MOVE_ACT_DECELERATE,400);																																		
																	}
																	else if (cliff_event_flag == 2)//right wall follow
																	{
																		ActList_Clear();
																		if(cliff_buffer & CLIFF_LEFT_TRIG)
																		{
																			ActList_Add(MOVE_ACT_TURN_LEFT,1350,TURN_SPEED,TURN_SPEED);
																		}
																		else if(cliff_buffer & CLIFF_FRONT_TRIG)
																		{
																			ActList_Add(MOVE_ACT_TURN_LEFT,900,TURN_SPEED,TURN_SPEED);
																		}
																		else if(cliff_buffer & CLIFF_RIGHT_TRIG)
																		{
																			ActList_Add(MOVE_ACT_TURN_LEFT,450,TURN_SPEED,TURN_SPEED);
																		}
																		ActList_WallStraightAndForward(MOVE_ACT_DECELERATE,400);																																		
																	}																	
																	cliff_event_flag = 0;
																	break;
																}															
																/*return*/
																if(arrived_flag == MT_HALFARRIVED)
																{
																	Usprintf("%s(%d):arrived_flag == MT_HALFARRIVED break! \n",__FUNCTION__, __LINE__);
																	retval = MT_ARRIVED;
																	break;
																}
																if( arrived_flag )
																{
																	Usprintf("%s(%d):arrived_flag2 !\n",__FUNCTION__, __LINE__);
																	retval = arrived_flag;
																	break;
																}
																if(ActList_GetCnt()!=0)
																{
																	ActList_Switch();
																}
																break;
			default:
																Action_SetMove(MOVE_ACT_STATIC);
																break;
		}

//		if(Wall_GetRightAdcValue()>400)
//		{
//			CM_SetBlockedByOffset(-CELL_SIZE, CELL_SIZE*2);
//		}
//		if(Wall_GetLeftAdcValue()>400)
//		{
//			CM_SetBlockedByOffset(CELL_SIZE, CELL_SIZE*2);
//		}
    if(Action_GetMove() != MOVE_ACT_BACK)
		{
			//station
			if((wall_dir == WALLDIR_WEST_LEFT) || (wall_dir == WALLDIR_EAST_LEFT))
			{
				if(Is_Meet_Station())
				{
					Rcon_ResetStatus();
					ActList_Clear();					
					ActList_Add(MOVE_ACT_TURN_RIGHT,700,TURN_SPEED,0);
					ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
					Action_SetMove(MOVE_ACT_HANDLER);
					wall_distance = Wall_High_Limit;
				}
				else if(Is_Left_Meet_Station())
				{
					Rcon_ResetStatus();
					ActList_Clear();
					ActList_Add(MOVE_ACT_TURN_RIGHT,300,TURN_SPEED,TURN_SPEED/4);
					ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
					Action_SetMove(MOVE_ACT_HANDLER);
					wall_distance = Wall_High_Limit;
				}
			}
			else
			{
				if(Is_Meet_Station())
				{
					Rcon_ResetStatus();
					ActList_Clear();
					ActList_Add(MOVE_ACT_STRAIGHT,900,0,TURN_SPEED);				
					ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
					Action_SetMove(MOVE_ACT_HANDLER);
					wall_distance = Wall_High_Limit;
				}
				else if(Is_Right_Meet_Station())
				{
					Rcon_ResetStatus();
					ActList_Clear();
					ActList_Add(MOVE_ACT_STRAIGHT,300,TURN_SPEED/4,TURN_SPEED);
					ActList_Add(MOVE_ACT_FORWARD,MAX_DISTANCE,WALL_WALK_SPEED,WALL_WALK_SPEED);
					Action_SetMove(MOVE_ACT_HANDLER);
					wall_distance = Wall_High_Limit;
				}			
			}
			/*bumper events*/
      bumper_trigger_status = Bumper_GetTrigStatus();

			if(bumper_trigger_status & LeftBumperTrig)
			{
				l_slip_stuck_cnt = 0;
				r_slip_stuck_cnt = 0;				
				Usprintf("%s(%d):LeftBumperTrig\n",__FUNCTION__, __LINE__);
				if((travel_state == WALL_SPOT) || (travel_state == WALL_RANDOM))CM_UpdateMapBumper(ACTION_NONE, bumper_trigger_status);
        ActList_Clear();

        if((wall_dir == WALLDIR_WEST_LEFT) || (wall_dir == WALLDIR_EAST_LEFT))
        {
					if(OBS_R_F_Near())
					{
						ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
						ActList_Add(MOVE_ACT_TURN_RIGHT,700,TURN_SPEED,TURN_SPEED/2);					
					}
					else ActList_Add(MOVE_ACT_BACK,small_turn,0,BACK_SPEED);
					
					if(wall_distance >= Wall_Low_Limit)wall_distance -= 50;
        }
        else
        {
					ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
          ActList_Add(MOVE_ACT_TURN_LEFT,700,TURN_SPEED/2,TURN_SPEED);
        }
				ActList_WallStraightAndForward(0,wall_str);
			}
			if(bumper_trigger_status & RightBumperTrig)
			{
				l_slip_stuck_cnt = 0;
				r_slip_stuck_cnt = 0;	
				Usprintf("%s(%d):RightBumperTrig Move = %d\n",__FUNCTION__, __LINE__,Wheel_GetLeftStep());
				if((travel_state == WALL_SPOT) || (travel_state == WALL_RANDOM))CM_UpdateMapBumper(ACTION_NONE, bumper_trigger_status);
				ActList_Clear();

        if((wall_dir == WALLDIR_WEST_LEFT) || (wall_dir == WALLDIR_EAST_LEFT))
        {
					ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
          ActList_Add(MOVE_ACT_TURN_RIGHT,700,TURN_SPEED,TURN_SPEED/2);
        }
        else
        {
					if(bumper_trigger_status & LeftBumperTrig)
					{
						ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
						ActList_Add(MOVE_ACT_TURN_LEFT,700,TURN_SPEED/2,TURN_SPEED);
					}
					else
					{
						if(OBS_L_F_Near())
						{
							ActList_Add(MOVE_ACT_BACK,200,BACK_SPEED,BACK_SPEED);
							ActList_Add(MOVE_ACT_TURN_LEFT,700,TURN_SPEED/2,TURN_SPEED);
						}
						else	ActList_Add(MOVE_ACT_BACK,small_turn,BACK_SPEED,0);											
					}
					if(wall_distance >= Wall_Low_Limit)wall_distance -= 50;//edit by vin
					Usprintf("wall_distance:%d\n",wall_distance);
        }
				ActList_WallStraightAndForward(0,wall_str);
			}
			if(bumper_trigger_status)
			{
				if(wall_around_flag)
				{
					wall_around_flag=0;
					wall_around_start_point.X = Map_GetRobotCell().X;
					wall_around_start_point.Y = Map_GetRobotCell().Y;
				}
			}
			/*cliff events*/
			cliff_status = Cliff_GetDetectiontProcess_Result();
			if(cliff_status)
			{
//				osDelay(200);
				cliff_status = Cliff_GetDetectiontProcess_Result();
				if(cliff_status)
				{
					Usprintf("cliff_status:%d\n",cliff_status);
					if(cliff_status == CLIFF_ALL_TRIG)
					{
						if(all_cliff_trigger_flag == 0)
						{
							all_cliff_trigger_flag = 1;
							ActList_BackToStop(500);
						}
					}
					else
					{
						if((Action_GetMove() == MOVE_ACT_FORWARD)||(Action_GetMove() == MOVE_ACT_TURN_RIGHT)||
							(Action_GetMove() == MOVE_ACT_TURN_LEFT)||(Action_GetMove() == MOVE_ACT_STRAIGHT)||(Action_GetMove() == MOVE_ACT_DECELERATE))
						{
							cliff_buffer = cliff_status;
							if((wall_dir == WALLDIR_WEST_LEFT) || (wall_dir == WALLDIR_EAST_LEFT))
							{
								cliff_event_flag = 1;						
							}
							else 
							{
								cliff_event_flag = 2;
							}
							ActList_BackToStop(400);
						}				
					}					
				}			
			}			
			/*stuck ?*/
			if(bumper_trigger_status || cliff_status)
			{
				spinning_cnt = 0;	
				if(bumper_cliff_event == 0)					
				{
					wall_bumper_cnt ++;
				}
				bumper_cliff_event = 1;				
				Usprintf("%s(%d):wall_type:%d act:%d wall_bumper_cnt :%d \n",__FUNCTION__, __LINE__,travel_state,Action_GetMove(),wall_bumper_cnt);
				if(wall_bumper_cnt > 25)//15 79
				{
					ActList_DecelerateToStop();
					if(Stuck_IsStucked())
					{
						Usprintf("%s(%d):stuck! \n",__FUNCTION__, __LINE__);
						retval = MT_ARRIVED;
						Error_SetCode(ERROR_STUCK);
						Mode_SetMode(MODE_USERINTERFACE);
						break;
					}
					else
					{
						arrived_flag = MT_ARRIVED;
						Usprintf("%s(%d):jam! \n",__FUNCTION__, __LINE__);
					}
				}
				if(arrived_flag == MT_HALFARRIVED)
				{
					Usprintf("%s(%d):arrived_flag == MT_HALFARRIVED break111! \n",__FUNCTION__, __LINE__);
					arrived_flag = MT_ARRIVED;
				}
			}
			else
			{
				bumper_cliff_event = 0;
			}
			/*abnormal events*/
			temp_motor_status = Motor_GetStatus();
			if(temp_motor_status)
			{
				Motor_ResetStatus();					
				if(temp_motor_status != CHECK_MOBILITY)				
				{
					CM_AbnormalHandler(temp_motor_status);
					ActList_DecelerateToStop();
					arrived_flag = MT_HALFARRIVED;
					Usprintf("%s(%d):arrived_flag = MT_HALFARRIVED111!!!\n",__FUNCTION__, __LINE__);
				}								
			}
		}
		if((Action_GetMove() == MOVE_ACT_FORWARD))//following a wall
		{
			wall_follow_cycle_cnt++;
			if(wall_follow_cycle_cnt > 4)
			{
				wall_follow_cycle_cnt = 0;
				if((wall_dir == WALLDIR_WEST_RIGHT) || (wall_dir == WALLDIR_EAST_RIGHT)) //right wall follow
				{
					wall_proportion = Wall_GetRightAverageAdcValue();
					wall_adc_buffer[3] = wall_proportion;
					obs_near_value = OBS_IsWallNear();
					//Usprintf("%s(%d):right wall_proportion = %d  wall_distance = %d  \n",__FUNCTION__, __LINE__,wall_proportion,wall_distance);
					/*monitoring if robot near the boundary*/
					if(!CM_WallFollowBoundaryEvent(&wall_proportion, &wall_distance, Map_GetRobotCell()))
					{
						if(obs_near_value & OBS_RIGHT_TRIG)wall_proportion += 100;
						if(obs_near_value & OBS_FRONT_TRIG)wall_proportion += 150;
						if(obs_near_value & OBS_LEFT_TRIG)wall_proportion += 200;
						if(obs_near_value)wall_distance = WALL_DEFAULT_DISTANCE;//edit by vin
					}
					else
					{
						spinning_cnt = 0;
					}
				
					wall_proportion = wall_proportion * 100 / wall_distance;
					wall_proportion -= 100;
					wall_delta = wall_proportion - wall_previous;

					wall_delta /= 3;
					right_wall_speed = RUN_SPEED_12 + wall_proportion / 12 + wall_delta / 5;
					left_wall_speed  = RUN_SPEED_12 - wall_proportion / 10 - wall_delta / 5;
					
					if (left_wall_speed > RUN_SPEED_14) 
					{
						right_wall_speed = RUN_SPEED_4;
						left_wall_speed = RUN_SPEED_14;						
					}
					if(right_wall_speed > RUN_SPEED_14)
					{
						right_wall_speed = RUN_SPEED_14;
					}
					
				}
				else //dir == wall dir left
				{
					wall_proportion = Wall_GetLeftAverageAdcValue();
					wall_adc_buffer[3] = wall_proportion;
					obs_near_value = OBS_IsWallNear();
					//Usprintf("%s(%d):left wall_proportion = %d  wall_distance = %d  \n",__FUNCTION__, __LINE__,wall_proportion,wall_distance);
					/*monitoring if robot near the boundary*/
					if(!CM_WallFollowBoundaryEvent(&wall_proportion, &wall_distance, Map_GetRobotCell()))
					{
						if(obs_near_value & OBS_RIGHT_TRIG)wall_proportion += 200;
						if(obs_near_value & OBS_FRONT_TRIG)wall_proportion += 150;
						if(obs_near_value & OBS_LEFT_TRIG)wall_proportion += 100;
						if(obs_near_value)wall_distance = WALL_DEFAULT_DISTANCE;
					}
					else
					{
						spinning_cnt = 0;
					}				
					wall_proportion = wall_proportion * 100 / wall_distance;
					wall_proportion -= 100;
					wall_delta = wall_proportion - wall_previous;
					wall_delta /= 3; 
					
					left_wall_speed  = RUN_SPEED_12 + wall_proportion / 12 + wall_delta / 5;
					right_wall_speed = RUN_SPEED_12 - wall_proportion / 10 - wall_delta / 5;
					
					if (right_wall_speed > RUN_SPEED_14) 
					{
						left_wall_speed = RUN_SPEED_4;
						right_wall_speed = RUN_SPEED_14;						
					}
					if(left_wall_speed > RUN_SPEED_14)
					{
						left_wall_speed = RUN_SPEED_14;
					}					
				}

				wall_previous = wall_proportion;
				
				right_wall_speed = Math_LimitingValue(right_wall_speed, RUN_SPEED_14);
				left_wall_speed = Math_LimitingValue(left_wall_speed, RUN_SPEED_14);

				Wheel_SetTargetSpeed(left_wall_speed,right_wall_speed);
				
				if((wall_obstacle_update_cnt %3) ==0)
				{
					if(wall_adc_buffer[3] < 20)
					{
						if((wall_adc_buffer[0] - wall_adc_buffer[1]) > (wall_distance/14))
						{
							if((wall_adc_buffer[1] - wall_adc_buffer[2]) > (wall_distance/14))
							{
								if((wall_adc_buffer[2] - wall_adc_buffer[3]) > (wall_distance/14))
								{
									wall_adc_buffer[0] = 100;
									wall_adc_buffer[1] = 100;
									wall_adc_buffer[2] = 100;
									ActList_Clear();
									ActList_WallStraightAndForward(MOVE_ACT_NONE,350);
								}
							}
						}
						//Usprintf("%s(%d):adc = %d %d %d !\n",__FUNCTION__, __LINE__,wall_adc_buffer[0],wall_adc_buffer[1],wall_adc_buffer[2]);
					}
					wall_adc_buffer[0] = wall_adc_buffer[1];
					wall_adc_buffer[1] = wall_adc_buffer[2];
					wall_adc_buffer[2] = wall_adc_buffer[3];
				}
				

				wall_obstacle_update_cnt++;
				if(wall_obstacle_update_cnt > 9)//set wall block every 500 ms 
				{
					wall_obstacle_update_cnt = 0;
					if(travel_state == WALL_SPOT)//do some spot travel wall follow stop calculation
					{												
						robot_angle_buffer -= Gyro_GetAngle(0);
						robot_angle_buffer = Math_RoundAngle(robot_angle_buffer);
						robot_angle_integrater -= robot_angle_buffer;
						Usprintf("%s(%d):intergrater = %d \n",__FUNCTION__, __LINE__,robot_angle_integrater);

						if(robot_angle_integrater > 5400)
						{
							if(Spot_GetSpiralDir() == SPOT_DIR_OUT)
							{
								Usprintf("%s(%d):spot not outside lane!!!\n",__FUNCTION__, __LINE__);
							  Spot_SetSpiralDir(SPOT_DIR_IN);
								ActList_WallOffEdgeStop(wall_dir);
								arrived_flag = MT_HALFARRIVED;
							}
							else
							{
								Usprintf("%s(%d):spot not inner lane!!!\n",__FUNCTION__, __LINE__);
								Mode_SetMode(MODE_USERINTERFACE);
								break;
							}
						}
						robot_angle_buffer = Gyro_GetAngle(0);
						
						if(Spot_GetSpiralDir() == SPOT_DIR_OUT)
						{
							if(Spot_OverLane(start_cnt))
							{
								ActList_WallOffEdgeStop(wall_dir);
								arrived_flag = MT_HALFARRIVED;
								Usprintf("%s(%d):Spot wall ended !\n",__FUNCTION__, __LINE__);
							}												
						}
						else
						{
							if(Spot_OverLane(start_cnt))
							{
								ActList_WallOffEdgeStop(wall_dir);
								arrived_flag = MT_HALFARRIVED;
								Usprintf("%s(%d):Spot wall ended !\n",__FUNCTION__, __LINE__);
							}									
//							if(Spot_InnerLane(spot_start_idx))
//							{
//								ActList_WallOffEdgeStop(wall_dir);
//								arrived_flag = MT_HALFARRIVED;
//								Usprintf("%s(%d):Spot wall ended inner lane!\n",__FUNCTION__, __LINE__);							
//							}												
						}						
						CM_UpdateSpotWallBlock((WallDir_t)1);
					}
					else
					{
						/*
							intergrated robot heading
							normally robot following a wall should not have a heading intergrated greater than 360 degree
							only in two condiction robot will have a over 360 degree heading
							1.robot trapped by obstcle , then robot wall follow over and over and over 
							2.robot wall follow error by some movable obstacle to and open area and following nothing and far away from 
								obstacles
						*/
						robot_angle_buffer -= Gyro_GetAngle(0);
						robot_angle_buffer = Math_RoundAngle(robot_angle_buffer);
						robot_angle_integrater -= robot_angle_buffer;

						if(robot_angle_integrater > 5400)
						{
							Usprintf("%s(%d):robot trapped in wall follow mode \n",__FUNCTION__, __LINE__);
//							Mode_SetMode(MODE_USERINTERFACE);//edit by vin
//							break;
						}
						robot_angle_buffer = Gyro_GetAngle(0);
						
						temp_robot_cell = Map_GetRobotCell();
						CM_UpdateMapWallBlock(temp_robot_cell,wall_dir);//edit by vin				
						wall_track_state =  Map_WallTrackUpdate(temp_robot_cell);
						if(travel_state == WALL_AROUND)
						{
							if(wall_track_state == 2)
							{
								wall_track_state=1;
								Map_WallTrackClear();//edit by vin
							}
						}
						if(wall_track_state == 1)//reached a new wall point
						{
							wall_no_move_cnt = 0;
							wall_bumper_cnt = 0;
							//Stuck_ResetCnt();
							if(((travel_state == WALL_TRAPPED) && wall_find_way_state)||(travel_state == WALL_AROUND))//in trapped state and enable wall find way
							{
								if(travel_state == WALL_TRAPPED)
								{
									xSemaphoreGive(g_binary_wallcalculate_start);//start the calculate path task to find shortest path 
								}
								
								if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == NORTH)wall_dir = WALLDIR_EAST_RIGHT;
								else if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == SOUTH)wall_dir = WALLDIR_WEST_RIGHT;

								wall_track_cnt ++;
								if(wall_track_cnt > 10)
								{
									if(Map_TwoCellMatch(temp_robot_cell,start_cell))
									{
										Usprintf("%s(%d):round a circle\n",__FUNCTION__, __LINE__);
										arrived_flag = MT_ARRIVED;
										ActList_DecelerateToStop();
										CM_SetBackToStartPointFlag(1);
									}
								}
								if(Wall_GetRightAverageAdcValue() < 100)
								{
//									spinning_cnt++;//edit by vin
//									Usprintf("%s(%d):spinning cnt:%d\n",__FUNCTION__, __LINE__,spinning_cnt);
//									if(spinning_cnt > 11)//about a circle
//									{
//										Usprintf("%s(%d):round a circle in the air\n",__FUNCTION__, __LINE__);
//										spinning_cnt = 0;								
//										ActList_Clear();
//										ActList_WallStraightAndForward(MOVE_ACT_DECELERATE,10000);								
//									}
								}	
								else
								{
									spinning_cnt = 0;
								}								
							}
						}
						else  if(wall_track_state == 2)//wall tack too many .... 
						{
							Usprintf("%s(%d):should stop wall and return to start point!\n",__FUNCTION__, __LINE__);							
							arrived_flag = MT_ARRIVED;
							ActList_DecelerateToStop();
							CM_SetBackToStartPointFlag(1);
							if(travel_state == WALL_TRAPPED)
							{
								CM_SetBackToStartPointFlag(2);
							}							
						}
						else  if(wall_track_state == 3)//wall tack too many .... 
						{
							wall_no_move_cnt++;
							Usprintf("%s(%d):robot circling %d \n",__FUNCTION__, __LINE__,wall_no_move_cnt);
							if(robot_angle_integrater < -5400)
							{
								wall_no_move_cnt = 0;
								Usprintf("%s(%d):robot circling ...\n",__FUNCTION__, __LINE__);
								ActList_Clear();
								ActList_WallStraightAndForward(MOVE_ACT_DECELERATE,10000);							
							}
						}
					}
				}			
				if(travel_state == WALL_NORMAL)
				{					
					wall_cross_status = CM_IsBackCrossing(wall_dir,robot_pos_buffer);
					if(wall_cross_status)
					{
						Usprintf("%s(%d):Have Return to same lane!\n",__FUNCTION__, __LINE__);
						ActList_WallOffEdgeStop(wall_dir);
						arrived_flag = MT_HALFARRIVED;					
					}
					
					/*check if go to a new lane*/
					if(Math_Diff_int(Map_GetRobotCountY(),robot_pos_buffer.Y) > CELL_COUNT_MUL_2)//CELL_COUNT_MUL_2
					{
						wall_cross_status = CM_IsCrossLane(wall_dir);
						if(wall_cross_status == 1)//crossing to new lane
						{
							Usprintf("%s(%d):Crossing to target lane!\n",__FUNCTION__, __LINE__);
							if(robot_dir_buffer == EAST)
							{
								Usprintf("hhhhhhh:(%d,%d)\n",Map_GetRobotCellX() + 2,Map_GetRobotCellY());
								Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() + 2,Map_GetRobotCellY()-1, BLOCKED_BUMPER);
								Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() + 2,Map_GetRobotCellY(), BLOCKED_BUMPER);
								Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() + 2,Map_GetRobotCellY()+1, BLOCKED_BUMPER);
							}
							else if(robot_dir_buffer == WEST)
							{
								Usprintf("aaaaaaa:(%d,%d)\n",Map_GetRobotCellX() - 2,Map_GetRobotCellY());
								Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() - 2,Map_GetRobotCellY()-1, BLOCKED_BUMPER);
								Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() - 2,Map_GetRobotCellY(), BLOCKED_BUMPER);
								Map_SetBlockIfUnclean(MAP,Map_GetRobotCellX() - 2,Map_GetRobotCellY()+1, BLOCKED_BUMPER);
							}
							ActList_WallOffEdgeStop(wall_dir);
							arrived_flag = MT_HALFARRIVED;
							Usprintf("%s(%d):arrived_flag = MT_HALFARRIVED222!!!\n",__FUNCTION__, __LINE__);
						}
						else   //moving parrallel to the lane direction
						{
							if(Path_OverLane(Map_GetRobotCountY(),robot_pos_buffer.Y))//over cross 1.5 lane
							{
								Usprintf("%s(%d):Crossing over 1/2 lane!\n",__FUNCTION__, __LINE__);
								if(robot_dir_buffer == EAST)
								{
									Map_SetCell(Map_GetRobotCellX() + 2,Map_GetRobotCellY(),BLOCKED_BUMPER);
								}
								else if(robot_dir_buffer == WEST)
								{
									Map_SetCell(Map_GetRobotCellX() - 2,Map_GetRobotCellY(),BLOCKED_BUMPER);
								}
								ActList_WallOffEdgeStop(wall_dir);
								arrived_flag = MT_HALFARRIVED;	
								Usprintf("%s(%d):arrived_flag = MT_HALFARRIVED333!!!\n",__FUNCTION__, __LINE__);								
							}
						}
					}
				}
				else if(travel_state == WALL_TRAPPED)
				{
					/* if path found , exit wall follow */
					if(xSemaphoreTake(g_binary_wallcalculate_found,0) == pdPASS)
					{
						wall_find_way_state = 0;
						ActList_DecelerateToStop();
						Usprintf("%s(%d):short way found1111111111111 !\n",__FUNCTION__, __LINE__);
						arrived_flag = MT_ARRIVED;
					}
				}
				if(travel_state == WALL_AROUND || travel_state == WALL_TRAPPED)
				{
					if(Path_RobotCloseToTargetCell(wall_around_start_point,6))
					{
						if(wall_around_result ==1)
						{
							wall_around_result =2;
						}
					}
					else if(Path_RobotLeaveToTargetCell(wall_around_start_point,12))
					{
						wall_around_result =1;
					}
					if(wall_around_result==2)
					{
						ActList_DecelerateToStop();
						Usprintf("%s(%d):short way found22222222222222 !\n",__FUNCTION__, __LINE__);
						arrived_flag = MT_ARRIVED;							
					}
				}	
			}
		}
		/*obs events or boundary*/
		if((Action_GetMove() == MOVE_ACT_FORWARD) || (Action_GetMove() == MOVE_ACT_STRAIGHT))
		{
			obs_trigger_status = OBS_GetTrigStatus();
			if(obs_trigger_status)
			{
				l_slip_stuck_cnt = 0;
				r_slip_stuck_cnt = 0;
				if((wall_dir == WALLDIR_WEST_LEFT) || (wall_dir == WALLDIR_EAST_LEFT))
				{
					if(obs_trigger_status & (OBS_FRONT_TRIG|OBS_RIGHT_TRIG))
					{
						ActList_Clear();
						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
						ActList_Add(MOVE_ACT_TURN_RIGHT,700,TURN_SPEED,TURN_SPEED);
						ActList_WallStraightAndForward(MOVE_ACT_DECELERATE,200);
						Usprintf("%s(%d):Obs Event!!!\n",__FUNCTION__, __LINE__);
					}
				}
				else
				{
					if(obs_trigger_status & (OBS_FRONT_TRIG|OBS_LEFT_TRIG))
					{
						ActList_Clear();
						ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
						ActList_Add(MOVE_ACT_TURN_LEFT,700,TURN_SPEED,TURN_SPEED);
						ActList_WallStraightAndForward(MOVE_ACT_DECELERATE,200);
						Usprintf("%s(%d):Obs Event!!!\n",__FUNCTION__, __LINE__);
					}
				}
				CM_UpdateMapObs(OBS_ALL_TRIG);
//				OBS_OverLimitCheck(obs_trigger_status);//edit by vin
				wall_distance = WALL_DEFAULT_DISTANCE;
//				if(travel_state != WALL_AROUND)
				{
					if(arrived_flag == MT_HALFARRIVED)
					{
						Usprintf("%s(%d):arrived_flag == MT_HALFARRIVED break222! \n",__FUNCTION__, __LINE__);
						arrived_flag = MT_ARRIVED;
					}
				}				
			}		
			if(Map_ReachBoundary())
			{
				arrived_flag = MT_ARRIVED;
				ActList_DecelerateToStop();
			}	
		}
		/*remote events*/
		if(Mode_GetMode() == MODE_NAVIGATION || Mode_GetMode() == MODE_NAVIGATION2)
		{
			if(Rcon_RemoteKey(Remote_Home))
			{
				CM_SetBackToStartPointFlag(1);
				arrived_flag = MT_ARRIVED;
				ActList_DecelerateToStop();
				Rcon_ResetRemoteCode();
				Usprintf("%s(%d):remote home\n",__FUNCTION__, __LINE__);
			}				
		}
		/*error codes*/
		if(Error_GetCode())
		{
			if(Error_GetCode() == ERROR_BATTERY)
			{
				if(Mode_GetMode() == MODE_SPOT)
				{
					Mode_SetMode(MODE_USERINTERFACE);
					break;					  				
				}
				if((Action_GetMove() == MOVE_ACT_FORWARD) || (Action_GetMove() == MOVE_ACT_STRAIGHT))
				{
					if(!CM_GetBackToStartPointFlag())
					{
						CM_SetBackToStartPointFlag(1);
						arrived_flag = MT_ARRIVED;
						ActList_DecelerateToStop();
						Usprintf("%s(%d):Water tank or No power!\n",__FUNCTION__, __LINE__);					
					}
				}
			}
			else
			{
				Usprintf("%s(%d):Error_GetCode()\n",__FUNCTION__, __LINE__);
				Mode_SetMode(MODE_USERINTERFACE);
				break;
			}
		}
		
		if(retval!=MT_NONE)
		{
			Usprintf("%s(%d):retval:%d \n\n",__FUNCTION__, __LINE__,retval);
			break;
		}		
		if(Action_GetMove() == MOVE_ACT_STRAIGHT)
		{
			/*check if the three cells in front of the robot when moving,if one of them is set as "boundary",stop to find short path or wall follow again*/
		  if(CM_CheckMeetBoundary(Map_GetRobotCell()))
			{
				 arrived_flag = MT_ARRIVED;
				 ActList_DecelerateToStop();				 
			}	
		}
				
		vTaskDelayUntil(&xLastWakeTime,10/portTICK_RATE_MS);
  }
	
	Wheel_SetSpeed(0,0);
  Map_WallTrackShowAll();
	Map_WallTrackSetObs(wall_dir);
  Usprintf("%s(%d):JumOFF_Trapped \n\n",__FUNCTION__, __LINE__);
	if(CM_GetBackToStartPointFlag()==1)
	{
		Path_BlockAllTargetPoints();
		Path_SetHomeCellEmpty();
		PathList_Clear();
	}		
  return retval;
}
/*
*@heading 		offset heading of the map 
*@offset_lat 	left or right side of the heading direction offset
*@offset_long	front or back side of the heading direction offset
*@*x 					offset result x (cell)
*@*y 					offset result y (cell)
*/
void CM_CellOffset(uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y)
{
	*x = Map_GetRelativeX(heading, offset_lat, offset_long);
	*y = Map_GetRelativeY(heading, offset_lat, offset_long);
}

void CM_SetBlockedByOffset(int16_t offset_lat, int16_t offset_long)
{
	Point16_t offset;
	offset = Map_GetRelativeByCell(Map_GetRobotCell(), Gyro_GetAngle(0),offset_lat,offset_long);
	Map_SetCell(offset.X,offset.Y, BLOCKED_BUMPER);
//	Usprintf("robot cell:(%d ,%d)  block cell:(%d ,%d) angle:%d\n",Map_GetRobotCell().X,Map_GetRobotCell().Y,offset.X,offset.Y,Gyro_GetAngle(0));
}

void MY_SetBlockedByOffset(Point16_t point, int16_t offset_lat, int16_t offset_long)
{
	Point16_t offset;
	offset = Map_GetRelativeByCell(point, Gyro_GetAngle(0),offset_lat,offset_long);
//	Usprintf("%s(%d):block point&&&&&&&&:(%d, %d)\n", __FUNCTION__, __LINE__,offset.X,offset.Y);
	Map_SetCell(offset.X,offset.Y, BLOCKED_BUMPER);
	
}
//static inline void CM_CellOffsetWithRad(uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y,double rad)
//{
//	Map_GetRelativeXY(heading, offset_lat, offset_long,x,y,rad);
//}

void CM_UpdatePosition(uint16_t heading_0, int16_t heading_1, int16_t left, int16_t right) 
{
	double	dd, temp_rad;
	int16_t path_heading;
	int32_t x_offset, y_offset;
	int8_t i = 0;

	if (left == 0 && right == 0) //no move
	{
		return;
	}

//	if((Mode_GetMode() == MODE_WALL) || (Mode_GetMode() == MODE_REMOTE))
//	{
//		return;	
//	}
	
	if ((heading_0 > heading_1) && ((heading_0 - heading_1) > 1800)) 
	{
		path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
	} 
	else if ((heading_1 > heading_0) && ((heading_1 - heading_0) > 1800)) 
	{
		path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
	} 
	else 
	{
		path_heading = (uint16_t)(heading_0 + heading_1) >> 1;
	}

	Wheel_SetCount(Wheel_GetLeftCount() - left,Wheel_GetRightCount() - right);

	dd = left + right;
	dd /= 2;

  temp_rad = Math_Deg2Rad(path_heading, 10);
	Map_MoveRobotCountTo(dd * cos(temp_rad), dd * sin(temp_rad));//calculate current position by heading and move distance
	
	for (i = 1; i >= -1; --i) //set 3 cells infront of robot position to cleaned
  {
		//Map_GetRelativeXY(heading_0, i * CELL_SIZE, CELL_SIZE,&x_offset,&y_offset,temp_rad);
		Map_GetRelativeXY(heading_0, i * CELL_SIZE, 0,&x_offset,&y_offset,temp_rad);
    Map_SetCell(x_offset, y_offset, CLEANED);
		#ifdef WIFI_TY
		if(i==0)
		{								
			if(Map_GetMapArray(x_offset,y_offset) != CLEANED)
			{
				if(Get_ACFlag())
				{	
					if((Path_GetRobotHeading4(Gyro_GetAngle(0))==EAST) || (Path_GetRobotHeading4(Gyro_GetAngle(0))==WEST))
					{
						mPC_NavDebug(x_offset,y_offset,0,CLEANED);
						AC_RealMap_AddPoint(x_offset,y_offset,CLEANED);					
					}							
				}			
			}
		}
		#endif
  }
}

void CM_UpdateSpotWallBlock(WallDir_t dir)
{
	int32_t	x_tmp, y_tmp;

	CM_CellOffset(Gyro_GetAngle(0), CELL_SIZE * -2, 0, &x_tmp, &y_tmp);
	Map_SetCell(x_tmp, y_tmp, BLOCKED_BUMPER);
	
}


void CM_UpdateMapWallBlock(Point16_t robot_cell, WallDir_t dir)
{
  int32_t temp_x, temp_y;
//	int8_t side = 0, heading = 1 , offset = 0;
	uint8_t i=0;
//	int32_t robot_y = robot_cell.Y;
	int32_t temp_angle = Gyro_GetAngle(0);

	switch(dir)
	{
		case	WALLDIR_EAST_LEFT: 	for(i=0;i<2;i++)
															{
																CM_CellOffset(temp_angle,2*CELL_SIZE, i*CELL_SIZE, &temp_x, &temp_y);//get robot x,y off set to i , j by c 
																if(!Map_SetBlockIfUnclean(MAP, temp_x, temp_y, BLOCKED_BUMPER))
																{
																	Map_SetCell(temp_x + 1, temp_y, BLOCKED_BUMPER);
																}                          
															}
															break;
		case	WALLDIR_EAST_RIGHT: for(i=0;i<2;i++)
															{
																CM_CellOffset(temp_angle,-2*CELL_SIZE, i*CELL_SIZE, &temp_x, &temp_y);//get robot x,y off set to i , j by c 
																if(!Map_SetBlockIfUnclean(MAP, temp_x, temp_y, BLOCKED_BUMPER))
																{
																	Map_SetCell(temp_x +1, temp_y, BLOCKED_BUMPER);
																}
															}
															break;
		case	WALLDIR_WEST_LEFT:  for(i=0;i<2;i++)
															{
																CM_CellOffset(temp_angle,2*CELL_SIZE, i*CELL_SIZE, &temp_x, &temp_y);//get robot x,y off set to i , j by c 
																if(!Map_SetBlockIfUnclean(MAP, temp_x, temp_y, BLOCKED_BUMPER))
																{
																	Map_SetCell(temp_x - 1, temp_y, BLOCKED_BUMPER);
																}
															}
															break;
		case	WALLDIR_WEST_RIGHT: for(i=0;i<2;i++)
															{
																CM_CellOffset(temp_angle,-2*CELL_SIZE, i*CELL_SIZE, &temp_x, &temp_y);//get robot x,y off set to i , j by c 
																if(!Map_SetBlockIfUnclean(MAP, temp_x, temp_y, BLOCKED_BUMPER))
																{
																	Map_SetCell(temp_x -1, temp_y, BLOCKED_BUMPER);
																}
															}
															break;
		default:break;
	}

}

void CM_UpdateStrBlock(Point16_t robot_cell, WallDir_t dir)
{
	int8_t i=0;	
	Usprintf("%s(%d):now point&&&&&&&&:(%d, %d)\n", __FUNCTION__, __LINE__,robot_cell.X,robot_cell.Y);
	switch(dir)
	{
		case	WALLDIR_WEST_RIGHT:													
		case	WALLDIR_EAST_RIGHT: 		
		case	WALLDIR_WEST_LEFT:
		case	WALLDIR_EAST_LEFT: 	for(i=-1;i<2;i++)
															{
																MY_SetBlockedByOffset(robot_cell,CELL_SIZE*i, CELL_SIZE*2);																																
															}
															break;

		default:break;
	}

}

void CM_SetBackBlock(Point16_t robot_cell)
{
	int8_t i=0,j=0;	
	Point16_t point;
	point.X = 5;
	point.Y = 0;
 	for(i=-4;i<4;i++)
	{
		for(j=-4;j<4;j++)
		{
			MY_SetBlockedByOffset(point,CELL_SIZE*j, -CELL_SIZE*i);
		}																																		
	}
}

void CM_UpdateMapBumper(Action_t action, uint8_t bumper)
{
	int16_t	c;


//	Usprintf("%s(%d):Robot Head Pos (%d, %d)\n", __FUNCTION__, __LINE__, (x_tmp), (y_tmp));
	Usprintf("%s(%d):Bumper State = %d \n", __FUNCTION__, __LINE__,bumper);
	
	if((bumper & RightBumperTrig) && (bumper & LeftBumperTrig)) 
	{
		for (c = -1; c <= 1; ++c) 
		{
			CM_SetBlockedByOffset(CELL_SIZE*c, CELL_SIZE*2);
		}
	} 
	else if (bumper & LeftBumperTrig) 
	{
		CM_SetBlockedByOffset(CELL_SIZE, CELL_SIZE*2);
		CM_SetBlockedByOffset(0, CELL_SIZE*2);
	} 
	else if (bumper & RightBumperTrig) 
	{
		CM_SetBlockedByOffset(0, CELL_SIZE*2);
		CM_SetBlockedByOffset(-CELL_SIZE, CELL_SIZE*2);
	}
}

void CM_UpdateMapObs(ObsTrig_t obs)
{	
	int8_t	c;	
	
	if(obs & OBS_FRONT_TRIG)
	{
		for (c = -1; c <= 1; ++c)//edit by vin 
		{
			CM_SetBlockedByOffset(CELL_SIZE*c, CELL_SIZE*2);
		}	
	}
	
	if(obs & OBS_LEFT_TRIG) 
	{
		CM_SetBlockedByOffset(CELL_SIZE, CELL_SIZE*2);
	} 
	if (obs & OBS_RIGHT_TRIG) 
	{
		CM_SetBlockedByOffset(-CELL_SIZE, CELL_SIZE*2);
	}
}

/*--------------------------Head Angle--------------------------------*/ 
void ActList_Add_HeadingToTarget(Point32_t target)
{
	CM_StoreCurrentTargetCnt(target);
	ActList_Add(MOVE_ACT_HEAD2COURCE, 0, ROTATE_TOP_SPEED, ROTATE_TOP_SPEED);
}

void CM_HeadToTarget(Point32_t target)
{
	int32_t target_course;
	Usprintf("%s(%d)robot:(%d,%d),target:(%d,%d)\n",__FUNCTION__, __LINE__,
	Map_GetRobotCellX(),Map_GetRobotCellY(),Map_CountToCell(target.X), Map_CountToCell(target.Y));
//	if(Math_TwoPoint_Dis(Map_GetRobotCount(),target) <= (ROBOT_SIZE*CELL_COUNT_MUL))
//	{
//		Usprintf("target is near,compare two cells\n",__FUNCTION__,__LINE__);
//		target_course = Math_Course2Dest(Map_GetRobotCellX(),Map_GetRobotCellY(),Map_CountToCell(target.X), Map_CountToCell(target.Y));
//	}
//	else
	{
		target_course = Math_Course2Dest(Map_GetRobotCellX(),Map_GetRobotCellY(),Map_CountToCell(target.X), Map_CountToCell(target.Y));
//		target_course = Math_Course2Dest(Map_GetRobotCountX(), Map_GetRobotCountY(), target.X, target.Y);	
	}
	
	Usprintf("%s(%d)target_course:%d\n",__FUNCTION__, __LINE__,target_course);
	CM_HeadToCourse(ROTATE_TOP_SPEED, target_course);	//turn to target position
}
void CM_HeadToCourse(uint8_t speed, int16_t angle)
{
	int16_t diff = 0;
	static int32_t	angle_turned = 0;
	Action_t action = ACTION_NONE;
	
	diff = angle - Gyro_GetAngle(0);

	Usprintf("%s(%d):Angle(%d)Gyro(%d)Diff(%d)(%d)Bias(%d)Temp(%d)Scale(%d)\n",
	         __FUNCTION__, __LINE__, angle, Gyro_GetAngle(0), diff, (angle - Gyro_GetAngle(0)), Gyro_GetXAcc(), Gyro_GetYAcc(), Gyro_GetZAcc());

	while (diff >= 1800) 
	{
		diff = diff - 3600;
	}

	while (diff <= (-1800)) 
	{
		diff = diff + 3600;
	}


	if (((diff <= 1800 && diff >= 1700) || (diff >= -1800 && diff <= -1700))) 
	{
		if (diff <= 1800 && diff >= 1700) 
		{
			if (angle_turned < 0) 
			{
				Usprintf("%s(%d):Turn Left\n", __FUNCTION__, __LINE__);
				action = ACTION_LT;
				angle_turned += diff;
			} 
			else 
			{
				Usprintf("%s(%d):Turn Right\n", __FUNCTION__, __LINE__);
				action = ACTION_RT;
				angle_turned += (diff - 3600);
			}
		} 
		else 
		{
			if (angle_turned > 0) 
			{
				Usprintf("%s(%d):Turn Right\n", __FUNCTION__, __LINE__);
				action = ACTION_RT;
				angle_turned += diff;
			} 
			else 
			{
				Usprintf("%s(%d):Turn Left\n", __FUNCTION__, __LINE__);
				action = ACTION_LT;
				angle_turned += (3600 + diff);
			}
		}
	} 
	else 
	{
		if ((diff >= 0) && (diff <= 1800)) 
		{	// turn right
			Usprintf("%s(%d):Turn Left\n", __FUNCTION__, __LINE__);
			action = ACTION_LT;
		} 
		else if ((diff <= 0) && (diff >= (-1800))) 
		{
			Usprintf("%s(%d):Turn Right\n", __FUNCTION__, __LINE__);
			action = ACTION_RT;
		}
		angle_turned += diff;
	}
	
	
	Wheel_SetTargetSpeed(speed,speed);
	Wheel_SetRightTargetStep(3600);
	Wheel_SetLeftTargetStep(3600);
//	Action_SetMove(MOVE_ACT_HEAD2COURCE);
	Heading_SetTargetAngle(angle);

	if(action == ACTION_LT)
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

	Usprintf("%s(%d):Act_Move_Head to Course!\n",__FUNCTION__, __LINE__);
}
/*
	while robot reached new lane , check if robot's heading crossing this lane 
	@param: Start_Dir robot last lane direction
	@param: Wall_Dir robot wall follow side £¬left wall follow or right wall follow
	@return 1 robot' heading angle is crossing the new lane\
					0 robot' heading is parallel to new lane
*/
uint8_t CM_IsCrossLane(WallDir_t wall_dir)
{
	switch(wall_dir)
	{
		case WALLDIR_EAST_LEFT:	if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == SOUTH)return 1;
														break;
		case WALLDIR_EAST_RIGHT:	if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == NORTH)return 1;
														break;
		case WALLDIR_WEST_LEFT:	if(Path_GetRobotHeading4(Gyro_GetAngle(0))==NORTH)return 1;
														break;
		case WALLDIR_WEST_RIGHT:	if(Path_GetRobotHeading4(Gyro_GetAngle(0))==SOUTH)return 1;
														break;
		default:break;
		
	}
  return 0;
}
/*
	check if robot's moving back to same lane

	that means robot's moving back to last lane
	@param: Start_Dir robot last lane direction
	@param: Wall_Dir robot wall follow side £¬left wall follow or right wall follow
	@param:	Start_Pos robot last lane end pos
	@return 1 robot's moving back to last lane
					2 robot's Moving forward to last lane
					0 robot's normal moving
*/
uint8_t CM_IsBackCrossing(WallDir_t wall_dir,Point32_t start_count)
{
	switch(wall_dir)
	{
		case WALLDIR_EAST_LEFT:	if((Map_GetRobotCountY() - start_count.Y) > 120)return 1;
														break;
		case WALLDIR_EAST_RIGHT:	if((start_count.Y - Map_GetRobotCountY()) > 120)return 1;
														break;
		case WALLDIR_WEST_LEFT:	if((start_count.Y-Map_GetRobotCountY()) > 120)return 1;
														break;
		case WALLDIR_WEST_RIGHT:	if((Map_GetRobotCountY()-start_count.Y) > 120)return 1;
														break;
		default:break;
		
	}
  return 0;
}

volatile uint8_t g_backtostartpoint_flag = 0;
	
uint8_t CM_GetBackToStartPointFlag(void)
{
	return g_backtostartpoint_flag;
}
void CM_SetBackToStartPointFlag(uint8_t back_flag)
{
	g_backtostartpoint_flag = back_flag;
}

WallDir_t CM_SetTrapWallDir(WallDir_t temp_dir)
{
	WallDir_t temp = WALLDIR_NONE;
	switch(Path_GetRobotHeading4(Gyro_GetAngle(0)))
	{
		case EAST:
//			return  WALLDIR_EAST_RIGHT;
			temp = WALLDIR_EAST_RIGHT;
			break;
		case NORTH:
			if(temp_dir ==  WALLDIR_EAST_RIGHT)
			{
//				return WALLDIR_EAST_RIGHT;
			  temp = WALLDIR_EAST_RIGHT;
			}
			else
			{
//				return WALLDIR_WEST_RIGHT;
				temp = WALLDIR_WEST_RIGHT;
			}
			break;
		case WEST:
//			return  WALLDIR_WEST_RIGHT;
					temp = WALLDIR_WEST_RIGHT;
			break;
		case SOUTH:
			if(temp_dir ==  WALLDIR_WEST_RIGHT)
			{
//				return  WALLDIR_WEST_RIGHT;
				temp = WALLDIR_WEST_RIGHT;
			}
			else
			{
//				return  WALLDIR_EAST_RIGHT;
				 temp = WALLDIR_EAST_RIGHT;
			}
			break;
		default:
			break;
	}
	return temp;//WALLDIR_NONE;
}

volatile Point32_t g_current_target_cnt;
void CM_StoreCurrentTargetCnt(Point32_t target)
{
	g_current_target_cnt = target;
}
Point32_t CM_GetCurrentTargetCnt(void)
{
	return g_current_target_cnt;
}
volatile uint8_t g_wall_trapped_flag = 0;
void CM_SetWallTrapFlag(uint8_t flag)
{
	g_wall_trapped_flag = flag;
}
uint8_t CM_GetWallTrapFlag(void)
{
	return g_wall_trapped_flag;
}


volatile uint8_t g_mobility_event_cnt = 0;
volatile uint8_t g_mobility_ignore_flag = 0;
void CM_ResetMobilityEventCnt(void)
{
  g_mobility_event_cnt = 0;
}
uint8_t CM_GetMobilityEventCnt(void)
{
  return g_mobility_event_cnt;
}
void CM_MobilityEventCntIncrease(void)
{
  g_mobility_event_cnt++;
}

uint8_t CM_GetMobilityIgnoreFlag(void)
{
  return g_mobility_ignore_flag;
}
void CM_ResetMobilityIgnoreFlag(void)
{
  g_mobility_ignore_flag = 0;
}
/*CM_AbnormalHandler
  when there is something wrong with the motors or other devices ,
	the motor status will change,and the CM_AbnormalHandler will 
	check the abnormal device,if still bad,it will return to userterface,
  if it is normal,the robot will cintinue cleaning!
*/
void CM_AbnormalHandler(uint8_t status)
{	
	uint16_t error_cnt = 0;
	int16_t  temp_current = 0;
	uint8_t  check_side_brush_flag = 0;
	uint8_t  check_main_brush_flag = 0;	
	uint16_t check_side_brush_time_cnt = 0;
	uint16_t check_main_brush_time_cnt = 0;	
	uint16_t check_vac_time_cnt = 0;
  uint8_t  finish_flag = 0;
	uint8_t  bumper_first_back = 0;
	uint8_t  bumper_cliff_flag = 0;
  uint8_t  bumper_trigger_status = 0;
	uint8_t  cliff_trigger_status = 0; 
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();	
	
	/*only deal with one abnormal event*/
	if(status & CHECK_L_WHEEL)
	{
	  status =  CHECK_L_WHEEL;
	}
	else if(status & CHECK_R_WHEEL)
	{
	  status =  CHECK_R_WHEEL;
	}
	else if(status & CHECK_BUMPER)
	{
	  status =  CHECK_BUMPER;
	}
	else if(status & CHECK_MAIN_BRUSH)
	{
	  status =  CHECK_MAIN_BRUSH;
	}
	else if(status & CHECK_SIDE_BRUSH)
	{
	  status =  CHECK_SIDE_BRUSH;
	}	
	else if(status & CHECK_VACUUM)
	{
	  status =  CHECK_VACUUM;
	}	
	
	/*disable all the motors at first*/
	Wheel_Stop();
	WHEEL_DISABLE();
	if(status==CHECK_SIDE_BRUSH)Brush_Side_SetPWM(0);
	if(status==CHECK_MAIN_BRUSH)Brush_Main_SetPWM(0);		
	if(status==CHECK_VACUUM)Vacuum_TurnOff();
	ActList_Clear();
	Motor_SetCheckState(DISABLE);
	Cliff_SetDetectState(DISABLE);
	Motor_SetCheckState(DISABLE);	
	Usprintf("%s(%d): %d  \n",__FUNCTION__, __LINE__,status);
	ActList_Clear();
	
	/*abnormal handle events initialize*/
	switch(status)
  {
		case	CHECK_L_WHEEL:  
	  {	
			Wheel_ResetSlowCnt();
			/*motor init*/ 			
      /*wheel init*/
			/*action init*/					
			if(L_F==1)
			{			
				ActList_Add(MOVE_ACT_TURN_LEFT,1600,TURN_SPEED,TURN_SPEED);
			}
			else
			{
				ActList_Add(MOVE_ACT_TURN_RIGHT,1600,TURN_SPEED,TURN_SPEED);
			}	
			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
			ActList_Add(MOVE_ACT_STATIC,0,0,0);
			Action_SetMove(MOVE_ACT_HANDLER);			
		}				
	  break;
		case	CHECK_R_WHEEL: 
    {
			Wheel_ResetSlowCnt();
			/*motor init*/			
      /*wheel init*/
			/*action init*/					
			if(R_F==1)
			{
				ActList_Add(MOVE_ACT_TURN_RIGHT,1600,TURN_SPEED,TURN_SPEED);
			}
			else
			{
				ActList_Add(MOVE_ACT_TURN_LEFT,1600,TURN_SPEED,TURN_SPEED);
			}	
			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
			ActList_Add(MOVE_ACT_STATIC,0,0,0);
			Action_SetMove(MOVE_ACT_HANDLER);			
		}
	  break;
		case	CHECK_BUMPER:
		{
			/*move back quickly*/
			Motor_SetCheckState(DISABLE);
			bumper_first_back = 1;
			ActList_Add(MOVE_ACT_BACK,400,BACK_SPEED,BACK_SPEED);
			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
			bumper_trigger_status = Bumper_GetTrigStatus();
//			if(bumper_trigger_status & LeftBumperTrig)
//			{
//				ActList_Add(MOVE_ACT_TURN_RIGHT,1000,TURN_SPEED,TURN_SPEED);
//			}
//			else
//			{
//				ActList_Add(MOVE_ACT_TURN_LEFT,1000,TURN_SPEED,TURN_SPEED);
//			}
			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);					
			ActList_Add(MOVE_ACT_STATIC,0,0,0);
			Action_SetMove(MOVE_ACT_HANDLER);			
		}
	  break;	
		case	CHECK_MAIN_BRUSH:
		{
			/*motor init*/		
      /*wheel init*/		
			/*action init*/
			/*move back for a distance of one robot size*/
			ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
			ActList_Add(MOVE_ACT_TURN_LEFT,1600,TURN_SPEED,TURN_SPEED);
			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
			ActList_Add(MOVE_ACT_STATIC,0,0,0);
			Action_SetMove(MOVE_ACT_HANDLER);		
		}
	  break	;		
		case	CHECK_SIDE_BRUSH:
		{
			/*motor init*/		
      /*wheel init*/		
			/*action init*/
			/*move back for a distance of one robot size*/
			ActList_Add(MOVE_ACT_BACK,300,BACK_SPEED,BACK_SPEED);
			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
			ActList_Add(MOVE_ACT_TURN_LEFT,1600,TURN_SPEED,TURN_SPEED);
			ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
			ActList_Add(MOVE_ACT_STATIC,0,0,0);
			Action_SetMove(MOVE_ACT_HANDLER);		
		}
	  break	;
		case	CHECK_VACUUM:
		{
		  Vacuum_ResetFailFlag();
		  /*just turn on the vac*/
		  Motor_SetPower(VAC_SPEED_NORMAL,0,0);      			
		  /*wheel init*/			
      /*action init*/	
			Wheel_Stop();			
		}
	  break;		
    default:break;		
	}	
	while((Mode_GetMode() != MODE_USERINTERFACE))
	{
		switch(Action_GetMove())
		{
			case MOVE_ACT_FORWARD:														
														if((Wheel_GetLeftStep() >= Wheel_GetLeftTargetStep()) || (Wheel_GetRightStep() >= Wheel_GetRightTargetStep()))
                          	{
															Usprintf("finish forward!!\n");
															finish_flag = 1;
														}
														bumper_trigger_status = Bumper_GetTrigStatus();
														if(bumper_trigger_status)
														{
															bumper_cliff_flag = 1;
															if(bumper_trigger_status & LeftBumperTrig)
															{
																ActList_Clear();
																ActList_Add(MOVE_ACT_BACK,500,BACK_SPEED,BACK_SPEED);
																ActList_Add(MOVE_ACT_DECELERATE,0,0,0);	
																ActList_Add(MOVE_ACT_STATIC,0,0,0);																	
																Action_SetMove(MOVE_ACT_HANDLER);																			
															}
															else if(bumper_trigger_status & RightBumperTrig)
															{
																ActList_Clear();
																ActList_Add(MOVE_ACT_BACK,500,BACK_SPEED,BACK_SPEED);
																ActList_Add(MOVE_ACT_DECELERATE,0,0,0);																		
																ActList_Add(MOVE_ACT_STATIC,0,0,0);
																Action_SetMove(MOVE_ACT_HANDLER);																							
															}																																																																																																												
														}
														cliff_trigger_status = Cliff_GetDetectiontProcess_Result();
														if(cliff_trigger_status)
														{
															bumper_cliff_flag = 1;
															ActList_Clear();
															ActList_Add(MOVE_ACT_BACK,500,BACK_SPEED,BACK_SPEED);
															ActList_Add(MOVE_ACT_DECELERATE,0,0,0);																		
															ActList_Add(MOVE_ACT_STATIC,0,0,0);
															Action_SetMove(MOVE_ACT_HANDLER);	
														}
														break;
		
			case MOVE_ACT_TURN_LEFT:
														Action_WaitForMoveEnded();
														break;
		
			case MOVE_ACT_TURN_RIGHT:
														Action_WaitForMoveEnded();
														break;	
			
			case MOVE_ACT_DECELERATE:
														if((Wheel_GetLeftSpeed() <= 10)&&(Wheel_GetRightSpeed() <= 10))
														{
															ActList_Switch();
														}
													  break;
			
			case MOVE_ACT_STATIC: 
														//Wheel_Stop();
														/*checking the mobility error ,meet the bumper or cliff events ,break and continue cleaning*/
														if(bumper_cliff_flag == 1)
														{
															bumper_cliff_flag = 0;
															finish_flag = 1;	
														}			
														/*no wheel error ,break and continue cleaning*/
														if(status & CHECK_L_WHEEL || status & CHECK_R_WHEEL)
														{
															finish_flag = 1;													
														}
														/*if bumper events again,move back again,if no,then break and continue cleaning*/
														else if(status & CHECK_BUMPER)
														{		
															if(bumper_first_back)
															{	
																bumper_first_back = 0;
																bumper_trigger_status = Bumper_GetTrigStatus();
																if(bumper_trigger_status)
																{
																	ActList_Add(MOVE_ACT_BACK,500,BACK_SPEED,BACK_SPEED);
																	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);																	
//																	if(bumper_trigger_status & LeftBumperTrig)
//																	{
//																		ActList_Add(MOVE_ACT_TURN_RIGHT,1000,TURN_SPEED,TURN_SPEED);
//																	}
//																	else
//																	{
//																		ActList_Add(MOVE_ACT_TURN_LEFT,1000,TURN_SPEED,TURN_SPEED);
//																	}																
																	ActList_Add(MOVE_ACT_STATIC,0,0,0);
																	Action_SetMove(MOVE_ACT_HANDLER);	
																}
																else
																{
																	finish_flag = 1;
																}
															}														
															else
															{
																bumper_trigger_status = Bumper_GetTrigStatus();
																if(bumper_trigger_status)
																{
																	Error_SetCode(ERROR_BUMPER);																	
																}
																finish_flag = 1;
															}																													
														}
														/*no brush error ,break and continue cleaning*/
														else if (status & CHECK_SIDE_BRUSH)
														{
															/*just turn on the brush*/
														  Motor_SetPower(0,CLEAN_SIDE_BRUSH_POWER,CLEAN_MAIN_BRUSH_POWER);	
															check_side_brush_flag = 1;														
														}
														else if (status & CHECK_MAIN_BRUSH)
														{
															/*just turn on the brush*/
														  Motor_SetPower(0,CLEAN_SIDE_BRUSH_POWER,CLEAN_MAIN_BRUSH_POWER);	
															check_main_brush_flag = 1;														
														}														
													  break;

			case MOVE_ACT_HANDLER:
														ActList_Switch();
													  break;
			
			case MOVE_ACT_BACK:
													  Action_WaitForMoveEnded();
			                      break;
			default:break;
		
		}
		/*check wheel current*/	
	 	if(status & CHECK_L_WHEEL || status & CHECK_R_WHEEL)
		{
			if(Action_GetMove() == MOVE_ACT_TURN_LEFT || Action_GetMove() == MOVE_ACT_TURN_RIGHT)
			{
				if(status & CHECK_L_WHEEL)
				{
					temp_current = Wheel_GetLeftCurrent();
				}
				else if (status & CHECK_R_WHEEL)
				{
					temp_current = Wheel_GetRightCurrent();
				}
				Usprintf("wheel current:%d\n",temp_current);
				if(temp_current > WHEEL_STALL_LIMIT  || temp_current < WHEEL_STALL_NOLOAD) 
				{
					error_cnt++;	
					if(error_cnt > 75)//1.5 seconds 
					{
						error_cnt = 0;
						finish_flag = 1;
						if(status & CHECK_L_WHEEL)
						{
							Error_SetCode(ERROR_LEFT_WHEEL);
						}
						else
						{
							Error_SetCode(ERROR_RIGHT_WHEEL);
						}
					}
				}	
				else
				{
					error_cnt = 0;				
				}
			}		
		}
		/*check side brush current*/
		if(check_side_brush_flag)
		{			
			check_side_brush_time_cnt++;
			if(check_side_brush_time_cnt > 150)//3 seconds later,break
			{
				check_side_brush_flag = 0;
				check_side_brush_time_cnt = 0;
			  finish_flag = 1;
			}
		  temp_current = Side_Brush_GetCurrent();
			Usprintf("brush current:%d\n",temp_current);
			if(temp_current > SIDE_BRUSH_STALL_LIMIT  || temp_current < SIDE_BRUSH_STALL_NOLOAD)
			{
				error_cnt++;	
				if(error_cnt > 50)//1 second
				{
					error_cnt = 0;
					Error_SetCode(ERROR_SIDE_BRUSH);
					finish_flag = 1;
				}
			}	
			else
			{
				error_cnt = 0;				
			}						
		}		
		/*check main brush current*/
		if(check_main_brush_flag)
		{			
			check_main_brush_time_cnt++;
			if(check_main_brush_time_cnt > 150)//3 seconds later,break
			{
				check_main_brush_flag = 0;
				check_main_brush_time_cnt = 0;
			  finish_flag = 1;
			}
		  temp_current = Main_Brush_GetCurrent();
			Usprintf("brush current:%d\n",temp_current);
			if(temp_current > MAIN_BRUSH_STALL_LIMIT  || temp_current < MAIN_BRUSH_STALL_NOLOAD)
			{
				error_cnt++;	
				if(error_cnt > 50)//1 second
				{
					error_cnt = 0;
					Error_SetCode(ERROR_MAIN_BRUSH);
					finish_flag = 1;
				}
			}	
			else
			{
				error_cnt = 0;				
			}						
		}			
		/*check vac current*/
		if(status & CHECK_VACUUM)
		{
			check_vac_time_cnt++;
			if(check_vac_time_cnt > 150)// 3 seconds
			{
				check_vac_time_cnt = 0;
			  finish_flag = 1;
			}
			temp_current = Vacuum_GetCurrent();
			Usprintf("vac current:%d\n",temp_current);
			if(temp_current > VACUUM_STALL_LIMIT  || temp_current < VACUUM_STALL_NOLOAD)
			{
				error_cnt++;	
				if(error_cnt > 75)//1.5 seconds 
				{
					finish_flag = 1;
					error_cnt = 0;
					if(temp_current > VACUUM_STALL_LIMIT)
					{  
						Error_SetCode(ERROR_FAN_H);
					}
					if(temp_current < VACUUM_STALL_NOLOAD)
					{
						Error_SetCode(ERROR_FAN_L);
					}
				}
			}	
			else
			{
				error_cnt = 0;				
			}		   		
		}			
		/*finish the abnormal events*/
		if(finish_flag)
		{
			finish_flag = 0;
			break;
		}
				
		vTaskDelayUntil(&xLastWakeTime,20 / portTICK_RATE_MS);
	}	
	/*if no error codes,restart all the motors*/
	if(!Error_GetCode())
	{
		Motor_SetCheckState(ENABLE);		
		Motor_WorkConfigure();
	}
}

/*CM_Drying
	drying moves, switch all motors to spot cleaning mode power 
	move back and forward twice
	stop while bumper or cliff triggered
	stop all motors after drying finish
*/
void CM_Drying(void)
{
	CM_Move_Turn(ROTATE_TOP_SPEED,0);
	Wheel_SetSpeed(0,0);
	Wheel_SetTargetSpeed(0,0);
	
//	uint8_t ret_val = 0;
//	portTickType xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();
//	

//	Motor_SpotConfigure();

//	Usprintf("%s(%d):Drying Start\n",__FUNCTION__, __LINE__);
//	
//	PathList_Clear();
//	ActList_Clear(); 
	/*forward and backward*/
	
//	ActList_Add(MOVE_ACT_BACK,800,RUN_SLOW_SPEED,RUN_SLOW_SPEED);
//	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//	ActList_Add(MOVE_ACT_FORWARD,800,RUN_SLOW_SPEED,RUN_SLOW_SPEED);
//	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//	ActList_Add(MOVE_ACT_BACK,800,RUN_SLOW_SPEED,RUN_SLOW_SPEED);
//	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//	ActList_Add(MOVE_ACT_FORWARD,800,RUN_SLOW_SPEED,RUN_SLOW_SPEED);
//	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//	ActList_Add(MOVE_ACT_BACK,800,RUN_SLOW_SPEED,RUN_SLOW_SPEED);
//	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//	ActList_Add(MOVE_ACT_FORWARD,800,RUN_SLOW_SPEED,RUN_SLOW_SPEED);
//	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//	ActList_Add(MOVE_ACT_STATIC,0,0,0);	

	/*turn round*/
//	ActList_Add(MOVE_ACT_TURN_LEFT,1500,RUN_SLOW_SPEED,RUN_SLOW_SPEED);
//	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//	
//	ActList_Add(MOVE_ACT_TURN_LEFT,1500,RUN_SLOW_SPEED,RUN_SLOW_SPEED);
//	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//	
//	ActList_Add(MOVE_ACT_TURN_LEFT,1500,RUN_SLOW_SPEED,RUN_SLOW_SPEED);
//	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
//	
//	ActList_Add(MOVE_ACT_TURN_LEFT,1500,RUN_SLOW_SPEED,RUN_SLOW_SPEED);
//	ActList_Add(MOVE_ACT_DECELERATE,0,0,0);
		
//	ActList_Add(MOVE_ACT_STATIC,0,0,0);	

//	Action_SetMove(MOVE_ACT_HANDLER);		
//	
//	
//	while(Mode_GetMode() == MODE_NAVIGATION)
//	{
//		switch(Action_GetMove())
//		{
//			case MOVE_ACT_FORWARD:    if(OBS_IsWallNear())
//																{
//																	Wheel_SetTargetSpeed(RUN_SLOW_SPEED,RUN_SLOW_SPEED);
//																}	
//															  Action_WaitForMoveEnded();																
//																break;
//			case MOVE_ACT_DECELERATE:	if((Wheel_GetLeftSpeed() <= 3)&&(Wheel_GetRightSpeed() <= 3))
//																{																	
//																	Action_SetMove(MOVE_ACT_HANDLER);																		
//																}
//																break;	
//			case MOVE_ACT_BACK:       Action_WaitForMoveEnded();
//																break;
//			case MOVE_ACT_TURN_LEFT:  Action_WaitForMoveEnded();
//																break;
//			case MOVE_ACT_TURN_RIGHT: Action_WaitForMoveEnded();
//																break;																
//			case MOVE_ACT_HANDLER:		ActList_Switch();
//																break;
//			case MOVE_ACT_STATIC:     ret_val = 1;
//																break;	  																
//			default:break;
//		}
//		if(Bumper_GetTrigStatus()) 
//		{
//			Usprintf("%s(%d):Bumper Event!\n",__FUNCTION__, __LINE__);
//			break;
//		}
//		if(Cliff_GetDetectiontProcess_Result())
//		{
//			Usprintf("%s(%d):Cliff Event!\n",__FUNCTION__, __LINE__);
//			break;
//		}
//		if(ret_val)break;
//		vTaskDelayUntil(&xLastWakeTime,50/portTICK_RATE_MS);
//	}
//	Usprintf("%s(%d):Drying finished!\n",__FUNCTION__, __LINE__);
//	ActList_Clear();
//	Display_SetCleanKeyWater(LED_CLEAN_GALL);
//	Display_SetMode(LED_MODE_STATIC);
//	Display_SetBrightness(3);
	//Motor_DisableAll();
}



void CM_WaitForGyroCal(void)
{
//	Gyro_Calibration_Cmd(ENABLE);
//	vTaskDelay(500/portTICK_RATE_MS);
//	Gyro_Calibration_Cmd(DISABLE);
}

uint8_t CM_WallFollowBoundaryEvent(int32_t *wall_proportion,int32_t *wall_distance,Point16_t robot_cell)
{
//	if(robot_cell.X > (Map_GetBoundary().east - 2))//over east boundary
//	{
//		if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == WEST)*wall_proportion = 100;
//		else *wall_proportion += (Map_GetRobotCountX() - (Map_GetBoundary().east_cnt - CELL_COUNT_MUL_2))/3;
//		*wall_distance = 100;
//		return 1;
//	}

//	if(robot_cell.X < (Map_GetBoundary().west + 2))//over west boundary
//	{
//		if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == EAST)*wall_proportion = 100;
//		else *wall_proportion += ((Map_GetBoundary().west_cnt + CELL_COUNT_MUL_2) - Map_GetRobotCountX())/3;
//		*wall_distance = 100;
//		return 1;
//	}
//	
//	if(robot_cell.Y > (Map_GetBoundary().north - 2))//over north boundary
//	{
//		if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == SOUTH)*wall_proportion = 100;
//		else *wall_proportion += (Map_GetRobotCountY() - (Map_GetBoundary().north_cnt - CELL_COUNT_MUL_2))/3;
//		*wall_distance = 100;
//		return 1;
//	}
//	
//	if(robot_cell.Y < (Map_GetBoundary().south + 2))//over south boundary
//	{
//		if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == NORTH)*wall_proportion = 100;
//		else *wall_proportion += ((Map_GetBoundary().south_cnt + CELL_COUNT_MUL_2) - Map_GetRobotCountY())/3;
//		*wall_distance = 100;
//		return 1;
//	}//edit by vin
	
	return 0;
}


uint8_t CM_CheckMeetBoundary(Point16_t robot_cell)
{
	uint8_t rvl = 0;
	int32_t x_offset = 0,y_offset = 0;
	int8_t	i = 0;
	
	for(i = -1;i <= 1;i++)
	{
		CM_CellOffset(Gyro_GetAngle(0), i*CELL_SIZE, 2*CELL_SIZE, &x_offset, &y_offset);
		if(Map_GetCell(x_offset,y_offset) == BLOCKED_BOUNDARY)
		{
			rvl = 1;
			break;
		}	
	}
	return rvl;
}


volatile CM_t g_cm_type = CM_NORMAL;

void CM_SetCMType(CM_t c)
{
	g_cm_type = c;
}

CM_t CM_GetCMType(void)
{
	return g_cm_type;
}

uint8_t CM_ReachSpotBoundary(void)
{
	int32_t x_offset = 0,y_offset = 0;
  int8_t i = 0;
	uint8_t reval = 0;

	for(i = -1; i < 1; i++ )
	{
		CM_CellOffset(Gyro_GetAngle(0), i, 2*CELL_SIZE, &x_offset, &y_offset);	

		if(Map_GetCell(x_offset,y_offset) == BLOCKED_BOUNDARY)
		{
		  reval = 1;
			break;
		}
	}
  return reval;
}

volatile uint8_t g_wall_out_trap_cnt = 0;

void CM_WallOutTrapCntIncrease(void)
{
	g_wall_out_trap_cnt++;
}

void CM_ResetWallOutTrapCnt(void)
{
  g_wall_out_trap_cnt = 0;
}

uint8_t CM_IsWallOutTrapOverLimit(void)
{
	Usprintf("wall out trap cnt:%d\n",g_wall_out_trap_cnt);
	if(g_wall_out_trap_cnt > 50)
	{
     return 1;
	}
	else
	{
	   return 0;
	}
}

volatile uint8_t g_near_target_flag = 0;

void CM_SetNearTargetFlag(uint8_t flag)
{
	g_near_target_flag = flag;
}

uint8_t CM_GetNearTargetFlag(void)
{
	return g_near_target_flag;
}


uint8_t CM_Move_Back(uint8_t speed,uint16_t dis)
{
	portTickType xStartTime = xTaskGetTickCount();	
	
	Action_SetMove(MOVE_ACT_HANDLER);	
	Wheel_SetDir(WHEEL_DIR_BACKWARD);
	Wheel_SetTargetSpeed(speed,speed);
	Wheel_ResetStep();
	Wheel_SetTargetStep(dis,dis);
	Usprintf("%s(%d)\n",__FUNCTION__,__LINE__);
	
	while(Mode_GetMode() == MODE_NAVIGATION)
	{
		if(xTaskGetTickCount() >= (xStartTime + 5*1000))
		{
			Usprintf("%s(%d),time out\n",__FUNCTION__,__LINE__);
			return 1;
		}
		
		if(Wheel_LeftStepReached(Wheel_GetLeftTargetStep())||Wheel_RightStepReached(Wheel_GetRightTargetStep()))
		{
			break;
		}
		
		if(Wheel_GetDir() != WHEEL_DIR_BACKWARD)Wheel_SetDir(WHEEL_DIR_BACKWARD);
		vTaskDelay(20/portTICK_RATE_MS);		
	}
	return 0;
}

uint8_t CM_Move_Turn(uint8_t speed,uint16_t dis)
{
	portTickType xStartTime = xTaskGetTickCount();	
	int32_t angle_diff = 0;
	
	Heading_SetTargetAngle(dis);
	Action_SetMove(MOVE_ACT_HANDLER);	
	Wheel_SetDir(WHEEL_DIR_BACKWARD);
	Wheel_SetTargetSpeed(speed,speed);
	if(Battery_GetVoltage()%2==0)
	{
		Wheel_SetDir(WHEEL_DIR_LEFT);
	}
	else
	{
		Wheel_SetDir(WHEEL_DIR_RIGHT);	
	}
	
	Usprintf("%s(%d)\n",__FUNCTION__,__LINE__);
	
	while(Mode_GetMode() == MODE_NAVIGATION)
	{
		angle_diff = Math_Diff_int(Heading_GetTargetAngle(), Gyro_GetAngle(0));
		if(xTaskGetTickCount() >= (xStartTime + 5*1000))
		{
			Usprintf("%s(%d),time out\n",__FUNCTION__,__LINE__);
			return 1;
		}
		if(angle_diff < 20)
		{																						
			break;
		}
		else if(angle_diff < 400)
		{
			angle_diff /= 20;
			if(angle_diff < 6)angle_diff = 6;
			Wheel_SetTargetSpeed(angle_diff,angle_diff);
		}		
		vTaskDelay(20/portTICK_RATE_MS);		
	}
	return 0;
}

uint8_t CM_IsLoop_Fault(void)
{
	if(Key_GetStatus())
	{
		Usprintf("%s(%d)\n",__FUNCTION__,__LINE__);
		return 1;
	}	
	
	if(Rcon_RemoteKey(Remote_Clean))
	{
		Usprintf("%s(%d)\n",__FUNCTION__,__LINE__);
		return 1;
	}	
	if(Cliff_GetDetectiontProcess_Result())
	{
		Usprintf("%s(%d)\n",__FUNCTION__,__LINE__);
//		return 1;
	}	
	
	return 0;
}

/*robot heading*/
volatile int16_t g_heading_target_angle = 0;
void Heading_SetTargetAngle(int16_t angle)
{
	g_heading_target_angle = angle;
}
int16_t Heading_GetTargetAngle(void)
{
	return g_heading_target_angle;
}


