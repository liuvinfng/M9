 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Random Path Cleaning Function
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
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


volatile SpotDir_t g_spot_outin = SPOT_DIR_OUT;
uint8_t g_spot_clear_flag = 0;
SpotDir_t g_spot_dir = SPOT_DIR_CW;
volatile uint8_t g_spot_lane_idx = 0;
Point16_t Spot_Wall_Cells[32] = {0};
volatile uint8_t g_spot_wall_idx = 0;

/*New Spot Mode*/
void Spot_Mode(void)
{
	CMState_t cm_state = CM_STATE_GYROINIT;
	uint8_t gyro_init_cnt = 0;
	uint8_t pathlist_state = 0;
	Point16_t home_position_cell;
	Point16_t temp_robot_cell;//temp_pos_cell, 
	uint8_t temp_idx = 0;

	PathList_t pathlist_point;

	uint8_t no_path_way_flag = 0;
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
  ICM42688_Config();
	Gyro_Cmd(DISABLE);
	osDelay(50);
	osDelay(300);
	Gyro_Cmd(ENABLE);
	osDelay(300);	
	
	
	Motor_SpotConfigure();
	Map_Initialize();
	Spot_SetSpiralDir(SPOT_DIR_OUT);
	Spot_SetClearFlag(1);
  pathlist_point.cell_pos.X=0;
  pathlist_point.cell_pos.Y=0;
	home_position_cell =  pathlist_point.cell_pos;
	

	Wall_AdjustBaseline();
	Usprintf("%s(%d):Spot Start!",__FUNCTION__, __LINE__);
	PathList_Clear();
	CM_ResetWallOutTrapCnt();
	temp_robot_cell.X = 1;
	temp_robot_cell.Y = 0;
	Spot_SetDir(SPOT_DIR_CCW);
	Spot_PathPlanning(temp_robot_cell);
	
	while((Mode_GetMode() != MODE_USERINTERFACE))//edit by vin
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
							}
							else
							{
								Usprintf("%s(%d):Gryo_GetUpdateFlag = %d\n",__FUNCTION__, __LINE__,Gryo_GetUpdateFlag());
								cm_state = CM_STATE_TOURING;	
								OBS_ResetTrigValue();
							}
						}
						break;
			case	CM_STATE_DRYING:		
															break;
			case	CM_STATE_TOURING:    
						pathlist_state = PathList_Out(&pathlist_point);
						if(!pathlist_state)//no pathlist 
						{						
							if(no_path_way_flag)
							{
								Mode_SetMode(MODE_USERINTERFACE);
								Speaker(SPK_CLEAN_FINISH);
								Motor_DisableAll();
								break;														
							}
							Usprintf("%s(%d):PathList_State = %d PathList_Point(%d,%d) S= %d \n",
								__FUNCTION__, __LINE__,pathlist_state,pathlist_point.cell_pos.X,pathlist_point.cell_pos.Y,pathlist_point.status);
							if(Path_RobotCloseToTargetCell(home_position_cell,3))
							{
								Mode_SetMode(MODE_USERINTERFACE);
								Speaker(SPK_CLEAN_FINISH);
								Motor_DisableAll();
								break;
							}
							else
							{
								Usprintf("%s(%d):No Path nor home!\n",__FUNCTION__, __LINE__);
								no_path_way_flag = 1;
								temp_robot_cell = Map_GetRobotCell();									
								Spot_PathPlanning(temp_robot_cell);	
								break;
							}
						}
						if(pathlist_state)
						{
							no_path_way_flag = 0;
							Usprintf("%s(%d):PathList_State = %d PathList_Point(%d,%d) S= %d \n",
								__FUNCTION__, __LINE__,pathlist_state,pathlist_point.cell_pos.X,pathlist_point.cell_pos.Y,pathlist_point.status);

							if(pathlist_point.status == SHORT_PATH)
							{
								Usprintf("%s(%d):Move to Map in shortest Path\n",__FUNCTION__, __LINE__);
								if(Path_RobotCloseToTargetCell(pathlist_point.cell_pos,2))
								{
									pathlist_state = PathList_Out(&pathlist_point);
								}
								if(CM_MoveToMap(pathlist_point.cell_pos,SPOT_CLEAN) == MT_OBSTACLE)
								{
									CM_UpdateMapBumper(ACTION_NONE, AllBumperT);		
									if(Spot_GetSpiralDir() == SPOT_DIR_OUT)
									{
										temp_robot_cell = Map_GetRobotCell();
										temp_idx = Math_GetMax(Math_Absolute(temp_robot_cell.X),Math_Absolute(temp_robot_cell.Y));//get current clean lane
										Usprintf("%s(%d):robot cell (%d,%d) idx = %d\n",__FUNCTION__, __LINE__, temp_robot_cell.X,temp_robot_cell.Y,temp_idx);
										Spot_SetLaneIdx(temp_idx);
										if(CM_WallToMap((Point16_t){0,0}, WALLDIR_WEST_RIGHT, WALL_SPOT) == MT_ARRIVED)
										{
											temp_robot_cell = Map_GetRobotCell();																		
											Spot_PathPlanning(temp_robot_cell);											
										}										
									}
									else
									{
										temp_robot_cell = Map_GetRobotCell();																		
										Spot_PathPlanning(temp_robot_cell);										 									
									}										
								}
							}
						}
						break;
		}
		vTaskDelayUntil(&xLastWakeTime,50/portTICK_RATE_MS);
	}
	Gyro_Cmd(DISABLE);
}


void Spot_PathPlanning(Point16_t start_cell)
{
  int8_t idx = 0;
	PathList_t  temp_pathlist;
	Point16_t temp_start_cell;
	uint8_t step = 1;
	 
	PathPoint_ClearAllPoints();
	PathList_Clear();
	
	PC_NavDebug_AllMap();

	temp_pathlist.cell_pos = start_cell;
	temp_pathlist.status = SHORT_PATH;
	PathList_AddNewPoint(temp_pathlist);//add a start cell first
	idx = Math_GetMax(Math_Absolute(start_cell.X),Math_Absolute(start_cell.Y));//get current clean lane

	temp_start_cell = start_cell;
	
	Usprintf("\n\r\n\r%s(%d):idx(%d) start_cell(%d,%d) idx(%d)\n",__FUNCTION__, __LINE__\
		,idx , start_cell.X,start_cell.Y, idx);
	if((idx > 3) && (Spot_GetSpiralDir() == SPOT_DIR_OUT))
	{
		Spot_SetSpiralDir(SPOT_DIR_IN);
	}
	if(Spot_GetSpiralDir() == SPOT_DIR_OUT)
	{
		for(;idx < 4;idx +=1)
		{
			if(Spot_IsLaneCleaned(idx,  temp_start_cell, Spot_GetDir()))
			{
			  PathPoint_AddPathPointToPathList(0);
				temp_start_cell = Spot_GetNextLanePos(PathList_ReadLastPath().cell_pos,1,step);	
			}
			else
			{
				temp_start_cell = Spot_GetNextLanePos(start_cell,1,step);	
			}					
		}
	}
	else
	{
		 Spot_ClearCleanCells(idx);
	}
	if(idx > 3)
	{
		idx = 4;
		Spot_ClearCleanCells(idx);
	}
	
	for(;idx > 0;idx -=1)
	{
		if(Spot_IsLaneCleaned(idx,  temp_start_cell, Spot_GetDir()))
		{
			PathPoint_AddPathPointToPathList(0);
			temp_start_cell = Spot_GetNextLanePos(PathList_ReadLastPath().cell_pos,0,step);		
		}
		else
		{
		  temp_start_cell = Spot_GetNextLanePos(start_cell,0,step);	
		}
	}

	PathList_ShowAll();
	
}


/*check if current lane have been cleaned up
*/
uint8_t Spot_IsLaneCleaned(uint8_t idx ,Point16_t start_cell, SpotDir_t dir)
{
	double angle = 0;
	double tan_result = 0;
	uint16_t angle_cnt = 0;
	int16_t angle_32 = 0;
	int32_t x = idx, y = idx , px = start_cell.X, py = start_cell.Y;
	uint8_t ret_val = 0;

  Usprintf("%s(%d):idx(%d) start_cell(%d,%d) dir(%d)\n",__FUNCTION__, __LINE__\
		,idx , start_cell.X,start_cell.Y,  dir);
	
	PathPoint_ClearAllPoints();

	PathPoint_AddOnePoint(start_cell);

	angle = Math_Course2Dest(0,0,start_cell.X,start_cell.Y)/10;
	
	for(angle_cnt = 0;angle_cnt < 361;angle_cnt ++)
	{	 
		if(Mode_GetMode() == MODE_USERINTERFACE)break;
		angle += dir;
		if(angle > 359)angle = 0;
		else if(angle < 0 )angle = 360;
		angle_32 = angle;

		if( angle == 45)
		{
			tan_result = 1;
		}
		else if( angle == 135)
		{
			tan_result = -1;
		}
		else if( angle == 225)
		{
			tan_result = 1;
		}
		else if( angle == 315)
		{
			tan_result = -1;
		}
		else 
		{
			tan_result = tan(Math_Deg2Rad(angle,1));
			if(tan_result == 0)continue;//ignore the 0 degree
		}
		
		if((angle_32 <= 45) || (angle_32 > 315))
		{
			x = idx;
			y = x * tan_result;
		}
		else if((angle_32 > 45) && (angle_32 <= 135))
		{
			y = idx;
			x = y / tan_result;
		}
		else if((angle_32 > 135) && (angle_32 <= 225))
		{
			x = -idx;
			y = x * tan_result;
		}
		else if((angle_32 > 225) && (angle_32 <= 315))
		{
			y = -idx;
			x = y / tan_result;
		}
		if((x != px) || (y != py))//new point
		{
			/*check if near start point*/
			if(angle_cnt > 180)
			{
				if(x == start_cell.X)
				{
					if(Map_IsRobotAccessible(x, y))
					{
						if(Math_Diff_int(y,start_cell.Y) < 3)
						{
							PathPoint_AddOnePoint((Point16_t){x,y});
							ret_val |= 0x01;
							break;
						}					
					}
				}
				if(y == start_cell.Y)
				{
					if(Map_IsRobotAccessible(x, y))
					{
						if(Math_Diff_int(x,start_cell.X) < 3)
						{
							PathPoint_AddOnePoint((Point16_t){x,y});
							ret_val |= 0x01;
							break;
						}												
					}
				}
			}			
			/*check if the cell is unclean*/
			if(Map_BrushBlockUnclean(x,y))
			{
				if(Map_IsRobotAccessible(x, y))
				{
					PathPoint_AddOnePoint((Point16_t){x,y});
					ret_val |= 0x01;
				}	
				else
				{
					break;
				}	
			}
			if(!Map_IsRobotAccessible(x, y))
			{
				break;
			}					
		}
		px = x;
		py = y;
	}
	PathPoint_SortPoints();
	return ret_val;
}

Point16_t Spot_GetNextLanePos(Point16_t start_cell,uint8_t dir,uint8_t step)
{
	Point16_t ret_cell = start_cell;
	int8_t diff = 0;
	Usprintf("%s(%d):start(%d,%d) dir=%d\n",__FUNCTION__, __LINE__,start_cell.X,start_cell.Y,dir);
	
	if(Math_Abs_int(start_cell.X) >= Math_Abs_int(start_cell.Y) )
	{
		if(start_cell.X > 0)//EAST
		{
			Usprintf("%s(%d):East\n",__FUNCTION__, __LINE__);
			if(dir)
			{
				ret_cell.X += 1;	
			}
			else
			{
				ret_cell.X -= 1;
			}
		}
		else                //WEST
		{
			Usprintf("%s(%d):West\n",__FUNCTION__, __LINE__);
			if(dir)
			{
				ret_cell.X -= 1;
			}
			else
			{
				ret_cell.X += 1;
			}
		}
		if(!dir)
		{
			diff = Math_Abs_int(ret_cell.Y) - Math_Abs_int(ret_cell.X);
			if(diff > 0)
			{
				if(ret_cell.Y > 0)ret_cell.Y -= diff;
				if(ret_cell.Y < 0)ret_cell.Y += diff;
			}
		}
	}
	else
	{
		if(start_cell.Y > 0)//NORTH
		{
			Usprintf("%s(%d):North\n",__FUNCTION__, __LINE__);
			if(dir)
			{
				ret_cell.Y += 1;
			}
			else
			{
				ret_cell.Y -= 1;
			}
		}
		else                //SOUTH
		{
			Usprintf("%s(%d):South\n",__FUNCTION__, __LINE__);
			if(dir)
			{
				ret_cell.Y -= 1;
			}
			else
			{
				ret_cell.Y += 1;
			}
		}
		if(!dir)
		{
			diff = Math_Abs_int(ret_cell.X) - Math_Abs_int(ret_cell.Y);
			if(diff > 0)
			{
				if(ret_cell.X > 0)ret_cell.X -= diff;
				if(ret_cell.X < 0)ret_cell.X += diff;
			}
		}
	}
	Usprintf("%s(%d):result(%d,%d) dir=%d\n",__FUNCTION__, __LINE__,ret_cell.X,ret_cell.Y,dir);
	
	return ret_cell;
}


void PathPoint_SortPoints(void)
{
	uint16_t i = 0;
	Point16_t point_1, point_2, point_3;
	
	Usprintf("%s(%d):%d\n",__FUNCTION__, __LINE__,PathPoint_GetPointCnt());
	
	while(i < (PathPoint_GetPointCnt() - 2)) 
	{
		point_1 = PathPoint_ReadPoint(i);
		point_2 = PathPoint_ReadPoint(i + 1);
		point_3 = PathPoint_ReadPoint(i + 2);
		//Usprintf("%s(%d):idx(%d)\n",__FUNCTION__, __LINE__,i);
		if((point_1.X == point_2.X) && (point_2.X == point_3.X))
		{
			PathPoint_RemoveOnePoint(i + 1);
		}
		else if((point_1.Y == point_2.Y) && (point_2.Y == point_3.Y))
		{
			PathPoint_RemoveOnePoint(i + 1);
		}
		else
		{
			i++;
		}
	}
	
}

/*spot dir
*/

void Spot_SetDir(SpotDir_t dir)
{
	g_spot_dir = dir;
	Usprintf("%s(%d):dir = %d\n",__FUNCTION__, __LINE__,g_spot_dir);
}
SpotDir_t Spot_GetDir(void)
{
	return g_spot_dir;
}
void Spot_ReverseDir(void)
{
	if(Spot_GetDir() == SPOT_DIR_CW)
	{
		Spot_SetDir(SPOT_DIR_CCW);
	}
	else
	{
		Spot_SetDir(SPOT_DIR_CW);
	}
}
void Spot_SetSpiralDir(SpotDir_t dir)
{
	 g_spot_outin = dir;
}
SpotDir_t Spot_GetSpiralDir(void)
{
	return g_spot_outin;
}
void Spot_SetClearFlag(uint8_t flag)
{
	g_spot_clear_flag = flag;
}
uint8_t Spot_GetClearFlag(void)
{
	return g_spot_clear_flag;
}

void Spot_SetLaneIdx(uint8_t idx)
{
	g_spot_lane_idx = idx;
}
uint8_t Spot_GetLaneIdx(void)
{
	return g_spot_lane_idx;
}


uint8_t Spot_OverLane(Point32_t start_cnt)
{
	uint8_t temp_idx = 0;
	Point32_t temp_robot_cnt;
	uint16_t angle = 0;
	
	temp_idx = Spot_GetLaneIdx();

	temp_robot_cnt = Map_GetRobotCount();
	
	angle = Math_Course2Dest(start_cnt.X,start_cnt.Y,temp_robot_cnt.X,temp_robot_cnt.Y);
	
	switch(Path_GetRobotHeading4(angle))
	{
		case EAST: 	if((temp_robot_cnt.X - start_cnt.X - 200) > temp_idx*CELL_COUNT_MUL)
								{
									return 1;
								}
								break;
		case NORTH: if((temp_robot_cnt.Y - start_cnt.Y - 200) > temp_idx*CELL_COUNT_MUL)
								{
									return 1;
								}
								break;
		case WEST:  if((start_cnt.X - temp_robot_cnt.X - 200) > temp_idx*CELL_COUNT_MUL)
								{
									return 1;
								}
								break;
		case SOUTH: if((start_cnt.Y - temp_robot_cnt.Y - 200) > temp_idx*CELL_COUNT_MUL)
								{
									return 1;
								}
								break;
		default:break;
		
	}
	
	return 0;
}
uint8_t Spot_InnerLane(uint8_t idx)   
{
	Point16_t temp_robot_cell;
	
	temp_robot_cell = Map_GetRobotCell();
	
	Usprintf("%s(%d):robot :(%d %d)  start idx :%d\n",__FUNCTION__, __LINE__,temp_robot_cell.X,temp_robot_cell.Y,idx);	
	
	if(((Math_Abs_int(temp_robot_cell.X) < idx))||((Math_Abs_int(temp_robot_cell.Y)) < idx))
	{
		Usprintf("%s(%d):Spot wall inner lane !\n",__FUNCTION__, __LINE__);			
		return 1;	
	}
	return 0;
}
void Spot_ClearCleanCells(int8_t idx)
{
	int16_t mx,my;
	Usprintf("%s(%d):\n", __FUNCTION__, __LINE__);
	for(mx = -idx; mx <= idx; mx++)
	{
		for(my = -idx; my <= idx; my++)
		{
			if(Map_GetCell(mx,my) == CLEANED)Map_SetCell(mx,my,UNCLEAN);
		}
	}
}

void Spot_ExpandBockedCells(void)
{
	int16_t mx,my;
	Usprintf("%s(%d)\n", __FUNCTION__, __LINE__);
	for(mx = (Map_GetXMin() - 1);mx <= Map_GetXMax();mx++)
	{
		for(my = (Map_GetYMin() - 1);my <= Map_GetYMax();my++)
		{
			if(Map_GetCell(mx,my) == BLOCKED_BUMPER)
			{
				if(Map_GetCell(mx - 1, my) < BLOCKED)Map_SetCell(mx - 1,my,BLOCKED_BOUNDARY);
				if(Map_GetCell(mx + 1, my) < BLOCKED)Map_SetCell(mx + 1,my,BLOCKED_BOUNDARY);
				if(Map_GetCell(mx, my + 1) < BLOCKED)Map_SetCell(mx,my + 1,BLOCKED_BOUNDARY);
				if(Map_GetCell(mx, my - 1) < BLOCKED)Map_SetCell(mx,my - 1,BLOCKED_BOUNDARY);
			}
		}
	}
}

void Spot_UpdateWallCell( Point16_t cell)
{
	Spot_Wall_Cells[g_spot_wall_idx] = cell;
	g_spot_wall_idx++;
}

void Spot_ResetWallCell(void)
{
	uint8_t i = 0;
	for(i = 0;i < g_spot_wall_idx; i++)
	{
		Spot_Wall_Cells[i] = (Point16_t){0,0};
	}
	g_spot_wall_idx = 0;
}

uint8_t Spot_CheckSameWallCell(Point16_t cell)
{
	uint8_t reval = 0;
	uint8_t i = 0;
	for(i = 0;i < g_spot_wall_idx-1; i++)
	{
		if((Spot_Wall_Cells[i].X == cell.X)&&(Spot_Wall_Cells[i].Y == cell.Y))
		{
		  reval = 1;
			break;		
		}		
	}
	return reval;
}

void Spot_CreateBoundary(uint8_t idx) 
{
  int8_t i = 0;
	/*east and west*/
  for( i = -idx; i < idx;i++)
	{ 
	  Map_SetCell(idx,i,BLOCKED_BOUNDARY);
		Map_SetCell(-idx,i,BLOCKED_BOUNDARY);		
	}
	/*north and south*/
  for( i = -idx; i < idx;i++)
	{ 
	  Map_SetCell(i,idx,BLOCKED_BOUNDARY);
		Map_SetCell(i,-idx,BLOCKED_BOUNDARY);		 		
	}		
}

SpotLaneDir_t Spot_GetPointLaneDir(Point16_t point)
{
	SpotLaneDir_t temp_spot_dir = SPOT_LANE_NONE;
	
	/*east or west*/
	if(Math_Abs_int(point.X) >= Math_Abs_int(point.Y))	
	{
		if(point.X >= 0)
		{
			temp_spot_dir = SPOT_LANE_EAST; 
		}
		else
		{
			temp_spot_dir = SPOT_LANE_WEST;
		}
	}
	/*south or north*/
	else
	{
		if(point.Y >= 0)
		{
			temp_spot_dir = SPOT_LANE_NORTH;
		}
		else
		{
			temp_spot_dir = SPOT_LANE_SOUTH;
		}			
	}
	
	return temp_spot_dir;
}

uint16_t Spot_GetPointLaneIdx(Point16_t point)  
{
  uint16_t 	temp_idx = 0;
	
	temp_idx = Math_GetMax(Math_Absolute(point.X),Math_Absolute(point.Y));

  return temp_idx;
}



