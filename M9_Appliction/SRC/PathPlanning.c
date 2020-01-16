/**
******************************************************************************
* @file        Shortest Path
* @author      Wfliu
* @version     Ver 20160118
* @date        18-Jan-2016
* @brief       Function to find the shorest path to target
******************************************************************************
* <h2><center>&copy; COPYRIGHT 2016 ILife CO.LTD</center></h2>
******************************************************************************
*/

/*
 * Path Planning for robot movement is a ZigZag way. When cleaning process starts,
 * robot will try to clean its left hand side first, a new lane must be cleaned
 * in both ends before getting the new cleaning target. Targets will be added to
 * the target list as soon as the robot is moving.
 *
 * After the cleaning process is done, robot will back to its starting point and
 * finishes the cleaning.
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



Pos_t Robot_Pos_Log[5];
Point16_t g_target_list[TARGET_LIST_AMOUNT]={0};
volatile uint8_t g_target_list_cnt = 0;
volatile Point16_t g_current_target;


volatile PathList_t g_pathlist[PATHLIST_CNT_LENGTH + 1] = {0};
volatile uint8_t g_pathlist_cnt = 0, g_pathlist_input_idx = 0, g_pathlist_output_idx = 0;
volatile Point32_t g_current_target_pos = {0};

/**calculate a path to unclean area from start point
*	@param	start_cell start cell of this path
*	@return		PATH_STATE_NOTARGET         no unclean area found
						PATH_STATE_NORMAL_CLEAN     path found 
						PATH_STATE_UNREACHABLE      unclean area found ,but no path found
*/
PathState_t Path_Next_V2(uint8_t mode,Point16_t start_cell)
{
	/*normal clean*/
	if(mode)
	{
		if(Path_NormalClean(start_cell,0) == 1)
		{
			Usprintf("%s(%d):Normal Clean !\n", __FUNCTION__, __LINE__);
			return PATH_STATE_NORMAL_CLEAN;
		}	
	}
	
  /*shortest path to clean*/
	if(TargetList_CreateList() == 0)
	{
		Usprintf("%s(%d):No Target Created !\n", __FUNCTION__, __LINE__);
		return PATH_STATE_NOTARGET;
	}
	/*target cell rearrange*/
	TargetList_ShowAll();
	TargetList_Rearrange();
	TargetList_ShowAll();
	PC_NavDebug_AllMap();
	/*calculate shortest path to the targets in the TargetList*/
	if(ShortestPath_GeneratePath(Map_GetRobotCell(),1))
	{
		PathPoints_Rearrange();
		PathPoint_AddPathPointToPathList(1);
		Path_NormalClean(PathPoint_ReadLastPoint(),1);		
		Usprintf("%s(%d):PathList Done\n", __FUNCTION__, __LINE__);
		return PATH_STATE_NORMAL_CLEAN;
	}
	/* no moveable points*/
  return PATH_STATE_UNREACHABLE;
}

/*"short_path_state" must be 1 after using the shortest path
	"short_path_state" should be 0 when normal clean 
*/
uint8_t Path_NormalClean(Point16_t start_cell,uint8_t short_path_state)
{
  uint8_t status = 0, cycle_breaker = 0;
  Point16_t next_pos  = start_cell;
	uint8_t first_lane_flag = 0;
	uint16_t robot_heading =  0;
	uint8_t roundable_flag = 0;
	PathState_t temp_state = PATH_STATE_NOTARGET;
  PathList_t temp_pathlist ;
	int16_t lane_dir = 2;
	Point16_t robot_pos = Map_GetRobotCell();
	static uint8_t robot_not_moving_cnt = 0;
	
	Usprintf("%s(%d):StartPos(%d£¬%d)\n", __FUNCTION__, __LINE__,start_cell.X,start_cell.Y);
	//lane_dir keeps robot always clean the lane near y = 0 lane
	//while robot the y+ side of the map , check lower lane first
	//while robot the y- side of the map , check higher lane first
	if(short_path_state)//short path
	{
		if(start_cell.Y > 0)
		{
			lane_dir = -2; 
		}
		else
		{
			lane_dir = 2; 
		}		
	}
	else//normalclean
	{
		if(robot_pos.Y > 0)
		{
			lane_dir = -2;
		}
		else
		{
			lane_dir = 2;
		}	
		PathList_Clear();
		Path_UpdateRobotPos();//store current robot pos to the robot pos log		

		if(Path_RobotNotMoving())//means last path plan path is not walkable , need to mark a new obs to map to prevent robot keep bumping
		{
			robot_not_moving_cnt++;
			if(robot_not_moving_cnt > 1)
			{
				robot_not_moving_cnt = 0;
				Usprintf("%s(%d): robot not moving!!!!!\n", __FUNCTION__, __LINE__);
				CM_SetBlockedByOffset(0,-CELL_SIZE*2);//set the cell infront of the robot blocked 
			}
		}
		else
		{
			robot_not_moving_cnt = 0;
		}	

		//two unclean cells beside the robot,turn to the cell near the y = 0 line 
		if(robot_pos.Y != 0)
		{
			if(Map_BrushBlockUnclean(next_pos.X,next_pos.Y + lane_dir) && Map_BrushBlockUnclean(next_pos.X,next_pos.Y - lane_dir))
			{
				Usprintf("%s(%d): move to last lane!\n", __FUNCTION__, __LINE__);
				next_pos.Y += lane_dir;
				temp_pathlist.cell_pos = next_pos;
				temp_pathlist.status = NORMAL_CLEAN;
				PathList_AddNewPoint(temp_pathlist);
				temp_state = PATH_STATE_NORMAL_CLEAN;						
			}					
		}				
	}
	
	status = Path_LaneCleaned(&next_pos);
	//2:this lane has two x to clean 
	//1: this lane has a x to clean 
	//0: this lane has been cleaned up
	
  Usprintf("%s(%d):Status = %d\n",__FUNCTION__, __LINE__,status);
  Usprintf("%s(%d):Next Pos (%d,%d)\n",__FUNCTION__, __LINE__,next_pos.X,next_pos.Y);
	if(status != 0)	//this lane still need to clean , move to next pos 
  {
		temp_pathlist.cell_pos = next_pos;
		temp_pathlist.status = NORMAL_CLEAN;
		PathList_AddNewPoint(temp_pathlist);
		robot_pos = temp_pathlist.cell_pos;
		first_lane_flag = 1;
    temp_state = PATH_STATE_NORMAL_CLEAN;
		if(status == 2)
		{
			if(next_pos.Y != 0)
			{
//				if(Path_LaneCleaned(&next_pos))
//				{
//					temp_pathlist.cell_pos = next_pos;
//					temp_pathlist.status = NORMAL_CLEAN;
//					PathList_AddNewPoint(temp_pathlist);
//					robot_pos = temp_pathlist.cell_pos;
//					temp_state = PATH_STATE_NORMAL_CLEAN;			
//				}					
				return 1;
			}
		}
  }
	else//this lane have cleaned up,check if need to wall follow back to check this lane
	{
		if(!short_path_state)
		{
			robot_heading = Path_GetRobotHeading8();

			if(Path_CheckRoundable(start_cell.X,start_cell.Y,start_cell.Y + 2,robot_heading))
			{
				roundable_flag = 1;
				temp_pathlist.cell_pos.X = start_cell.X;
				temp_pathlist.cell_pos.Y = start_cell.Y + 2;
			}
			else if(Path_CheckRoundable(start_cell.X,start_cell.Y,start_cell.Y - 2,robot_heading))
			{
				roundable_flag = 1;
				temp_pathlist.cell_pos.X = start_cell.X;
				temp_pathlist.cell_pos.Y = start_cell.Y - 2;
			}
			if(roundable_flag)
			{
				temp_pathlist.status = NORMAL_CLEAN;
				PathList_AddNewPoint(temp_pathlist);
				return 1;
			}				
		}
	}
	/*update the next pos*/
	if(first_lane_flag)
	{
		next_pos = temp_pathlist.cell_pos;
	}
	else
	{
		next_pos = start_cell;
	}
	/*add all the next moveable points*/
	if(Map_BrushBlockUnclean(next_pos.X,next_pos.Y + lane_dir))
	{
		next_pos.Y += lane_dir;
		temp_pathlist.cell_pos = next_pos;
		temp_pathlist.status = NORMAL_CLEAN;
		PathList_AddNewPoint(temp_pathlist);
		temp_state = PATH_STATE_NORMAL_CLEAN;
		cycle_breaker = 0;
		while(PathList_GetCnt() < PATHLIST_CNT_LENGTH)
		{
			next_pos =  temp_pathlist.cell_pos;
			status = Path_LaneCleaned(&next_pos);
			if(status != 0)
			{
				temp_pathlist.cell_pos = next_pos;
				temp_pathlist.status = NORMAL_CLEAN;
				PathList_AddNewPoint(temp_pathlist);
				temp_state = PATH_STATE_NORMAL_CLEAN;
				if(status == 2)
				{
					if(next_pos.Y != 0)
					{
						return 1;
//						if(Path_LaneCleaned(&next_pos))
//						{
//							temp_pathlist.cell_pos = next_pos;
//							temp_pathlist.status = NORMAL_CLEAN;
//							PathList_AddNewPoint(temp_pathlist);
//							robot_pos = temp_pathlist.cell_pos;
//							temp_state = PATH_STATE_NORMAL_CLEAN;			
//						}			
					}
				}
				if(Map_BrushBlockUnclean(next_pos.X,next_pos.Y + lane_dir))
				{
					next_pos.Y += lane_dir;
					temp_pathlist.cell_pos = next_pos;
					temp_pathlist.status = NORMAL_CLEAN;
					PathList_AddNewPoint(temp_pathlist);
					temp_state = PATH_STATE_NORMAL_CLEAN;
				}
				else
				{
					break;
				}
			}
			else
			{
				break;
			}
			cycle_breaker++;
			if(cycle_breaker >= PATHLIST_CNT_LENGTH)break;
		}
	}
	else if(Map_BrushBlockUnclean(next_pos.X,next_pos.Y - lane_dir))
	{
		next_pos.Y -= lane_dir;
		temp_pathlist.cell_pos = next_pos;
		temp_pathlist.status = NORMAL_CLEAN;
		PathList_AddNewPoint(temp_pathlist);
		temp_state = PATH_STATE_NORMAL_CLEAN;
		
		while(PathList_GetCnt() < PATHLIST_CNT_LENGTH)
		{
			next_pos =  temp_pathlist.cell_pos;
			status = Path_LaneCleaned(&next_pos);
			if(status != 0)
			{
				temp_pathlist.cell_pos = next_pos;
				temp_pathlist.status = NORMAL_CLEAN;
				PathList_AddNewPoint(temp_pathlist);
				temp_state = PATH_STATE_NORMAL_CLEAN;

				if(status == 2)
				{
					if(next_pos.Y != 0)
					{
						return 1;
//						if(Path_LaneCleaned(&next_pos))
//						{
//							temp_pathlist.cell_pos = next_pos;
//							temp_pathlist.status = NORMAL_CLEAN;
//							PathList_AddNewPoint(temp_pathlist);
//							robot_pos = temp_pathlist.cell_pos;
//							temp_state = PATH_STATE_NORMAL_CLEAN;			
//						}			
					}
				}
				
				if(Map_BrushBlockUnclean(next_pos.X,next_pos.Y - lane_dir))
				{
					next_pos.Y -= lane_dir;
					temp_pathlist.cell_pos = next_pos;
					temp_pathlist.status = NORMAL_CLEAN;
					PathList_AddNewPoint(temp_pathlist);
					temp_state = PATH_STATE_NORMAL_CLEAN;
				}
				else
				{
					break;
				}
			}
			else
			{
				break;
			}
		}
	}

	
	if(temp_state == PATH_STATE_NORMAL_CLEAN)return 1;

	return 0;
}







PathState_t Path_WallFineWay(Point16_t start_cell)
{
	Usprintf("%s(%d):start to find path\n", __FUNCTION__, __LINE__);
	if(TargetList_CreateList() == 0)
	{
		Usprintf("%s(%d):No Target Created  \n", __FUNCTION__, __LINE__);
		return PATH_STATE_NOTARGET;
	}
	/*calculate shortest path to the targets in the TargetList*/
	if(ShortestPath_GeneratePath(Map_GetRobotCell(),1))
	{
		PathPoints_Rearrange();
		PathPoint_AddPathPointToPathList(1);
		Path_NormalClean(PathPoint_ReadLastPoint(),1);
		Usprintf("%s(%d):PathList Done\n", __FUNCTION__, __LINE__);
		return PATH_STATE_NORMAL_CLEAN;
	}
	/* no moveable points*/
  return PATH_STATE_UNREACHABLE;
}
/**calculate a certain lane on the map to check if this lane have been cleaned up
*	@param	*p_land_pos  a lane for calculation , and parse back the uncleand end cell of this lane
*	@return	 1 this lane do have a uncleand  end 
*					 0 this lane have been clean up ,robot should move to other lane
*					 2 this lane do have two uncleand  end 
*/
uint8_t Path_LaneCleaned(Point16_t *p_lane_pos)
{
  uint8_t x_offset = 0;
  uint8_t lane_min = 0, lane_max = 0;
  int16_t temp_x = p_lane_pos->X;
  int16_t temp_y = p_lane_pos->Y;
  Usprintf("%s(%d):Lane Pos(%d,%d) \n", __FUNCTION__, __LINE__,temp_x,temp_y);
  while((Mode_GetMode() != MODE_USERINTERFACE))//find min x max x on same lane
  {
    x_offset++; 		
		//find the min x unclean block on the same lane
		temp_x = p_lane_pos->X - x_offset;
		if(temp_x <= MAP_LIMIT_LOW)
		{
			Usprintf("L touch the lane low limit,break   x:%d x_offset:%d\n",temp_x,x_offset);
			break;//touch Map limit
		}
		else
		{
			if(Map_BrushBlockUnclean(temp_x,temp_y))
			{
//				Usprintf("L lane unclean cell:(%d ,%d)\n",temp_x,temp_y);
				if(Map_BrushBlockUnclean(temp_x + 1, temp_y))lane_min = x_offset;
			}
		}
		if(Map_Cell_Blocked(temp_x, temp_y))
		{
			Usprintf("L lane block cell:(%d ,%d)\n",temp_x,temp_y);
			break;
		}		
  }	
	
	x_offset = 0;
	
	
  while((Mode_GetMode() != MODE_USERINTERFACE))//find min x max x on same lane
  {
    x_offset++; 		
		//find the min x unclean block on the same lane
		temp_x = p_lane_pos->X + x_offset;
		if(temp_x >= MAP_LIMIT_HIGH)
		{
			Usprintf("H touch the lane high limit,break   x:%d x_offset:%d\n",temp_x,x_offset);			
			break;//touch Map limit
		}
		else
		{
			if(Map_BrushBlockUnclean(temp_x,temp_y))
			{
//				Usprintf("H lane unclean cell:(%d ,%d)\n",temp_x,temp_y);
				if(Map_BrushBlockUnclean(temp_x - 1, temp_y))lane_max = x_offset;
			}
		}
		if(Map_Cell_Blocked(temp_x, temp_y))
		{
			Usprintf("H lane block cell:(%d ,%d)\n",temp_x,temp_y);
			break;
		}		
  }
  if(lane_min > 0)lane_min -= 1;//robot occupy 3x3,so need 1 cell space
	if(lane_max > 0)lane_max -= 1;//robot occupy 3x3,so need 1 cell space

  Usprintf("%s(%d):path_Lane Lane_Min(%d) Lane_Max(%d)\n",__FUNCTION__, __LINE__,lane_min,lane_max);
  Usprintf("%s(%d):path_Lane X Offset(%d) Robot Direction(%d)\n",__FUNCTION__, __LINE__,x_offset,Path_GetRobotDirection());

  if((lane_min == 0)&&(lane_max == 0))//this lane has been cleaned up
  {
    return 0;
  }
  if((lane_min != 0) && (lane_max != 0))
	{
		if(p_lane_pos->Y != 0)
		{
			if(lane_min > lane_max)
			{			
				p_lane_pos->X += lane_max;	
			}
			else
			{
				p_lane_pos->X -= lane_min;	
			}
			return 2;			
		}
	}
	
	
  if(Path_GetRobotDirection() == WEST)
  {
    if(lane_min != 0)
    {
      p_lane_pos->X -= lane_min;
      return 1;
    }
    if(lane_max != 0)
    {
      p_lane_pos->X += lane_max;
      return 1;
    }
  }
  else
  {
    if(lane_max != 0)
    {
      p_lane_pos->X += lane_max;
      return 1;
    }
    if(lane_min != 0)
    {
      p_lane_pos->X -= lane_min;
      return 1;
    }
  }

  return 0;
}
/*path list*/
/**add new point to the pathlist
*	@param	point new point to add to path list
*/
void PathList_AddNewPoint(PathList_t point)
{		
	if(Map_TwoCellMatch(point.cell_pos,Map_GetRobotCell()))return; // no need to add a point that robot current in to PathList 
	if(g_pathlist_input_idx > 0)
	{
		// no need to add a point that existed
		if(Map_TwoCellMatch(point.cell_pos, g_pathlist[g_pathlist_input_idx - 1].cell_pos))return; 
	}
	if(g_pathlist_cnt > PATHLIST_CNT_LENGTH)
	{
		Usprintf("%s(%d):Path List Full!! \n",__FUNCTION__, __LINE__);
		return;
	}
	
	g_pathlist[g_pathlist_input_idx] = point;
	g_pathlist_input_idx++;
	if(g_pathlist_input_idx > PATHLIST_CNT_LENGTH)g_pathlist_input_idx = 0;
	g_pathlist_cnt++;
	
	Usprintf("%s(%d):(%d,%d) S=%d PathList_Cnt= %d \n",__FUNCTION__, __LINE__,
						point.cell_pos.X, point.cell_pos.Y, point.status, g_pathlist_cnt);

	PC_NavDebug(point.cell_pos.X, point.cell_pos.Y, 0, 9);
}
/**get out the newest point of the pathlist
*	@param	*p_out return the newest point
* @return 1 get point 
*         0 no path point return
*/
uint8_t PathList_Out(PathList_t *p_out)
{
	if(g_pathlist_cnt == 0)
	{
		Usprintf("%s(%d):No Path Out!!\n",__FUNCTION__, __LINE__);
		return 0;//no path to go
	}
	*p_out = g_pathlist[g_pathlist_output_idx];
	Usprintf("%s(%d):Path(%d,%d) idx = %d  !!\n",__FUNCTION__, __LINE__,
		g_pathlist[g_pathlist_output_idx].cell_pos.X,g_pathlist[g_pathlist_output_idx].cell_pos.Y,g_pathlist_output_idx);
	g_pathlist_output_idx++;
	if(g_pathlist_output_idx > PATHLIST_CNT_LENGTH)g_pathlist_output_idx = 0;
	if(g_pathlist_cnt > 0)g_pathlist_cnt--;
	return 1;//new path has return 
}
uint8_t PathList_GetNextPathListPoint(Point32_t *point_result)
{
	if(g_pathlist_cnt == 0)
	{
		Usprintf("%s(%d):No Path Out!!\n",__FUNCTION__, __LINE__);
		return 0;//no path to go
	}
	*point_result = Map_CellToPoint(g_pathlist[g_pathlist_output_idx].cell_pos);
	Usprintf("%s(%d):Path(%d,%d) idx = %d  !!\n",__FUNCTION__, __LINE__,
		g_pathlist[g_pathlist_output_idx].cell_pos.X,g_pathlist[g_pathlist_output_idx].cell_pos.Y,g_pathlist_output_idx);
	g_pathlist_output_idx++;
	if(g_pathlist_output_idx > PATHLIST_CNT_LENGTH)g_pathlist_output_idx = 0;
	if(g_pathlist_cnt > 0)g_pathlist_cnt--;
	return 1;//new path has return 
}
PathList_t PathList_ReadPath(uint8_t idx)
{
	if(idx < PATHLIST_CNT_LENGTH)
	{
		return g_pathlist[idx];
	}
	else
	{
		return g_pathlist[0];
	}
}
PathList_t PathList_ReadLastPath(void)
{
	if(g_pathlist_input_idx == 0)return g_pathlist[PATHLIST_CNT_LENGTH];
	return g_pathlist[(g_pathlist_input_idx - 1)];
}
void PathList_DeleteOne(uint8_t idx)
{
	uint8_t i=0;
	if(g_pathlist_cnt<idx)return;
	Usprintf("%s(%d):DETELE^^^^^^^^^^^^^^^^^^^^^^^^^^^^:(%d,%d)!!\n",__FUNCTION__, __LINE__,g_pathlist[idx].cell_pos.X,g_pathlist[idx].cell_pos.Y);
	for(i = idx; i < (PathList_GetCnt()-1); i++)
	{
		g_pathlist[i] = g_pathlist[i + 1];
	}
	g_pathlist_cnt--;
}
void PathList_Change(uint8_t id1,uint8_t id2)
{
	Point16_t point_temp;
	point_temp = g_pathlist[id1].cell_pos;
	g_pathlist[id1].cell_pos = g_pathlist[id2].cell_pos;
	g_pathlist[id2].cell_pos = point_temp;
}
void PathList_ShowAll(void)
{
	#ifdef MY_D
	uint8_t idx = 0;
	for(idx = 0; idx < 6; idx ++)//g_pathlist_cnt
	{
		printf2("%s(%d):idx(%d)(%d,%d) State = %d\n",__FUNCTION__, __LINE__,idx,
			g_pathlist[idx].cell_pos.X,g_pathlist[idx].cell_pos.Y,g_pathlist[idx].status);
	}
	#endif
}

void PathList_Clear(void)
{
	#ifdef ENABLE_DEBUG_TARGET
	while(g_pathlist_cnt)
	{
		PC_NavDebug(g_pathlist[g_pathlist_cnt].cell_pos.X, g_pathlist[g_pathlist_cnt].cell_pos.Y, 0, 1);
		osDelay(5);
		Usprintf("%s(%d):(%d,%d) State=%d PathList_Cnt= %d \n",__FUNCTION__, __LINE__,
						g_pathlist[g_pathlist_cnt].cell_pos.X, g_pathlist[g_pathlist_cnt].cell_pos.Y, g_pathlist[g_pathlist_cnt].status, g_pathlist_cnt);
		osDelay(5);
		g_pathlist_cnt--;
	}
	#endif
	g_pathlist_cnt = 0;
	g_pathlist_input_idx = 0;
	g_pathlist_output_idx = 0;
}
uint8_t PathList_GetCnt(void)
{
	return g_pathlist_cnt;
}
void Path_SetCurrentTargetPoint(Point32_t P)
{
	g_current_target_pos = P;
}
Point32_t Path_GetCurrentTargetPoint(void)
{
	return g_current_target_pos;
}
/*target list*/
void TargetList_ShowAll(void) 
{
	#ifdef ENABLE_DEBUG_TARGET
	uint8_t i=0;
	Point16_t temp_target;
	for(i = 0; i < TargetList_ReadCnt(); i++)
	{
		temp_target = TargetList_ReadTarget(i);
		Usprintf("%s(%d):Target(%d)=(%d,%d) \n",__FUNCTION__, __LINE__, i, temp_target.X, temp_target.Y);
	}
	#endif
}
uint8_t TargetList_CreateList(void)
{
	int16_t x = 0,y = 0;
	uint8_t ret_val = 0;
	TargetList_Clear();
	Usprintf("%s(%d):g_y_min = %d g_y_max = %d g_x_min = %d g_x_max = %d\n", __FUNCTION__, __LINE__,g_y_min,g_y_max,g_x_min,g_x_max);
	for(y = (g_y_min - 1); y <= g_y_max; y++)
	{
		for(x = (g_x_min - 1); x <= g_x_max; x++)
		{
			if(Map_GetCell(x,y) == CLEANED)
			{
				if(TargetList_IsBlockTargetable(x, y + 1))//unclean
				{
					TargetList_AddTarget(x, y + 1);
					ret_val = 1;
				}
				if(TargetList_IsBlockTargetable(x,y - 1))//unclean
				{
					TargetList_AddTarget(x, y - 1);
					ret_val = 1;
				}

			}
		}
	}
	Usprintf("%s(%d):Rt_Val = %d \n", __FUNCTION__, __LINE__,ret_val);
	return ret_val;
}
uint8_t TargetList_ReadCnt(void)
{
	return g_target_list_cnt;
}
void TargetList_AddTarget(int16_t x,int16_t y)
{
	if(g_target_list_cnt >= TARGET_LIST_AMOUNT)
	{
		Usprintf("%s(%d):too many Target add %d\n", __FUNCTION__, __LINE__, g_target_list_cnt);
		return ;
	}
	g_target_list[g_target_list_cnt].X = x;
	g_target_list[g_target_list_cnt].Y = y;
	//Usprintf("%s(%d):g_target_list[%d](%d,%d) \n", __FUNCTION__, __LINE__, g_target_list_cnt, x, y);
	g_target_list_cnt++;
}
/*
fetch a target from TargetList by given index
@param uint8_t Idx: index of the target needed to read out
return Point16_t target cell pos(x,y)
*/
Point16_t TargetList_ReadTarget(uint8_t idx)
{
	if(idx < TargetList_ReadCnt())
	{
		return g_target_list[idx];
	}
	Usprintf("\n\r idx out range", __FUNCTION__, __LINE__);
	return (Point16_t){0,0};
}
/*delete a target from TargetList by given index
* @ param uint8_t Idx: index of the target needed to delete
*/
void TargetList_DeleteTarget(uint8_t idx)
{
	uint8_t i=0;
//	Point16_t temp_target = g_target_list[idx];
	//Usprintf("%s(%d):TargetList[%d] (%d,%d) \n",__FUNCTION__, __LINE__,idx,temp_target.X,temp_target.Y);
	for(i = idx; i < (TargetList_ReadCnt()-1); i++)
	{
		g_target_list[i] = g_target_list[i + 1];
	}
	g_target_list_cnt--;
}   
/*clear the target list
*/
void TargetList_Clear(void)
{
	g_target_list_cnt = 0;
}
/*rearrange the target list
*leave the head and tail target and deletes the targets between head and tail
*which means one continue lane only have head and tail for target
*/
void TargetList_Rearrange(void)
{
	uint16_t break_cnt = 10000;
	uint8_t same_line_flag = 0;
	uint8_t i = 0,j = 0;
	int16_t x = 0;
	Point16_t target_1,target_2;
	Usprintf("%s(%d):target rearrange! \n", __FUNCTION__, __LINE__);
	for(i = 0; i < (TargetList_ReadCnt() - 1);i++)
	{
		target_1 = TargetList_ReadTarget(i);
		j = i + 1;
		while(j < TargetList_ReadCnt())
		{
			same_line_flag = 1;
			if(target_1.Y == TargetList_ReadTarget(j).Y)//two target points have same y , need to check if on same line
			{
				target_2 = TargetList_ReadTarget(j);

				for(x = target_1.X; x < target_2.X; x++)
				{
					if(Map_Cell_Blocked(x, target_1.Y))break;
				}
				if(x == (target_2.X))
				{
					if(Math_Diff_int(target_1.X,Map_GetRobotCell().X) < Math_Diff_int(target_2.X,Map_GetRobotCell().X))
					{
						TargetList_DeleteTarget(j);//target2					
					}
					else
					{
						TargetList_DeleteTarget(i);//target1
					}
					same_line_flag=0;
				}
			}
			if(same_line_flag)j++;
			break_cnt--;
			if(break_cnt == 0)break;
		}
	}
}
void TargetList_StoreCurrentTarget(Point16_t target)
{
	g_current_target = target;
}
Point16_t TargetList_GetCurrentTarget(void)
{
	return g_current_target;
}
/*find the target which on the most north position
* returns a northest target on the target list
*/
Point16_t TargetList_GetGreatestYTarget(void)
{
	Point16_t temp_taget;
	uint8_t i = 0;
	Usprintf("%s(%d): \n", __FUNCTION__, __LINE__);
	temp_taget = TargetList_ReadTarget(0);
	for(i = 0; i < TargetList_ReadCnt(); i++)
	{
		if(temp_taget.Y < TargetList_ReadTarget(i).Y)
		{
			temp_taget = TargetList_ReadTarget(i);
		}
	}
	return temp_taget;
}
/*check if the given cell(x,y) is targetable
* a unclean cell whick nears a cleaned cell is a potential target cell 
* if all 8 cells around this potential target cell are not a blocked cell
* then this cell is a targetable cell, should add to the target list
*/
uint8_t TargetList_IsBlockTargetable(int16_t x,int16_t y)
{
	int8_t val = 0, i = 1, j = 1;
	CellState_t temp_cell = UNCLEAN;
	if(Map_GetCell(x,y) != UNCLEAN)return 0;
	for(i = 1;i >- 2;i--)
	{
		for(j = 1; j >- 2;j--)
		{
			temp_cell = Map_GetCell(x + i, y + j);
			if(temp_cell == UNCLEAN)val++;
			else if(temp_cell >= BLOCKED)
			{
				return 0;
			}
		}
	}
//	if(Map_GetCell(MAP,x,y)!=UNCLEAN)return 0;
//	if(Map_GetCell(MAP,x+1,y)==UNCLEAN)Val++;
//	if(Map_GetCell(MAP,x-1,y)==UNCLEAN)Val++;
//	if(Map_GetCell(MAP,x,y+1)==UNCLEAN)Val++;
//	if(Map_GetCell(MAP,x,y-1)==UNCLEAN)Val++;
	if(val > 0)return 1;
	return 0;					
}
/*check two position count if so near
* this function is use for checking if robot is getting to the target position 
*/
uint8_t Path_IsOnPosition(Point32_t pos_a,Point32_t pos_b)
{
	if (Math_Diff_int(pos_a.X, pos_b.X) < 150 && Math_Diff_int(pos_a.Y, pos_b.Y) < 150)return 1;//150
	return 0;
}
/*check if a given count's y is at a given lane
* this function is use for checking if robot is at a lane
*/
uint8_t Path_RobotOnLane(int32_t lane_y,int32_t target_y)
{
  if(Math_Diff_int(target_y,lane_y) < 80)return 1;
  return 0;
}
/*check if a given count's y if far enough from a given lane
* this function is for checking if robot is moving over to the current lane 
* if robot move 1.5 cell far from the given lane , means robot if over this lane
*/
uint8_t Path_OverLane(int32_t lane_y,int32_t target_y)
{
  if(Math_Diff_int(lane_y,target_y) > CELL_COUNT_MUL_3_2)return 1;
  return 0;
}
/*check if robot is near a position by counts (400)
*/
uint8_t Path_RobotNearPosCount(Point32_t target_count)//for deceleration
{
	Point32_t temp_robot_count = Map_GetRobotCount();
	if ((Math_Diff_int(temp_robot_count.X,target_count.X) < CELL_COUNT_MUL) 
		&& (Math_Diff_int(temp_robot_count.Y,target_count.Y) < CELL_COUNT_MUL))
	{
		return 1;
	}
	return 0;
}
/*check if robot is near a position by 1.5 cell distance
*/
uint8_t Path_RobotCloseToTargetCount(Point32_t target_cnt)//for deceleration
{
	Point32_t temp_robot_cnt = Map_GetRobotCount();
	if ((Math_Diff_int(temp_robot_cnt.X,target_cnt.X) <= (CELL_COUNT_MUL))//CELL_COUNT_MUL_3_2//CELL_COUNT_MUL_7_4
		&& (Math_Diff_int(temp_robot_cnt.Y,target_cnt.Y) <= (CELL_COUNT_MUL)))//CELL_COUNT_MUL_3_2//CELL_COUNT_MUL_7_4
	{
		return 1;        
	}
	return 0;
}
/*check if robot is near a cell 
* distance between robot and the given cell if less than 3 cell 
*/
uint8_t Path_RobotCloseToTargetCell(Point16_t target_cell,uint8_t distance)//for deceleration
{
	Point16_t temp_robot_cell = Map_GetRobotCell();
	if ((Math_Diff_int(temp_robot_cell.X,target_cell.X) < distance) 
		&& (Math_Diff_int(temp_robot_cell.Y,target_cell.Y) < distance))
	{
		return 1;
	}
	return 0;
}
uint8_t Path_RobotLeaveToTargetCell(Point16_t target_cell,uint8_t distance)//for deceleration
{
	Point16_t temp_robot_cell = Map_GetRobotCell();
	if ((Math_Diff_int(temp_robot_cell.X,target_cell.X) > distance) 
		&& (Math_Diff_int(temp_robot_cell.Y,target_cell.Y) > distance))
	{
		return 1;
	}
	return 0;
}
/*get robot directions
* first try to get robot heading by gyro of +/- 5 degree for the ENWS direction
* if robot is not heading to those 4 directons , try to get robot drection by a +/- 45 degree to ENWS directoins
*/
uint16_t Path_GetRobotDirection()
{
	uint16_t	dir;
//	Point16_t temp_pos;

	dir = (uint16_t) round(((double)Gyro_GetAngle(0)) / 100);
	switch(dir) 
	{
		case 0://-50-49=0 = X+
		case 36://(3550-3649) = X+
//			return EAST;//0
			dir = (uint16_t)EAST;
			break;
		case 9://(850-949)=9 = Y+
//			return NORTH;//900
			dir = (uint16_t)NORTH;
			break;
		case 18://(1750-1849)=18 = X-
//			return WEST;//1800
			dir = (uint16_t)WEST;
			break;
		case 27://(2650-2749)=27 = Y-
//			return SOUTH;//2700
			dir = (uint16_t)SOUTH;
			break;
		default:
			/*
			 * Failed to use the Gyro value, use the robot displacement to find the direction.
			 * Only handle NORTH & SOUTH direction.
			 */
//			temp_pos = Path_GetPreviousRobotPos().pos;
//			if(temp_pos.Y == Map_GetRobotCellY())
//			{
//				if(Map_GetRobotCellX() > temp_pos.X)
//				{
//					return EAST;
//				}
//				if(Map_GetRobotCellX() < temp_pos.X)
//				{
//					return WEST;
//				}
//			}
			
			dir = (uint16_t)Path_GetRobotHeading4(Gyro_GetAngle(0));

		
		
			break;
	}
	return dir;
}
/*
	returns robot's heading in 4 directions(one direction = 900)
*/
uint16_t Path_GetRobotHeading4(uint16_t angle)
{
	if((angle <= 450) || (angle > 3150))
	{
		return EAST;
	}
	if((angle > 450) && (angle <= 1350))
	{
		return NORTH;
	}
	if((angle > 1350) && (angle <= 2250))
	{
		return WEST;
	}
	if((angle > 2250) && (angle <= 3150))
	{
		return SOUTH;
	}
	return EAST;
}
/*
	returns robot's heading in 8 directions(one direction = 450)
*/
uint16_t Path_GetRobotHeading8(void)
{
	if((Gyro_GetAngle(0) > 3375) || (Gyro_GetAngle(0) <= 225))
	{
		return EAST;
	}
	if((Gyro_GetAngle(0) > 225) && (Gyro_GetAngle(0) <= 675))
	{
		return NORTH_EAST;
	}
	if((Gyro_GetAngle(0) > 675) && (Gyro_GetAngle(0) <= 1125))
	{
		return NORTH;
	}
	if((Gyro_GetAngle(0) > 1125) && (Gyro_GetAngle(0) <= 1575))
	{
		return NORTH_WEST;
	}
	if((Gyro_GetAngle(0) > 1575) && (Gyro_GetAngle(0) <= 2025))
	{
		return WEST;
	}
	if((Gyro_GetAngle(0) > 2025) && (Gyro_GetAngle(0) <= 2475))
	{
		return SOUTH_WEST;
	}
	if((Gyro_GetAngle(0) > 2475) && (Gyro_GetAngle(0) <= 2925))
	{
		return SOUTH;
	}
	if((Gyro_GetAngle(0) > 2925) && (Gyro_GetAngle(0) <= 3375))
	{
		return SOUTH_EAST;
	}
	return 0;
}
/*get robot normal cleaning wall follow directions
*/
WallDir_t Path_GetNormalWallDir(Point16_t target_cell)
{
	WallDir_t ret_val = WALLDIR_NONE;
	//if(Path_GetRobotHeading4()==EAST)//robot heading about +/-45 degreee to EAST
	if((Gyro_GetAngle(0) < 900)|| (Gyro_GetAngle(0) > 2700))
	{
		if(target_cell.Y>Map_GetRobotCellY())
		{
			ret_val = WALLDIR_EAST_RIGHT;
		}
		else
		{
			ret_val = WALLDIR_EAST_LEFT;
		}
	}
	else 
	{
		if(target_cell.Y>Map_GetRobotCellY())
		{
			ret_val = WALLDIR_WEST_LEFT;
		}
		else
		{
			ret_val = WALLDIR_WEST_RIGHT;
		}
	}
	return ret_val;
}       
/*when robot running at escaping wall follow mode 
* robot should aways follow the right wall
* so should calculate which directions of the map robot now right wall following
*/
WallDir_t Path_GetShortWallDir(Point16_t target_cell,Point16_t robot_cell) //bug!
{
  uint16_t temp_robot_heading;
	WallDir_t ret_val = WALLDIR_NONE;
	temp_robot_heading = Path_GetRobotHeading8();
	if( temp_robot_heading == EAST)return WALLDIR_NONE;
	if( temp_robot_heading == WEST)return WALLDIR_NONE;
	if(target_cell.Y > robot_cell.Y)
	{
		//if((temp_robot_heading == NORTH) && 
		if(Map_GetCell(robot_cell.X, robot_cell.Y + 2) == BLOCKED_BUMPER)
		{
			if(target_cell.X > robot_cell.X)
			{
				if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == NORTH)ret_val = WALLDIR_WEST_LEFT;
			}
			else
			{
				if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == NORTH)ret_val = WALLDIR_EAST_RIGHT;
			}
		}
	}
	else
	{
		//if((temp_robot_heading == SOUTH) && 
		if((Map_GetCell(robot_cell.X, robot_cell.Y - 2) == BLOCKED_BUMPER))
		{
			if(target_cell.X > robot_cell.X)
			{
				if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == SOUTH)ret_val = WALLDIR_WEST_RIGHT;
			}
			else
			{
				if(Path_GetRobotHeading4(Gyro_GetAngle(0)) == SOUTH)ret_val = WALLDIR_EAST_LEFT;
			}
		}
	}
	return ret_val;
}
/* update robot's position cell to the log when robot entering the path planning
*/
void Path_UpdateRobotPos(void)
{
	uint8_t i=0;
	for(i = 0; i < 4; i++)
	{
		Robot_Pos_Log[i] = Robot_Pos_Log[i+1];
	}
	Robot_Pos_Log[4].pos = Map_GetRobotCell();
	Robot_Pos_Log[4].dir = Path_GetRobotHeading8();
}
/*get the robot pos cell when robot last entered path planning
*/
Pos_t Path_GetPreviousRobotPos(void)
{
	return Robot_Pos_Log[3];
}
/*if last time robot enter path planning and now robot entering path planning are at the same cell
* means robot not moving any where
*/
volatile uint8_t g_reset_robot_not_moving = 0;
uint8_t Path_RobotNotMoving(void)
{	
	if(g_reset_robot_not_moving == 1)
	{
		g_reset_robot_not_moving = 0;
		return 0;
	}
	if(Robot_Pos_Log[4].pos.X	!=	Robot_Pos_Log[3].pos.X)return 0;
	if(Robot_Pos_Log[4].pos.Y	!=	Robot_Pos_Log[3].pos.Y)return 0;
	if(Robot_Pos_Log[4].dir	!=	Robot_Pos_Log[3].dir)return 0;
	return 1;
}


uint8_t Path_CheckRoundable(int16_t start_x,int16_t start_y,int16_t target_y,uint16_t heading)
{
	int16_t i = 0;
	uint8_t sum = 0;
	if(heading == EAST)
	{
		for(i = start_x; i < Map_GetXMax(); i++)
		{
			if((Map_GetCell(i,target_y) == CLEANED) && (Map_GetCell(i,start_y) == UNCLEAN))sum++;
			if(Map_GetCell(i,target_y) > CLEANED)return 0;
			if(sum > 1)return 1;
			if((i - start_x)>10)return 0;
		}
	}
	else if(heading == WEST)
	{
		for(i = start_x; i > Map_GetXMin(); i--)
		{
			if((Map_GetCell(i,target_y) == CLEANED) && (Map_GetCell(i,start_y) == UNCLEAN))sum++;
			if(Map_GetCell(i,target_y) > CLEANED)return 0;
			if(sum > 1)return 1;
			if((start_x - i )>10)return 0;
		}
	}
	return 0;
}

/*block all target points */

void Path_BlockAllTargetPoints(void)
{
	int16_t x = 0,y = 0;
	TargetList_Clear();
	Usprintf("%s(%d):g_y_min = %d g_y_max = %d\n", __FUNCTION__, __LINE__,g_y_min,g_y_max);
	for(y = (g_y_min); y <= g_y_max; y++)
	{
		for(x = (g_x_min); x <= g_x_max; x++)
		{
			if(Map_GetCell(x,y) == CLEANED)
			{
				Map_SetBlockIfUnclean(MAP,x+1,y,BLOCKED);
				Map_SetBlockIfUnclean(MAP,x-1,y,BLOCKED);
				Map_SetBlockIfUnclean(MAP,x,y+1,BLOCKED);
				Map_SetBlockIfUnclean(MAP,x,y-1,BLOCKED);
			}
		}
	}
	for(y = (g_y_min); y <= g_y_max; y++)
	{
		for(x = (g_x_min); x <= g_x_max; x++)
		{
			if(Map_GetCell(x,y) == BLOCKED)
			{
				Map_SetBlockIfUnclean(MAP,x+1,y,BLOCKED_BOUNDARY);
				Map_SetBlockIfUnclean(MAP,x-1,y,BLOCKED_BOUNDARY);
				Map_SetBlockIfUnclean(MAP,x,y+1,BLOCKED_BOUNDARY);
				Map_SetBlockIfUnclean(MAP,x,y-1,BLOCKED_BOUNDARY);
			}
		}
	}
}
void Path_SetHomeCellEmpty(void)
{
	int8_t i,j;
	Point16_t home = Map_GetHomeCell();
	Usprintf("Home1(%d,%d) Heading(%d)\n",home.X,home.Y);	
	
	for(i=-2;i<3;i++)//i=-4;i<5;i++
	{
		for(j=-2;j<3;j++)//j=-4;j<5;j++
		{
			Map_SetCell(home.X+i,home.Y+j,CLEANED);			
		}
	}
	Map_SetCell(home.X,home.Y,UNCLEAN);//set home point unclean
	Map_SetCell(home.X - 1,home.Y,UNCLEAN);//set home point unclean
	Map_SetCell(home.X + 1,home.Y,UNCLEAN);//set home point unclean.
	Map_SetCell(home.X,home.Y - 1,UNCLEAN);//set home point unclean
	Map_SetCell(home.X,home.Y + 1,UNCLEAN);//set home point unclean	
	Speaker(SPK_ROBOT_TO_START_POINT);
}
volatile uint8_t path_nowayhome_flag = 0;
void Path_SetNoWayHome(uint8_t pathway)
{
	path_nowayhome_flag = pathway;
}
uint8_t Path_IsNoWayHome(void)
{
   if(path_nowayhome_flag == 1)
	 {
		 return 1;
	 }
	 return 0;
}

DirectionCardinal_t Path_GetCellHeading8(Point16_t start_cell,Point16_t dest_cell)
{
		int16_t angle = 0;
	angle = Math_Course2Dest(start_cell.X,start_cell.Y,dest_cell.X,dest_cell.Y);
	
	if((angle > 3375) || (angle <= 225))
	{
		return EAST;
	}
	else if((angle > 225) && (angle <= 675))
	{
		return NORTH_EAST;
	}
	else if((angle > 675) && (angle <= 1125))
	{
		return NORTH;
	}
	else if((angle > 1125) && (angle <= 1575))
	{
		return NORTH_WEST;
	}
	else if((angle > 1575) && (angle <= 2025))
	{
		return WEST;
	}
	else if((angle > 2025) && (angle <= 2475))
	{
		return SOUTH_WEST;
	}
	else if((angle > 2475) && (angle <= 2925))
	{
		return SOUTH;
	}
	else 
	{
		return SOUTH_EAST;
	}
}


