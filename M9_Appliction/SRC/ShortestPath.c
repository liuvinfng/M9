/**
 ******************************************************************************
 * @file	Shortest Path
 * @author	Wfliu
 * @version	Ver 20160118
 * @date	18-Jan-2016
 * @brief	Function to find the shorest path to target
 ******************************************************************************
 * <h2><center>&copy; COPYRIGHT 2016 ILife CO.LTD</center></h2>
 ******************************************************************************
 */

/*
 * Functions in this file are for determining the path from the current robot postion
 * to a give target point. There are 2 version of shortest path method is include.
 *
 * In the first version, it use the A-Star like algorithm, starting from the current
 * robot position point, assign a value to the points that have an offset 1, which
 * coordinate is (x -/+ offset, y -/+ offset), then increase the offset by 1, until the
 * cell for the target point is set or there is not more value to set in the map. If the
 * cell for the target point is set, trace back the path to the robot position. This
 * method is a little bit time consuming, since the worst case would be O(3). And
 * optimization is limited.
 *
 * In the second version, it use the up-side-down tree method, the target point is
 * set to the root of the tree. Start for the target point position, construct the tree
 * by adding the line segment(a vertical line segment that without any obstcal from the map)
 * as the node that the connected to each other is the map. When the line segment includes
 * the current robot position is added into the tree, start to trace back to the root of
 * the tree. By using this method, it is trying to save the memory and computation time.
 * And by using this method, we could further optimise the searching time in the future and
 * more flexible to do with the path to the target.
 *
 * If your want to use this method for searching the shorest path, enable the define
 * SHORTEST_PATH_V2 when compile the program. The are 2 ways to store the line segments,
 * either using dynamic memory or use the static memory. It is found that, when use the
 * dynamic memory, due to the system doesn't come with the memory management machanism,
 * the program always hang when allocation memory. So it should be use with care. When
 * using the static memory, a predefined array have to be assigned, it is defined in the
 * header file ShortestPath.h, which is POS_LINE_CNT, currently, it is set to
 * (MAP_SIZE * 11 / 4), in the complicate environment, this value is not enough. It is
 * better to set with a larger value.
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

void PC_NavDebug_Line(Line_t L,uint8_t type);

static Line_t g_pathline[POS_LINE_CNT];

volatile Point16_t g_pathpoints[POS_POINT_CNT + 1] = {0};

volatile uint16_t g_pathpoint_cnt = 0;

static uint16_t g_line_cnt = 0;

volatile uint16_t g_line_level=0;


/*Creating lines from robot position y to target position y
	*@ Robot_Y robot position y
	*@ Target_Y target position y
	*
	*
*/
uint8_t ShortestPath_GeneratePath(Point16_t robot_cell,uint8_t first_line_width)
{
	Line_t temp_line;
	Usprintf("%s(%d)\n", __FUNCTION__, __LINE__);
	PathLine_Reset();//clear Path_Line[] content and set line_cnt to 0
	PathPoint_ClearAllPoints();//clear Path_Points[] 
	PathList_Clear();
	temp_line = PathLine_FindOneLine(robot_cell,first_line_width);//from robot pos to create a line
	PathLine_AddLine(temp_line);
	
	if(PathLine_FindAllLines())//lines from Robot_Pos to Target_Pos has created
	{
		PathLine_SortLines();
		ShortestPath_CreatePathPoint(robot_cell);
		PointList_ShowAllPathPoints();
		return 1;
	}
	else
	{
		Usprintf("%s(%d)No Line Found\n", __FUNCTION__, __LINE__);
	}
	
	//Usprintf("\n%s %d: No Line Created  ", __FUNCTION__, __LINE__);
	return 0;
	//PathLine_ShowAllLines();
}
void ShortestPath_CreatePathPoint(Point16_t robot_cell)
{
	DirectionCardinal_t pre_dir = DIR_NONE,cur_dir = DIR_NONE ;
	int16_t breakcnt = 10000,line_idx;
	Point16_t pre_cell,target;
	uint8_t x_level = 0;
	Line_t line;
	
	line_idx = PathLine_GetLineCnt() - 1;
	line = g_pathline[line_idx];
	target = TargetList_GetCurrentTarget();
	
	x_level = Math_Diff_int(target.X,line.x_start+line.x_first);

	//PathPoint_AddOnePoint(robot_cell);//edit by vin
	if(!PathLine_IsPosOnLine(target,line))
	{
		PathPoint_AddOnePoint(target);
		target.Y = line.y;
		Usprintf("%s(%d):offset target position\n",__FUNCTION__,__LINE__);
	}
	else if(PathLine_GetLineCnt() == 1)
	{
		Usprintf("%s(%d):path line only one line\n",__FUNCTION__,__LINE__);
		PathPoint_AddOnePoint(target);
		PathPoint_AddOnePoint(robot_cell);
		return ;
	}	
	
	while((Mode_GetMode() != MODE_USERINTERFACE)&&(breakcnt--))//edit by vin
	{		
		pre_cell = target;
		pre_dir = cur_dir;
		
		LineList_GetNextLowPriorityPoint(&line_idx,&target,&x_level,&cur_dir);
//		Usprintf("target(%d,%d),x_level:%d,pre_dir:%d,cur_dir:%d,\n",target.x,target.y,x_level,pre_dir,cur_dir);
		if(cur_dir != pre_dir)
		{
			PathPoint_AddOnePoint(pre_cell);	
		}
		
		if((line_idx == 0)&&(x_level == 0))
		{
			PathPoint_AddOnePoint(target);	
			break;
		}
		
		if((pre_cell.X == target.X) && (pre_cell.Y == target.Y))
		{
			Usprintf("%s(%d):-----Error-----,reach same point\n",__FUNCTION__,__LINE__);
		}
	}	
}

uint8_t LineList_GetNextLowPriorityPoint(int16_t *line_idx,Point16_t *next_cell,uint8_t *x_level,DirectionCardinal_t *next_dir)
{
	Point16_t  cur_cell = *next_cell;
	int16_t i = 0,cur_line_idx = *line_idx;
	Line_t line;
	uint8_t isfind = 0;
	Point16_t up_cell,down_cell ;
	int16_t min,max;
	line = g_pathline[cur_line_idx];
	
	up_cell = Map_GridToCell(next_cell->X,next_cell->Y+1);
	down_cell = Map_GridToCell(next_cell->X,next_cell->Y-1);
	
//	Usprintf("select cell(%d,%d),dir:%d,x_level:%d,idx:%d,line y:%d x:%d\n",next_cell->X,next_cell->Y,*next_dir,*x_level,*line_idx,line.y,line.xs);

	for(i = cur_line_idx;(i >= 0)&&(!isfind);i--)
	{
		if(g_pathline[i].level == (line.level - 1)&&(g_pathline[i].select == 1))
		{
			
			if((g_pathline[i].y == up_cell.Y)&&(up_cell.X >= g_pathline[i].x_start)&&(up_cell.X <= (g_pathline[i].x_start+g_pathline[i].x_offset)))
			{
				
				*next_cell = up_cell;
				*x_level = Math_Diff_int(up_cell.X,g_pathline[i].x_start +g_pathline[i].x_first);
				*next_dir = NORTH;
				*line_idx = i;
		//		Usprintf("find_up,y:%d,x:%d,%d select:%d\n",g_pathline[i].y,g_pathline[i].x_start,g_pathline[i].x_offset,g_pathline[i].select);
				PC_NavDebug(next_cell->X,next_cell->Y,0,9);
				return 1;
			}
			else if((g_pathline[i].y == down_cell.Y)&&(down_cell.X >= g_pathline[i].x_start)&&(down_cell.X <= (g_pathline[i].x_start+g_pathline[i].x_offset)))
			{				
				*next_cell = down_cell;
				*x_level = Math_Diff_int(down_cell.X,g_pathline[i].x_start +g_pathline[i].x_first);
				*next_dir = SOUTH;
				*line_idx = i;
		//		Usprintf("find_down,y:%d,x:%d,%d select:%d\n",g_pathline[i].y,g_pathline[i].x_start,g_pathline[i].x_offset,g_pathline[i].select);
				PC_NavDebug(next_cell->X,next_cell->Y,0,9);
				return 1;
			}
		}		
		else if(g_pathline[i].level == (line.level - 2))
		{
			break;
		}
	}
	

	if(*x_level != 0)    //same line
	{
		next_cell->X = line.x_start + line.x_first ;	
		*x_level = 0;
		*next_dir = (next_cell->X < (line.x_start + line.x_first))? EAST : WEST;
		PC_NavDebug(next_cell->X,next_cell->Y,0,9);
		return 1;
	}
	else  
	{
		/*斜对角*/
		
		for(i = cur_line_idx;(i >= 0)&&(!isfind);i--)
		{
			if((g_pathline[i].level == (line.level - 1))&&(g_pathline[i].select == 1)&&Math_Diff_int(g_pathline[i].y,line.y)== 1)
			{
				max =  Math_GetMin(g_pathline[i].x_start + g_pathline[i].x_offset,line.x_start +line.x_offset);
				min =  Math_GetMax(g_pathline[i].x_start,line.x_start);	
				
				if(max >= min -1)
				{	
					if((g_pathline[i].x_start +g_pathline[i].x_offset) < line.x_start)
					{						
						*next_cell = Map_GridToCell(g_pathline[i].x_start +g_pathline[i].x_offset,g_pathline[i].y);	
					}
					else if((g_pathline[i].x_start) > (line.x_start +line.x_offset))
					{
						*next_cell = Map_GridToCell(g_pathline[i].x_start,g_pathline[i].y);	
					}
					else
					{
						Usprintf("%s(%d):-----Error-----\n",__FUNCTION__,__LINE__);
						*next_cell = Map_GridToCell(cur_cell.X,g_pathline[i].y);
					}
					
					*next_dir =  Path_GetCellHeading8(cur_cell,*next_cell);	
					*line_idx = i;
					*x_level = Math_Diff_int(g_pathline[i].x_start + g_pathline[i].x_first,next_cell->X);
					PC_NavDebug(next_cell->X,next_cell->Y,0,9);
					return 1;
				}
			}
			else if(g_pathline[i].level == (line.level - 2))
			{
				break;
			}
		}
	}
	
	Usprintf("%s(%d):-----Error-----,can't find\n",__FUNCTION__,__LINE__);
	
	return 0;
}


/*find all line starting from robot pos and stops when reached at any target
*/
uint8_t PathLine_FindAllLines(void)
{
	uint16_t pre_line_idx = 0, new_line_idx = 0, current_line_idx = 0;
//	uint16_t fine_line_timer = 0;
	uint16_t break_cnt = 10000;
	uint8_t pre_flag = 1;
	Usprintf("\n%s %d: Start to find line Line_Cnt = %d ", __FUNCTION__, __LINE__,PathLine_GetLineCnt());
//	fine_line_timer = Time_GetCurrentTime();
	while((break_cnt--) && (Mode_GetMode() != MODE_USERINTERFACE))//edit by vin
	{
		pre_flag=1;
		current_line_idx = pre_line_idx;
		while(current_line_idx < new_line_idx)
		{
			if(pre_flag)
			{
				pre_line_idx = PathLine_GetLineCnt();
				pre_flag=0;
			}
			if(PathLine_FindNextLines(PathLine_ReadLine(current_line_idx)))//向线段两边扩散，直到包含目标点，并把包含的目标点设为当前目标点
			{
				//Usprintf("\n%s %d: Reach Target!", __FUNCTION__, __LINE__);
				return 1;
			}
			current_line_idx++;
			//Usprintf("\n%s %d: Current_Line_Idx=%d New_Line_Idx=%d", __FUNCTION__, __LINE__,Current_Line_Idx,New_Line_Idx);
		}
		new_line_idx = PathLine_GetLineCnt();//all line
		PathLine_IncreaseLevel();
		
		if(pre_line_idx == new_line_idx)
		{
			//Usprintf("\n%s %d: No New line added New_Line_Idx=%d ", __FUNCTION__, __LINE__,New_Line_Idx);
			break;//no new line added
		}
	}
	//Usprintf("\n%s %d: No Line Found Out_Cnt = %d ", __FUNCTION__, __LINE__,Out_Cnt);
	return 0;
	//New_Line_Cnt = PathLine_GetLineCnt()-Current_Line_Idx; 
}
/*find a line by given start cell and line width
*/
Line_t PathLine_FindOneLine(Point16_t start_cell, uint8_t line_width)
{
	Line_t ret_val;
	ret_val.y = start_cell.Y;
	int16_t x_min, x_max;
	uint8_t low_flag = 1, high_flag = 1;
	
	x_min = x_max = start_cell.X; 
	
//	Usprintf("\n\r%s %d: StartPos X = %d  Y = %d ", __FUNCTION__, __LINE__, Temp_Pos.X,Temp_Pos.Y);
	while(low_flag || high_flag)
	{
		if((x_min > MAP_LIMIT_LOW) && (low_flag) && (x_min >(g_x_min-5)))//edit by vin
		{
			x_min--;
//			if((!Map_Cell_Blocked(temp_xs, ret_val.y))&&(!Map_Cell_Blocked(temp_xs, ret_val.y + line_width))
//				&& (!Map_Cell_Blocked(temp_xs, ret_val.y - line_width)))//3 line is not blocked
			if(Map_IsBlock_Access(x_min, ret_val.y))
			{
				
			}
			else
			{
				if(x_min<start_cell.X)x_min ++;
				low_flag = 0;
			}
		}
		else
		{
			low_flag = 0;
		}
		
		if((x_max < MAP_LIMIT_HIGH) && (high_flag) && (x_max < (g_x_max+5)))//edit by vin
		{
			x_max++;
//			if((!Map_Cell_Blocked(temp_xe,ret_val.y)) && (!Map_Cell_Blocked(temp_xe, ret_val.y + line_width)) 
//				&& (!Map_Cell_Blocked(temp_xe,ret_val.y - line_width)))//3 line is not blocked
			if(Map_IsBlock_Access(x_max,ret_val.y))
			{

			}
			else
			{
				if(x_max>start_cell.X)x_max --;
				high_flag=0;
			}
		}
		else
		{
			high_flag=0;
		}		
	}
	ret_val.x_start = x_min;
	ret_val.x_offset = x_max - x_min;
	ret_val.x_first = start_cell.X - x_min;	
	ret_val.select = 0;
	ret_val.level = PathLine_GetLineLevel();
	//Usprintf("\n\r%s %d: Line (%d,%d,%d) Level = %d", __FUNCTION__, __LINE__, ret_val.xs,ret_val.xe,ret_val.y,ret_val.level);
	PC_NavDebug_Line(ret_val,5);
	return ret_val;
}
/*find lines which next to the given line 
*/
uint8_t PathLine_FindNextLines(Line_t current_line)
{
	Line_t next_line;
	uint16_t target_idx = 0;
	Point16_t cell/*,target*/;
	uint8_t find_line = 0;
	uint8_t min_stop = 0,max_stop = 0,offset = 0,j = 0;
	int16_t search_min ,search_max;
	uint8_t ischeck = 0;	
	
	for(cell.Y = (current_line.y + 1);cell.Y >= (current_line.y - 1);cell.Y--)
	{
		if(cell.Y == current_line.y)continue;       //current lane is no need to find
		if((cell.Y >= (MAP_LIMIT_HIGH - 2))||(cell.Y <= (MAP_LIMIT_LOW + 2)))continue;
		if((cell.Y >= (g_y_max + 5))||(cell.Y <= (g_y_min - 5)))continue;//edit by vin		
		
		search_min = current_line.x_start + current_line.x_first;
		search_max = current_line.x_start + current_line.x_first;
		
		min_stop = max_stop = offset = 0;

		while((Mode_GetMode() != MODE_USERINTERFACE))//edit by vin
		{	
			for(j = 0;j <= 1;j++)
			{					
				if(j == 0)
				{
					if(min_stop)continue;
					
					cell.X = (current_line.x_start + current_line.x_first) - offset;
					
					if((cell.X  < (current_line.x_start - 1))||(search_min < current_line.x_start))
					{
						min_stop = 1;
						ischeck = 0;
					}
					else if((cell.X <= search_min))
					{
						ischeck = 1;
					}
					else
					{
						ischeck = 0;
					}
				}
				else if(j == 1)
				{
					if(max_stop)continue;
					
					cell.X = (current_line.x_start + current_line.x_first) + offset;

					if((cell.X  > (current_line.x_start + current_line.x_offset+1))||(search_max > (current_line.x_start + current_line.x_offset)))
					{
						max_stop = 1;
						ischeck = 0;
					}
					else if((cell.X >= search_max))
					{
						ischeck = 1;
					}	
					else
					{
						ischeck = 0;
					}
				}
				
				if(ischeck)
				{
					find_line = 1;
					if(!Map_IsBlock_Access(cell.X,cell.Y))
					{
						find_line = 0;
					}							
																									
					if(find_line)
					{
						next_line = PathLine_FindOneLine(cell,1);
						PathLine_AddLine(next_line);
						for(target_idx = 0; target_idx < TargetList_ReadCnt(); target_idx++)
						{
							if(PathLine_IsPosOnLine(TargetList_ReadTarget(target_idx),next_line))//target is in line
							{
								TargetList_StoreCurrentTarget(TargetList_ReadTarget(target_idx));
								Usprintf("\n%s Current target = (%d,%d)\n", __FUNCTION__, __LINE__,TargetList_ReadTarget(target_idx).X,TargetList_ReadTarget(target_idx).Y);
								return 1;
							}
						}						
					
						if(next_line.x_start < search_min)
						{
							search_min = next_line.x_start - 1 ;
						}
						if((next_line.x_start + next_line.x_offset) > search_max)
						{
							search_max = (next_line.x_start + next_line.x_offset) + 1;
						}	
					}	
				}	
			}
			
			if(min_stop&&max_stop)
			{
				break;
			}
			offset++;	
		}
	}
	return 0;	
}

/*sort lines
* start from target line to robot line
* get the next line which next to high level and deletes all other lines have on same level
* after this process, will get only one direction lines from robot to target 
*/
void PathLine_SortLines(void)
{
	int16_t i = 0,pre_idx = 0,low_idx = 1000;
	int16_t min,max;
	
	pre_idx = g_line_cnt - 1;	
	for(i = g_line_cnt - 1;i >= 0;i--)
	{
		if(g_pathline[i].level <= g_pathline[pre_idx].level - 2)
		{	
			g_pathline[low_idx].select = 1;
			PC_NavDebug_Line(g_pathline[low_idx],8);
			pre_idx = low_idx;
		}
		
		if((g_pathline[i].level == (g_pathline[pre_idx].level - 1))&&Math_Diff_int(g_pathline[i].y,g_pathline[pre_idx].y)== 1)
		{
			max =  Math_GetMin(g_pathline[i].x_start + g_pathline[i].x_offset,g_pathline[pre_idx].x_start +g_pathline[pre_idx].x_offset);
			min =  Math_GetMax(g_pathline[i].x_start,g_pathline[pre_idx].x_start);
			
			if(max >= min - 1)
			{
				low_idx = i;
			}
		}
	}	
	g_pathline[g_line_cnt - 1].select = 1;
	g_pathline[0].select = 1;		
}

/*reset all path lines
*/
void PathLine_Reset(void)
{
	g_line_cnt = 0;
	PathLine_ResetLineLevel();
}
/*add a line to the line list
*/
void PathLine_AddLine(Line_t line)
{
	if (g_line_cnt + 1 >= POS_LINE_CNT) {
		Usprintf("%s(%d):too many lines inserted %d\n", __FUNCTION__, __LINE__, g_line_cnt);
		return;
	}
	if(PathLine_LineExisted(line))
	{
		return;
	}
	
	g_pathline[g_line_cnt] = line;
	g_line_cnt++;
}
/*get line counts of line list
*/
uint16_t PathLine_GetLineCnt(void)
{
	return g_line_cnt;
}
/*read a line in the line list by given index
*/
Line_t PathLine_ReadLine(uint16_t idx)
{
	return g_pathline[idx];
}
/*edit a line in the line list by given index
*/
void PathLine_EditLine(Line_t L,uint16_t idx)
{
	if (idx >= POS_LINE_CNT) 
	{
		Usprintf("%s(%d):Line Index out of range %d\n", __FUNCTION__, __LINE__, idx);
		return;
	}
	g_pathline[idx] = L;
}


/*get current level of the line list
*/
uint16_t PathLine_GetLineLevel(void)
{
	return g_line_level;
}
/*increase the line level 
*/
void PathLine_IncreaseLevel(void)
{
	g_line_level++;
}
/*reset line level
*/
void PathLine_ResetLineLevel(void)
{
	g_line_level = 0;
}
/*check if the given line is already on the line list
*/
uint8_t PathLine_LineExisted(Line_t line)
{
	uint16_t i=0;
	for(i = 0; i < PathLine_GetLineCnt(); i++)
	{
		if((line.x_start == g_pathline[i].x_start) && (line.x_offset == g_pathline[i].x_offset) && (line.y == g_pathline[i].y))
		{
			return 1;
		}
	}
	return 0;
}

/*check if the given cell is on the given line
*/
uint8_t PathLine_IsPosOnLine(Point16_t cell,Line_t line)
{
	if(cell.Y == line.y)
	{
		if((cell.X >= line.x_start) && (cell.X <= (line.x_start+line.x_offset)))return 1;
	}
	return 0;
}
/*show all line of the line list by printing to usart
*/
void PathLine_ShowAllLines(void)
{ 
	uint16_t i = 0;
	Usprintf("%s(%d):Line_Cnt = %d \n",__FUNCTION__, __LINE__,PathLine_GetLineCnt());
	for(i = 0; i < PathLine_GetLineCnt(); i++)
	{
//		Usprintf("%s(%d):Line Xs(%d)Xe(%d)Y(%d)Lv(%d)Idx(%d)\n",
//							__FUNCTION__, __LINE__,PathLine_ReadLine(i).xs,PathLine_ReadLine(i).xe,PathLine_ReadLine(i).y,
//							PathLine_ReadLine(i).level,PathLine_ReadLine(i).idx);
		PC_NavDebug_Line(PathLine_ReadLine(i),5);
	}
}
/*calculates acreage of all line of line list
*/
uint32_t PathLine_GetAcreage(void)
{
	uint32_t acreage_temp=0;
	Line_t temp_line;
	uint16_t i=0;
	for(i=0;i<PathLine_GetLineCnt();i++)
	{
		//Send_MapLine(PathLine_ReadLine(i),2);
		temp_line = PathLine_ReadLine(i);
		acreage_temp += (temp_line.x_offset + 1);
	}
	return acreage_temp;
}

/*PathPont*/
/*add all the path point to the path list
*/
void PathPoint_AddPathPointToPathList(uint16_t start_idx)
{
	uint16_t i = 0;
	PathList_t temp_pathlist;
	for(i = start_idx; i < PathPoint_GetPointCnt(); i++)
	{
		temp_pathlist.cell_pos = PathPoint_ReadPoint(i);
		temp_pathlist.status = SHORT_PATH;
		PathList_AddNewPoint(temp_pathlist);
	}
}

void PathPoints_Rearrange(void)
{
	uint16_t i = 0,max=0;
	Point16_t temp_point;
	max = PathPoint_GetPointCnt()-1;
	for(i = 0; i < (PathPoint_GetPointCnt()+1)/2; i++)
	{
		temp_point = PathPoint_ReadPoint(i);
		PathPoint_EditPoint(PathPoint_ReadPoint(max-i),i);
		PathPoint_EditPoint(temp_point,max-i);
	}
	
}

/*insert lane away from obstacles*/
void PathPoint_Optimize(void)
{
	uint16_t i = 0;
	int16_t xs = 0,xe = 0,temp_y = 0;
	int16_t ys = 0,ye = 0,temp_x = 0;
	uint8_t reliable_val = 0;
	for(i = 0;i < PathPoint_GetPointCnt();i++)
	{
		if(g_pathpoints[i].Y == g_pathpoints[i + 1].Y)//same insert lane
		{
			Usprintf("%s(%d)Y origin f p:(%d,%d)  s p:(%d,%d)\n",__FUNCTION__, __LINE__,g_pathpoints[i].X,g_pathpoints[i].Y,g_pathpoints[i + 1].X,g_pathpoints[i + 1].Y);
			xs = Math_GetMin(g_pathpoints[i].X,g_pathpoints[i+1].X);
			xe = Math_GetMax(g_pathpoints[i].X,g_pathpoints[i+1].X);			
			temp_y = g_pathpoints[i].Y;
			reliable_val = PathPoint_PathYReliableWeight(xs, xe,temp_y);
		  Usprintf("%s(%d)Y reliable_val :%d\n",__FUNCTION__, __LINE__,reliable_val);
			if(reliable_val)
			{
				if(reliable_val == 1)//down
				{
					if(PathPoint_PathYReliableWeight(xs, xe,temp_y - 1) == 0)
					{
						g_pathpoints[i].Y = temp_y - 1;
						g_pathpoints[i + 1].Y = temp_y - 1;
						Usprintf("%s(%d)Y change f p:(%d,%d)  s p:(%d,%d)\n",__FUNCTION__, __LINE__,g_pathpoints[i].X,g_pathpoints[i].Y,g_pathpoints[i + 1].X,g_pathpoints[i + 1].Y);
					}						
				}
				if(reliable_val == 2)//up
				{
					if(PathPoint_PathYReliableWeight(xs, xe,temp_y + 1) == 0)
					{
						g_pathpoints[i].Y = temp_y + 1;
						g_pathpoints[i + 1].Y = temp_y + 1;
						Usprintf("%s(%d)Y change f p:(%d,%d)  s p:(%d,%d)\n",__FUNCTION__, __LINE__,g_pathpoints[i].X,g_pathpoints[i].Y,g_pathpoints[i + 1].X,g_pathpoints[i + 1].Y);
					}								
				}
			}			
		}		
	}
	for(i = 0;i < PathPoint_GetPointCnt();i++)
	{
		if(g_pathpoints[i].X == g_pathpoints[i + 1].X)//same line
		{
			Usprintf("%s(%d)origin X f p:(%d,%d)  s p:(%d,%d)\n",__FUNCTION__, __LINE__,g_pathpoints[i].X,g_pathpoints[i].Y,g_pathpoints[i + 1].X,g_pathpoints[i + 1].Y);
			ys = Math_GetMin(g_pathpoints[i].Y,g_pathpoints[i+1].Y);
			ye = Math_GetMax(g_pathpoints[i].Y,g_pathpoints[i+1].Y);			
			temp_x = g_pathpoints[i].X;
			reliable_val = PathPoint_PathXReliableWeight(ys, ye,temp_x);
		  Usprintf("%s(%d)X reliable_val :%d\n",__FUNCTION__, __LINE__,reliable_val);
			if(reliable_val)
			{
				if(reliable_val == 1)//left
				{
					if(PathPoint_PathXReliableWeight(ys, ye,temp_x - 1) == 0)
					{
						g_pathpoints[i].X = temp_x - 1;
						g_pathpoints[i + 1].X = temp_x - 1;
						Usprintf("%s(%d)X change f p:(%d,%d)  s p:(%d,%d)\n",__FUNCTION__, __LINE__,g_pathpoints[i].X,g_pathpoints[i].Y,g_pathpoints[i + 1].X,g_pathpoints[i + 1].Y);
					}							
				}
				if(reliable_val == 2)//right
				{
					if(PathPoint_PathXReliableWeight(ys, ye,temp_x + 1) == 0)
					{
						g_pathpoints[i].X = temp_x + 1;
						g_pathpoints[i + 1].X = temp_x + 1;	
						Usprintf("%s(%d)X change f p:(%d,%d)  s p:(%d,%d)\n",__FUNCTION__, __LINE__,g_pathpoints[i].X,g_pathpoints[i].Y,g_pathpoints[i + 1].X,g_pathpoints[i + 1].Y);
					}								
				}
			}						
		}		
	}	
}
/*the path is along the obstacle*/
/*
*0:this path can be used 
*1:this path should be adjust,direction :down
*2:this path should be adjust,direction :up
*3:only this path can be used
*4:this path is not accessible
*/
uint8_t PathPoint_PathYReliableWeight(int16_t xs,int16_t xe,int16_t y)
{
	int16_t j = 0;
	uint8_t r_val = 0;
	for(j = xs;j <= xe;j++)
	{	
		if((Map_GetCell(j,y) >= BLOCKED)||(Map_GetCell(j,y - 1) >= BLOCKED)||(Map_GetCell(j,y + 1) >= BLOCKED))
		{
			return 4;		
		}		
	}
	for(j = xs;j <= xe;j++)
	{
		if(Map_GetCell(j,y + 2) >= BLOCKED)
		{
			r_val |= 0x01;
		}
		if(Map_GetCell(j,y - 2) >= BLOCKED)
		{
			r_val |= 0x02;
		}			
	}	
	return r_val;
}
/*the path is along the obstacle*/
/*
*0:this path can be used 
*1:this path should be adjust,direction :left
*2:this path should be adjust,direction :right 
*3:only this path can be used
*4:this path is not accessible
*/
uint8_t PathPoint_PathXReliableWeight(int16_t ys,int16_t ye,int16_t x) 
{
	int16_t j = 0;
	uint8_t r_val = 0;
	for(j = ys;j <= ye;j++)
	{	
		if((Map_GetCell(x,j) >= BLOCKED)||(Map_GetCell(x - 1,j) >= BLOCKED)||(Map_GetCell(x + 1,j) >= BLOCKED))
		{
			return 4;		
		}		
	}
	for(j = ys;j <= ye;j++)
	{
		if(Map_GetCell(x + 2,j) >= BLOCKED)
		{
			r_val |= 0x01;
		}
		if(Map_GetCell(x - 2,j) >= BLOCKED)
		{
			r_val |= 0x02;
		}							
	}		
	return r_val;
}
/*add a point to the path point
*/
void PathPoint_AddOnePoint(Point16_t point)
{
	if(g_pathpoint_cnt >= POS_POINT_CNT)
	{
		Usprintf("%s(%d)%d\n",__FUNCTION__, __LINE__,g_pathpoint_cnt);
		return;
	}
	g_pathpoints[g_pathpoint_cnt] = point;
	Usprintf("%s %d:P=(%d,%d) Cnt=%d  \n", __FUNCTION__, __LINE__,point.X,point.Y,g_pathpoint_cnt);
	g_pathpoint_cnt++;
//	PC_NavDebug(point.X,point.Y,0,BLOCKED_BOUNDARY);
//	vTaskDelay(800/portTICK_RATE_MS);
}
/*get the number of the path points
*/
uint16_t PathPoint_GetPointCnt(void)
{
	return g_pathpoint_cnt;
}
/*edit a point in the path point list to the given point by index
*/
void PathPoint_EditPoint(Point16_t point,uint16_t idx)
{
	if(idx >= POS_POINT_CNT)
	{
		Usprintf("%s(%d)%d\n",__FUNCTION__, __LINE__,g_pathpoint_cnt);
		return;
	}
	g_pathpoints[idx] = point;
	Usprintf("%s %d:P=(%d,%d) Cnt=%d  \n", __FUNCTION__, __LINE__,point.X,point.Y,idx);
}
/*read a path point by given index
*/
Point16_t PathPoint_ReadPoint(int16_t idx)
{
	if(idx < 0)
	{
		Usprintf("%s(%d)Idx=%d\n",__FUNCTION__, __LINE__,idx);
	}
	return g_pathpoints[idx];
}
/*read the last point added to the path point
*/
Point16_t PathPoint_ReadLastPoint(void)
{
	if(g_pathpoint_cnt == 0)return g_pathpoints[0];
	return g_pathpoints[g_pathpoint_cnt - 1];
}
/*clear all the path points and reset path point counts
*/
void PathPoint_ClearAllPoints(void)
{
	uint16_t i=0;
	for(i = 0; i < PathPoint_GetPointCnt(); i++)
	{
		g_pathpoints[i].X = 0;
		g_pathpoints[i].Y = 0;
	}
	g_pathpoint_cnt = 0;
}
/*remove one path point by given index
*/
void PathPoint_RemoveOnePoint(uint16_t idx)
{
	uint16_t i=0;
	//Usprintf("%s(%d):g_pathpoints[%d](%d,%d)\n",__FUNCTION__, __LINE__,idx,g_pathpoints[idx].X,g_pathpoints[idx].Y);
	for(i = idx; i < (PathPoint_GetPointCnt() - 1); i++)
	{
		g_pathpoints[i] = g_pathpoints[i+1];
	}
	g_pathpoint_cnt--;
}
/*remove one last point
*/
void PathPoint_RemoveLastPoint(void)
{
	Usprintf("%s(%d):g_pathpoints[%d](%d,%d)\n",__FUNCTION__, __LINE__,g_pathpoint_cnt,g_pathpoints[g_pathpoint_cnt-1].X,\
	g_pathpoints[g_pathpoint_cnt-1].Y);
	if(g_pathpoint_cnt > 0)g_pathpoint_cnt--;
}
/*insert a point to the path point list at the given index
*/
void PathPoint_InsertOnePoint(Point16_t point,uint8_t idx)
{
	uint16_t i = 0;
	for(i = PathPoint_GetPointCnt(); i > idx; i--)
	{
		g_pathpoints[i] = g_pathpoints[i-1];
	}
	g_pathpoints[idx] = point;
	g_pathpoint_cnt++;
}
/*show all the path points to pc debug tool
*/
void PathPoint_ShowAllPathPoints(void)
{
	uint16_t i = 0;
	Point16_t temp_point;
	for(i = 0; i < PathPoint_GetPointCnt(); i++)
	{
		temp_point = PathPoint_ReadPoint(i);
		PC_NavDebug(temp_point.X, temp_point.Y, 0, BLOCKED_CLIFF);
		Usprintf("%s(%d):g_pathpoints[%d](%d,%d)\n",__FUNCTION__, __LINE__,i,g_pathpoints[i].X,g_pathpoints[i].Y);
	}
}

void PointList_ShowAllPathPoints(void)
{
	int16_t i = 0,j = 0;
	Point16_t current_cell,next_cell;
	int16_t min = 0,max = 0;
	for(i = 0; i < PathPoint_GetPointCnt(); i++)
	{
//		Usprintf("%s(%d):g_pathpoints[%d](%d,%d)\n",__FUNCTION__, __LINE__,i,t_pointlist[i].x,t_pointlist[i].y);
	}
	
	for(i = 0;i < (PathPoint_GetPointCnt() - 1);i++)
	{
		current_cell = PathPoint_ReadPoint(i);
		next_cell = PathPoint_ReadPoint(i + 1);
		
		if(current_cell.Y == next_cell.Y) // same lane
		{
			min = (current_cell.X < next_cell.X) ? current_cell.X :next_cell.X;
			max = (current_cell.X < next_cell.X) ? next_cell.X : current_cell.X;
			
			for(j = min;j <= max;j++)
			{
				PC_NavDebug(j,current_cell.Y, 0, 9);
			}
		}
		else if(current_cell.X == next_cell.X)
		{
			min = (current_cell.Y < next_cell.Y) ? current_cell.Y :next_cell.Y;
			max = (current_cell.Y < next_cell.Y) ? next_cell.Y : current_cell.Y;
			
			for(j = min;j <= max; j++)
			{
				PC_NavDebug(current_cell.X,j, 0, 9);
			}
		}
	}
}


