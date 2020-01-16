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



uint8_t g_map[MAP_SIZE][(MAP_SIZE + 1) / 4];

//int16_t homeX, homeY;

double g_count_x, g_count_y, g_relative_sin, g_relative_cos;
uint16_t g_relative_theta = 3600;
int16_t g_x_min, g_x_max, g_y_min, g_y_max;
int16_t g_x_min_buffer = 0, g_x_max_buffer = 0, g_y_min_buffer = 0, g_y_max_buffer = 0;
int16_t g_x_range_min, g_x_range_max, g_y_range_min, g_y_range_max;

uint8_t g_wall_map_idx = 0;

Point8_t g_wall_map[WALL_MAP_SIZE]={0};

volatile Point16_t g_home_point ={0,0};
volatile int16_t g_boundary_width = 0, g_boundary_height = 0;
volatile int32_t g_boundary_width_cnt = 0, g_boundary_height_cnt = 0;

volatile Boundary_t g_map_boundary;

volatile uint8_t g_boundary_width_flag = 0,g_boundary_height_flag = 0;

//int16_t debugMsg[DEBUG_MSG_SIZE][3];
//uint8_t debugMsg_idx;

/*
	reset map[][] value to 0
	g_x_range_min = 1 - MAP_SIZE
	xRangeMax = MAP_SIZE - 1
	g_y_range_min = 1 - MAP_SIZE
	yRangeMax = MAP_SIZE - 1
	reset robot position count(x,y) to 0
	reset debugMsg to 0
*/
void Map_Initialize(void) 
{
	uint16_t c, d;
	#ifdef WIFI_TY
	AC_SetFlag(0);
	#endif
	mPC_NavDebug(0xff,0xff,0,0);
	if(Mode_GetMode() == MODE_USERINTERFACE)return;
	for(c = 0; c < MAP_SIZE; ++c) 
	{
		for(d = 0; d < (MAP_SIZE + 1) / 4; ++d) 
		{
			g_map[c][d] = 0;
		}
	}	
  Map_SetBoundary(80,80);//80
	g_x_min = g_x_max = g_y_min = g_y_max = 0;
	g_boundary_width_flag = 1;
	g_boundary_height_flag = 1;
	g_x_range_min = g_x_min - (MAP_SIZE - (g_x_max - g_x_min + 1));
	g_x_range_max = g_x_max + (MAP_SIZE - (g_x_max - g_x_min + 1));
	g_y_range_min = g_y_min - (MAP_SIZE - (g_y_max - g_y_min + 1));
	g_y_range_max = g_y_max + (MAP_SIZE - (g_y_max - g_y_min + 1));

	g_count_x = 0;
	g_count_y = 0;
	#ifdef WIFI_TY	
	g_clean_area=0;
	g_clean_time=0;
	AC_Reset_RealMap();
	Wifi_Update_MapReport_Init();
	Wifi_Open_Stream_trans();
	Wifi_Start_Stream_trans();
	AC_SetFlag(1);
	#endif
}

void Map_ResetBoundaryFlag(void)
{
	g_boundary_width_flag = 0;
	g_boundary_height_flag = 0;
}



/* calculate how many cells robot have cleaned
 * @return	numbers of cleaned cells on the map
 */
uint32_t Map_GetCellAcreage(void)
{
	uint32_t acreage_temp=0;
	int32_t x_sum = 0, y_sum = 0;
	int16_t mx,my;
	for(mx = Map_GetXMin();mx < Map_GetXMax();mx++)
	{
		for(my = Map_GetYMin();my < Map_GetYMax();my++)
		{
			if(Map_GetCell(mx,my)==CLEANED)
			{
				acreage_temp++;
				x_sum += mx;
				y_sum += my;
			}
		}
	}
	if(acreage_temp > 0)Usprintf("%s(%d):average(%d,%d)\n",__FUNCTION__, __LINE__,x_sum / acreage_temp,y_sum / acreage_temp);
	return acreage_temp;
}

int16_t Map_GetXMin(void)
{
	return g_x_min;
}
int16_t Map_GetXMax(void)
{
	return g_x_max;
}
int16_t Map_GetYMin(void)
{
	return g_y_min;
}
int16_t Map_GetYMax(void)
{
	return g_y_max;
}

/* get robot current position's X by encoder count
 */
int32_t Map_GetRobotCountX(void) 
{
	return (int32_t)round(g_count_x);
}
/* get robot current position's Y by encoder count
 */
int32_t Map_GetRobotCountY(void) 
{
	return (int32_t)round(g_count_y);
}
/* get robot current position (X,Y) by encoder count
 */
Point32_t Map_GetRobotCount(void)
{
	Point32_t Temp;
	Temp.X = (int32_t)round(g_count_x);
	Temp.Y = (int32_t)round(g_count_y);
	return Temp;
}
/* get robot cell position 
 * @return	Point16_t
 */
void Map_SetRobotCell(Point16_t cell)
{
  Map_SetRobotCount(Map_CellToCount(cell.X),Map_CellToCount(cell.Y));
}
/* get robot cell position 
 * @return	Point16_t
 */
Point16_t Map_GetRobotCell(void)
{
  Point16_t Temp;
  Temp.X = Map_GetRobotCellX();
  Temp.Y = Map_GetRobotCellY();
  return Temp;
}
/* get robot current cell's X position
 */
int16_t Map_GetRobotCellX(void) 
{
	return Map_CountToCell(g_count_x);
}
/* get robot current cell's Y position
 */
int16_t Map_GetRobotCellY(void) 
{
	return Map_CountToCell(g_count_y);
}
/* update robot's coordinate count
 * @param x	robot moves x counts 
 * @param y	robot moves y counts  
 */
void Map_MoveRobotCountTo(double d_x, double d_y) 
{
	g_count_x += d_x;
	g_count_y += d_y;
}
/* set robot's coordinate count to (x,y)
 * @param x	robot's new position x count 
 * @param y	robot's new position y count 
 */
void Map_SetRobotCount(double x, double y) 
{
	g_count_x = x;
	g_count_y = y;
}
/* check two input cell the same position or not
 * @param a	Cell a
 * @param a	Cell b
 * @return	1 cell a and b at the same cell position
 * 					0 two different cells
 */
uint8_t Map_TwoCellMatch(Point16_t cell_a,Point16_t cell_b)
{
	if(cell_a.X != cell_b.X)return 0;
	if(cell_a.Y != cell_b.Y)return 0;
	return 1;
}

/* check if robot current position count is in the center of robot cell
 * @return	1	 robot in the center
 *          0  robot not in the center
 */
uint8_t Map_IsRobotCellCenter(void)
{
	Point32_t Cell_Center;
	Cell_Center = Map_CellToPoint(Map_GetRobotCell());
	if(Math_Diff_int(Cell_Center.X,Map_GetRobotCountX())>CELL_COUNT_MUL_1_4)return 0;
	if(Math_Diff_int(Cell_Center.Y,Map_GetRobotCountY())>CELL_COUNT_MUL_1_4)return 0;
	return 1;
}
/*set cell to a certain value
 * @param id		Map id
 * @param x_cell		 
 * @param y_cell		 
 * @param value CellState_t
 */
void Map_SetCell(int16_t x, int16_t y, CellState_t value) 
{
	int16_t debug_x = x, debug_y = y;
	static uint8_t out_range_cnt = 0,clean_area_cnt=0;
	
	Map_Adjust_Mapsize(x,y);
	x = Map_CellToMap(x);
	y = Map_CellToMap(y);
	if(x >= 0 && x <= MAP_SIZE && y >= 0 && y <= MAP_SIZE) 
	{
		out_range_cnt = 0;
		
		if(Map_GetMapArray(x,y) != value) 
		{						
		  Map_SetMapArray(x,y,value);
			#ifdef WIFI_TY
			if(Get_ACFlag())
			{
				clean_area_cnt++;
				if(clean_area_cnt>100)
				{
					clean_area_cnt = 0;
					g_clean_area++;
				}			
			}
			if(value==BLOCKED)
			{					
				if(Get_ACFlag())
				{
					mPC_NavDebug(debug_x,debug_y,0,BLOCKED);
					AC_RealMap_AddPoint(debug_x,debug_y,value);					
				}		
			}	
			#endif			
		}
			
	}
	else
	{
		out_range_cnt++;
		if(out_range_cnt > 3)
		{
			out_range_cnt = 0;
			if((Mode_GetMode() == MODE_NAVIGATION)||(Mode_GetMode() == MODE_NAVIGATION2))
			{
				Usprintf("%s(%d):out range,should stop!!!\n",__FUNCTION__, __LINE__);
				Mode_SetMode(MODE_USERINTERFACE);
			}
		}
	}
//	}
}
/*Map_GetCell get cell(x,y) value 
 * @param id	Map id
 * @param x	Cell x
 * @param y	Cell y
 * @return	CellState
 */
CellState_t Map_GetCell(int16_t x, int16_t y) 
{
	CellState_t val;
	x = Map_CellToMap(x);
	y = Map_CellToMap(y);
	if(x >= 0 && x <= MAP_SIZE && y >= 0 && y <= MAP_SIZE) 
	{
		val = (CellState_t)Map_GetMapArray(x,y);
	}
	else 
	{
		Usprintf("%s(%d):out range\n",__FUNCTION__, __LINE__);
		Usprintf("%s(%d):input (%d,%d)\n",__FUNCTION__, __LINE__,x,y);
		Usprintf("%s(%d):range x(%d,%d), y(%d,%d)\n",__FUNCTION__, __LINE__,g_x_min,g_x_max,g_y_min,g_y_max);
		val = BLOCKED_BOUNDARY;
	}
	return val;
}
/*set map cell by checking if this cell is UNCLEAN
 * @param id	Map id
 * @param x	Cell x
 * @param y	Cell y
 * @return	CellState
 */
uint8_t Map_SetBlockIfUnclean(uint8_t id,int16_t x,int16_t y,CellState_t value)
{
	if(Map_GetCell(x,y) == UNCLEAN)
	{
		Map_SetCell(x,y,value);
		return 1;
	}
	return 0;
}


/* Adjust Map size by x,y*/
void Map_Adjust_Mapsize(int16_t x,int16_t y)
{
	if(x < g_x_min) 
	{//expand x xMin
		if(x > MAP_LIMIT_LOW)
		{
			g_x_min = x;
			g_x_range_min = g_x_max - MAP_SIZE;// + 1;//xMax-MAP_SIZE+1
			g_x_range_max = g_x_min + MAP_SIZE;// - 1;//xMin+MAP_SIZE-1
		}
	} 
	else if(x > g_x_max) 
	{//expand x Max
		if(x < MAP_LIMIT_HIGH)
		{
			g_x_max = x;
			g_x_range_min = g_x_max - MAP_SIZE;// + 1;//xMax-MAP_SIZE+1
			g_x_range_max = g_x_min + MAP_SIZE;// - 1;//xMin+MAP_SIZE-1
		}
	}
	if(y < g_y_min) 
	{//expand y Min
		if(y > MAP_LIMIT_LOW)
		{
			g_y_min = y;
			g_y_range_min = g_y_max - MAP_SIZE;// + 1;//yMax-MAP_SIZE+1
			g_y_range_max = g_y_min + MAP_SIZE;// - 1;//yMin+MAP_SIZE-1
		}
	} 
	else if(y > g_y_max) 
	{//expand y Max
		if(y < MAP_LIMIT_HIGH)
		{
			g_y_max = y;
			g_y_range_min = g_y_max - MAP_SIZE;// + 1;//yMax-MAP_SIZE+1
			g_y_range_max = g_y_min + MAP_SIZE;// - 1;//yMin+MAP_SIZE-1
		}
	}
}

/*get map array data by x,y of the array*/
uint8_t Map_GetMapArray(int16_t x,int16_t y)
{
	uint8_t map_val=0;
	map_val = g_map[x][y/4];
	map_val >>= (y%4)*2;
	map_val &= 0x03;
//	if(y&0x01)
//	{
//		Map_Val = map[x][y/2];
//		Map_Val &=0x0f;
//	}
//	else //odd y number
//	{
//		Map_Val = map[x][y/2];
//		Map_Val >>=4;
//	}
	return map_val;
}
/*Set map array data by x,y of the array*/
void Map_SetMapArray(int16_t x,int16_t y,uint8_t data)
{
	uint8_t pos = 0;
	pos = (y%4)*2;
	data <<= pos;
	g_map[x][y/4] &= ~(0x03<<pos); 
	g_map[x][y/4] |= data;
	
//	if(y&0x01)
//	{
//		y/=2;
//		data &=0x0f;
//		g_map[x][y] &= 0xf0;
//		g_map[x][y] |= data ;
//	}
//	else //odd y number
//	{
//		y/=2;
//		data <<=4;
//		g_map[x][y] &= 0x0f;
//		g_map[x][y] |= data;
//	}
	//Usprintf("\n\r Map content X = %d Y = %d M = %d ",x,y,map[x][y/2]);
}
/*return Map_Array Raw Data*/
uint8_t Get_Map_Raw(uint8_t x,uint8_t y)
{
  return g_map[x][y];
}
/*return Map_Array ram address*/
//void Get_Map_Address(uint8_t *ad)
//{
//  ad = &g_map[0][0];
//}

/*
@ intput data cell
@ return map array index
*/
int16_t Map_CellToMap(int16_t data)
{
//	data = data + MAP_SIZE + MAP_SIZE / 2;
//	data %= MAP_SIZE;
	data = data + MAP_SIZE / 2;
	return data;
}


void Map_ClearBlocks(void) 
{
	int16_t c, d;

	for(c = g_x_min; c < g_x_max; ++c) 
	{
		for(d = g_y_min; d < g_y_max; ++d) 
		{
			if(Map_GetCell( c, d) == BLOCKED_OBS || Map_GetCell(c, d) == BLOCKED_BUMPER || Map_GetCell(c, d) == BLOCKED_CLIFF) {
				if(Map_GetCell(c - 1, d) != UNCLEAN && Map_GetCell(c, d + 1) != UNCLEAN && Map_GetCell(c + 1, d) != UNCLEAN && Map_GetCell(c, d - 1) != UNCLEAN) {
					Map_SetCell((c), (d), CLEANED);
				}
			}
		}
	}

	Map_SetCell((Map_GetRobotCellX() - 1), (Map_GetRobotCellY()), CLEANED);
	Map_SetCell((Map_GetRobotCellX()), (Map_GetRobotCellY() + 1), CLEANED);
	Map_SetCell((Map_GetRobotCellX() + 1), (Map_GetRobotCellY()), CLEANED);
	Map_SetCell((Map_GetRobotCellX()), (Map_GetRobotCellY() - 1), CLEANED);
}
/*
	*calculating a relative X value by input a heading angle,offset_lat,offset_long
@ param uint16_t heading : input heading angle from 0-3600
@ parma int16_t offset_lat: offset distance from robot position left/right side by heading angle (count)  
@ parma int16_t offset_long: distance from robot position on heading angle (count)  
return int32_t: relative X by cell
*/
int32_t Map_GetRelativeX(uint16_t heading, int16_t offset_lat, int16_t offset_long) 
{
	int32_t ret_val = 0;
	if(heading != g_relative_theta) 
	{
		if(heading == 0) 
		{
			g_relative_sin = 0;
			g_relative_cos = 1;
		} 
		else if(heading == 900) 
		{
			g_relative_sin = 1;
			g_relative_cos = 0;
		}
		else if(heading == 1800) 
		{
			g_relative_sin = 0;
			g_relative_cos = -1;
		} 
		else if(heading == 2700) 
		{
			g_relative_sin = -1;
			g_relative_cos = 0;
		}
		else 
		{
			g_relative_sin = sin(Math_Deg2Rad(heading, 10));
			g_relative_cos = cos(Math_Deg2Rad(heading, 10));
		}
	}
	ret_val = Map_GetRobotCountX() + (int32_t)( (((double)offset_long * g_relative_cos * CELL_COUNT_MUL) -
	                                      ((double)offset_lat	* g_relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
	return Map_CountToCell(ret_val);
}
Point16_t Map_GetRelativeByCell(Point16_t target_cell,int16_t heading, int16_t offset_lat, int16_t offset_long) 
{
	int32_t temp_x, temp_y;
	Point16_t point;
	
	if(heading < 0)heading =  Math_NormalAngle(heading);
	
	if(heading != g_relative_theta) 
	{
		if(heading == 0) 
		{
			g_relative_sin = 0;
			g_relative_cos = 1;
		} 
		else if(heading == 900) 
		{
			g_relative_sin = 1;
			g_relative_cos = 0;
		} 
		else if(heading == 1800) 
		{
			g_relative_sin = 0;
			g_relative_cos = -1;
		} 
		else if(heading == 2700) 
		{
			g_relative_sin = -1;
			g_relative_cos = 0;
		} 
		else 
		{
			g_relative_sin = sin(Math_Deg2Rad(heading, 10));
			g_relative_cos = cos(Math_Deg2Rad(heading, 10));
		}
	}

	 temp_x = Map_CellToCount(target_cell.X) + (int32_t)( (((double)offset_long * g_relative_cos * CELL_COUNT_MUL) -
	                                      ((double)offset_lat	* g_relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
	
	 temp_y = Map_CellToCount(target_cell.Y) + (int32_t)( ( ((double)offset_long * g_relative_sin * CELL_COUNT_MUL) +
	                                      ((double)offset_lat *	g_relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
	 
	 point.X = Map_CountToCell(temp_x);
	 point.Y = Map_CountToCell(temp_y);
	 return point;
}
/*
	*calculating a relative Y value by input a heading angle,offset_lat,offset_long
@ param uint16_t heading : input heading angle from 0-3600
@ parma int16_t offset_lat: offset distance from robot position left/right side by heading angle (count)  
@ parma int16_t offset_long: distance from robot position on heading angle (count)  
return int32_t: relative Y by cell
*/
int32_t Map_GetRelativeY(uint16_t heading, int16_t offset_lat, int16_t offset_long) 
{
	int32_t ret_val = 0;
	if(heading != g_relative_theta)
	{
		if(heading == 0) 
		{
			g_relative_sin = 0;
			g_relative_cos = 1;
		}
		else if(heading == 900) 
		{
			g_relative_sin = 1;
			g_relative_cos = 0;
		} 
		else if(heading == 1800) 
		{
			g_relative_sin = 0;
			g_relative_cos = -1;
		} 
		else if(heading == 2700) 
		{
			g_relative_sin = -1;
			g_relative_cos = 0;
		} 
		else 
		{
			g_relative_sin = sin(Math_Deg2Rad(heading, 10));
			g_relative_cos = cos(Math_Deg2Rad(heading, 10));
		}
	}

	ret_val = Map_GetRobotCountY() + (int32_t)( ( ((double)offset_long * g_relative_sin * CELL_COUNT_MUL) +
	                                      ((double)offset_lat *	g_relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
	 
	return Map_CountToCell(ret_val);
}

void Map_GetRelativeXY(uint16_t heading, int16_t offset_lat, int16_t offset_long,int32_t *x,int32_t *y,double rad) 
{
	int32_t temp_x, temp_y;
	if(heading != g_relative_theta) 
	{
		if(heading == 0) 
		{
			g_relative_sin = 0;
			g_relative_cos = 1;
		} 
		else if(heading == 900) 
		{
			g_relative_sin = 1;
			g_relative_cos = 0;
		} 
		else if(heading == 1800) 
		{
			g_relative_sin = 0;
			g_relative_cos = -1;
		} 
		else if(heading == 2700) 
		{
			g_relative_sin = -1;
			g_relative_cos = 0;
		} 
		else 
		{
			g_relative_sin = sin(rad);
			g_relative_cos = cos(rad);
		}
	}

	 temp_x = Map_GetRobotCountX() + (int32_t)( (((double)offset_long * g_relative_cos * CELL_COUNT_MUL) -
	                                      ((double)offset_lat	* g_relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
	
	 temp_y = Map_GetRobotCountY() + (int32_t)( ( ((double)offset_long * g_relative_sin * CELL_COUNT_MUL) +
	                                      ((double)offset_lat *	g_relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
	 
	 *x = Map_CountToCell(temp_x);
	 *y = Map_CountToCell(temp_y);
}

Point16_t Map_GetRelativeXY2(uint16_t heading, int16_t offset_lat, int16_t offset_long) 
{
	int32_t temp_x, temp_y;
	Point16_t point;
	if(heading != g_relative_theta) 
	{
		if(heading == 0) 
		{
			g_relative_sin = 0;
			g_relative_cos = 1;
		} 
		else if(heading == 900) 
		{
			g_relative_sin = 1;
			g_relative_cos = 0;
		} 
		else if(heading == 1800) 
		{
			g_relative_sin = 0;
			g_relative_cos = -1;
		} 
		else if(heading == 2700) 
		{
			g_relative_sin = -1;
			g_relative_cos = 0;
		} 
		else 
		{
			g_relative_sin = sin(Math_Deg2Rad(heading, 10));
			g_relative_cos = cos(Math_Deg2Rad(heading, 10));
		}
	}

	 temp_x = Map_GetRobotCountX() + (int32_t)( (((double)offset_long * g_relative_cos * CELL_COUNT_MUL) -
	                                      ((double)offset_lat	* g_relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
	
	 temp_y = Map_GetRobotCountY() + (int32_t)( ( ((double)offset_long * g_relative_sin * CELL_COUNT_MUL) +
	                                      ((double)offset_lat *	g_relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
	 
	 point.X = Map_CountToCell(temp_x);
	 point.Y = Map_CountToCell(temp_y);
	 return point;
}
/*
 * convert a cell(int16_t) to count(double)  
 *
 * @param int16_t cell: cell(x/y) value
 * @return	converted int32_t count(x/y) value
 */
int32_t Map_CellToCount(int16_t cell) 
{
	return cell * CELL_COUNT_MUL;
}
/*
 * convert a count(double) to cell(int16_t)
 *
 * @param double count: count(x/y) value
 * @return	converted int16_t cell(x/y) value
 */
int16_t Map_CountToCell(double count) 
{
	if(count < -CELL_COUNT_MUL_1_2) 
	{
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL - 1;
	} 
	else 
	{
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL;
	}
}
int16_t Map_CountToCell_My(double count) 
{
	return count/600;
}
/*
 * convert a Point16_t Cell to count(Point32_t)
 *
 * @param Point16_t cell:	input cell position(x,y)
 * @return	converted Point32_t count
 */
Point32_t Map_CellToPoint( Point16_t cell ) 
{
	Point32_t pnt;
	pnt.X = Map_CellToCount(cell.X);
	pnt.Y = Map_CellToCount(cell.Y);
	return pnt;
}
/*
 * convert a Point32_t count to cell(Point16_t)
 *
 * @param Point32_t pnt:	input count(x,y)
 * @return	Point16_t converted cell(x,y)
 */
Point16_t Map_PointToCell( Point32_t pnt ) 
{
	Point16_t cell;
	cell.X = Map_CountToCell(pnt.X);
	cell.Y = Map_CountToCell(pnt.Y);
	return cell;
}
Point16_t Map_PointToCell_My( Point32_t pnt ) 
{
	Point16_t cell;
	cell.X = Map_CountToCell_My(pnt.X);
	cell.Y = Map_CountToCell_My(pnt.Y);
	return cell;
}
/*
 * set a count*count cells to state started from cell_x,cell_y
 *
 * @param int8_t count:				counts to set 
 * @param int16_t cell_x:			counts to set 
 * @param int16_t cell_y:			counts to set 
 * @param CellState_t state:	counts to set 
 * @return	none
 */
void Map_Set_Cells(int8_t count, int16_t cell_x, int16_t cell_y, CellState_t state)
{
	int8_t i, j;

	for ( i = -(count / 2); i <= count / 2; i++ ) 
	{
		for ( j = -(count / 2); j <= count / 2; j++ )
		{
			Map_SetCell((cell_x + i), (cell_y + j), state);
		}
	}
}

void Map_ClearCleanedCells(void)
{
	int16_t mx,my;
	Usprintf("%s(%d)\n", __FUNCTION__, __LINE__);
	for(mx = (Map_GetXMin() - 1);mx <= Map_GetXMax();mx++)
	{
		for(my = (Map_GetYMin() - 1);my <= Map_GetYMax();my++)
		{
			if(Map_GetCell(mx,my) == CLEANED)Map_SetCell(mx,my,UNCLEAN);
		}
	}
}


/*
 * Check whether a given point is an blocked or not.
 *
 * @param x	X coordinate of the give point.
 * @param y	Y coordinate of the give point.
 *
 * @return	0 if the given point is not blocked
 * 		1 if the given point is blocked
 */
uint8_t Map_Cell_Blocked(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	CellState_t cs;

	cs = Map_GetCell(x, y);
	if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY)retval = 1;
	return retval;
}

/*
 * Check a block of 3X3 is cleanable or not 
 * if any of the cells of 3X3 has a blocked means this 3x3 block is uncleanable
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not cleanable
 *		1 if the block is cleanable
      
 */
int8_t Map_IsBlockCleanable(int16_t x, int16_t y)
{
	int16_t	i, j;

	for (i = -1;i<1; i++) 
	{
		for (j=-1;j<1; j++) 
		{
			if (Map_Cell_Blocked(x + i, y + j) == 1)return 0;
		}
	}
	return 1;
}

uint8_t Map_IsBlock_Access(int16_t x,int16_t y)
{
	uint8_t retval  = 1;
	int16_t i,j;
	
	for(i = -1;((retval == 1)&& (i <=  1)); i++)
		for(j = -1;((retval == 1)&& (j <=  1)); j++)
		{
			if(Map_Cell_Blocked(x + i,y + j))retval = 0;
		}
		
	return retval;
}
uint8_t Map_IsBlock_Access2(uint8_t offset,int16_t x,int16_t y)
{
	uint8_t retval  = 1;
	int16_t i,j;
	
	for(i = -offset;((retval == 1)&& (i <=  offset)); i++)
		for(j = -offset;((retval == 1)&& (j <=  offset)); j++)
		{
			if(Map_Cell_Blocked(x + i,y + j))retval = 0;
		}
		
	return retval;
}


int8_t Map_IsBlockAllCleaned(int16_t x, int16_t y)
{
	int16_t	i, j;

	for (i = -1;i<1; i++) 
	{
		for (j=-1;j<1; j++) 
		{
			if (Map_GetCell(x + i, y + j) == UNCLEAN)return 0;
		}
	}
	return 1;
}

/*
 * Check a block is cleaned or not, a block is defined as have the same size of brush.
 *
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not cleaned
 *		1 if the block is cleaned
 */
/*
 * Check a block is uncleaned or not, a block is defined as have the same size of brush.
 * Since the brush occupies 3 cells, if there is any one of those 3 cells unclean, then the
 * block is treated as unclean.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is cleaned
 *		1 if the block is uncleaned
 */
uint8_t Map_BrushBlockUnclean(int16_t x, int16_t y)
{
	uint8_t retval = 0, count = 0;

	if (Map_GetCell(x, y) == UNCLEAN)count++;

	if (count == 1)retval = 1;

//	if(!Map_IsRobotAccessible(x, y))retval = 0;
	
	return retval;
}
/*check if the robot can move to this cell
	return 0 robot can move to this cell 
	return 1 robot can not move to this cell
*/
uint8_t Map_IsRobotAccessible(int16_t x, int16_t y)
{
	int8_t i = 0,j = 0;
  for(i = -1;i <= 1;i++)
	{
		for(j = -1;j <= 1;j++)
		{
			if (Map_GetCell(x + i, y + j) >= BLOCKED)
			{
				return 0;
			}
		}	
	}		
	return 1;
}
/*update robot pos to wall track array
	check if robot circling by overlap the walked track
	return 0 robot not move to a new cell
	return 1 robot move to a new cell and add this cell to the  g_wall_map[]
	return 2 g_wall_map[] is full 
	return 3 robot move back to some track which robot have walked throuh
*/
uint8_t Map_WallTrackUpdate(Point16_t pos)
{
	static Point16_t pre_pos={0,0};
	uint8_t wall_track_overlap_idx = 0;
	if((pos.X == pre_pos.X) && (pos.Y == pre_pos.Y))return 0;//not moving
	pre_pos = pos;
	wall_track_overlap_idx = Map_WallTrackExited(pos);
	if(wall_track_overlap_idx == 0)
	{
		if(Map_WallTrackAdd(pos))return 1;
		/*wall track full
			should stop to start point
		*/
		return 2;//full
	}
	else
	{
		if((g_wall_map_idx - wall_track_overlap_idx) < 11)
		{
			Usprintf("%s(%d):circled overlap idx = %d track idx = %d\n",__FUNCTION__, __LINE__,wall_track_overlap_idx,g_wall_map_idx);
			return 3;
		}
		Usprintf("%s(%d):overlap idx = %d track idx = %d\n",__FUNCTION__, __LINE__,wall_track_overlap_idx,g_wall_map_idx);
	}
	return 0;
}
uint8_t Map_WallTrackExited(Point16_t pos)
{
	uint8_t i = 0;	
	for(i = 0;i < g_wall_map_idx; i++)
	{
		if((g_wall_map[i].x == pos.X) && (g_wall_map[i].y == pos.Y))return (i+1);
	}
	return 0;
}
uint8_t Map_WallTrackAdd(Point16_t pos)
{
	g_wall_map[g_wall_map_idx].x = pos.X;
	g_wall_map[g_wall_map_idx].y = pos.Y;
	if(g_wall_map_idx < WALL_MAP_SIZE)
	{
		g_wall_map_idx++;
		return 1;
	}
	else 
	{
		Usprintf("%s(%d):Wall track full!\n", __FUNCTION__, __LINE__);
		return 0;
	}
}
uint8_t Map_GetWallTrackCnt(void)
{
	return g_wall_map_idx;
}
void Map_WallTrackClear(void)
{
	g_wall_map_idx = 0;
}
void Map_WallTrackShowAll(void)
{
	/*uint8_t i = 0;
	for(i = 0; i < g_wall_map_idx; i++)
	{
		Usprintf("%s(%d):idx %d (%d,%d)\n", __FUNCTION__, __LINE__,i,g_wall_map[i].x, g_wall_map[i].y);
	}
	for(i = 0; i < g_wall_map_idx; i++)
	{
		PC_NavDebug(g_wall_map[i].x, g_wall_map[i].y, 0 ,7);
	}*/
}
void Map_WallTrackSetObs(WallDir_t wall_dir)
{
	uint8_t i = 0 ,j = 0;
	int8_t k = -1;
	int16_t x1 = 0, x2 = 0, y1 = 0, y2 = 0  ;
	uint8_t wall_track_flag = 0;
//	DirectionCardinal_t dir = DIR_NONE;
	
	if((wall_dir == WALLDIR_WEST_RIGHT) || (wall_dir == WALLDIR_EAST_RIGHT))
	{
		k = 1;
	}

	for(i = 0; i < (g_wall_map_idx - 2); i++)
	{
		
		if((g_wall_map[i].x == g_wall_map[i + 1].x) && (g_wall_map[i + 1].x == g_wall_map[i + 2].x))
		{
			if(g_wall_map[i].y < g_wall_map[i + 2].y )
			{
				x1 = (3 * k);
				y1 = 0 ;
			}
			else
			{
				x1 = - (3 * k);
				y1 = 0 ;
			}
			wall_track_flag = 1;
		}
		else if((g_wall_map[i].y == g_wall_map[i + 1].y) && (g_wall_map[i + 1].y == g_wall_map[i + 2].y))
		{
			if(g_wall_map[i].x < g_wall_map[i + 2].x )
			{
				x1 = 0;
				y1 = - (3 * k);
			}
			else
			{
				x1 = 0;
				y1 = (3 * k);
			}
			wall_track_flag = 1;
		}

		
		if(wall_track_flag)
		{
			for(j = i;j < (i+3);j++)
			{
				x2 = x1 + g_wall_map[j].x;
				y2 = y1 + g_wall_map[j].y;
				if(Map_GetCell(x2,y2) == UNCLEAN)
				{
					Map_SetCell(x2,y2,BLOCKED);
				}
			}
		}
	}
}

/*connect a gap between two blocked cell at y axis direction
*/ 
void Map_FillGap(void)
{
	#ifdef ENABLE_DEBUG
	int16_t mx,my;
	CleanMode_t mode_buffer = Mode_GetMode();
	Usprintf("%s(%d)\n", __FUNCTION__, __LINE__);
	for(mx = (Map_GetXMin() - 1);mx <= Map_GetXMax();mx++)
	{
		for(my = (Map_GetYMin() - 1);my <= Map_GetYMax();my++)
		{
			if(Map_GetCell(mx,my) < BLOCKED)
			{
				if((Map_GetCell(mx,my + 1) >= BLOCKED) && (Map_GetCell(mx,my - 1) >= BLOCKED))
				{
					Map_SetCell(mx,my,BLOCKED);
				}
			}
			if(Mode_GetMode() !=  mode_buffer)return;
		}
	}
	#endif
}



int16_t Map_GetBoundaryWidth(void)
{
	return g_boundary_width;
}

int16_t Map_GetBoundaryHeight(void)
{
	return g_boundary_height;
}

void Map_AdjustBoundary(void)
{
	if(g_boundary_width_flag)
	{
		if((g_x_max - g_x_min) > (g_boundary_width - 2))
		{
			g_map_boundary.east = g_x_max - 2;
			g_map_boundary.west = g_x_min + 2;
			g_map_boundary.east_cnt = Map_CellToCount(g_map_boundary.east);
			g_map_boundary.west_cnt = Map_CellToCount(g_map_boundary.west);
			g_boundary_width_flag = 0;
			Map_StoreMapEdge();
			Map_SetXBoundary(g_map_boundary.east + 4,-g_boundary_height,g_boundary_height);
			Map_SetXBoundary(g_map_boundary.west - 5,-g_boundary_height,g_boundary_height);
			Map_LoadMapEdge();
			Usprintf("%s(%d):boundary east = %d west = %d \n", __FUNCTION__, __LINE__,g_map_boundary.east,g_map_boundary.west);
		}
	}
	if(g_boundary_height_flag)
	{
		if((g_y_max - g_y_min) > (g_boundary_height - 2))
		{
			g_map_boundary.north = g_y_max - 2 ;
			g_map_boundary.south = g_y_min + 2;
			g_map_boundary.north_cnt = Map_CellToCount(g_map_boundary.north);
			g_map_boundary.south_cnt = Map_CellToCount(g_map_boundary.south);
			g_boundary_height_flag = 0;
			Map_StoreMapEdge();
			Map_SetYBoundary(g_map_boundary.north + 4,-g_boundary_width,g_boundary_width);
			Map_SetYBoundary(g_map_boundary.south - 4,-g_boundary_width,g_boundary_width);
			Map_LoadMapEdge();
			Usprintf("%s(%d):boundary north = %d south = %d \n", __FUNCTION__, __LINE__,g_map_boundary.north,g_map_boundary.south);
		}
	}
}



void Map_SetBoundary(int16_t width,int16_t height)
{

	g_boundary_width = width;
	g_boundary_height = height;
	
	g_map_boundary.east = g_boundary_width - 4;
	g_map_boundary.west = -g_boundary_width + 4;
	g_map_boundary.north = g_boundary_height - 4;
	g_map_boundary.south = -g_boundary_height + 4;
	
	if(g_map_boundary.east > (MAP_LIMIT_HIGH - 2))g_map_boundary.east =  MAP_LIMIT_HIGH - 2;
	if(g_map_boundary.west < (MAP_LIMIT_LOW + 2))g_map_boundary.west =  MAP_LIMIT_LOW + 2;
	if(g_map_boundary.north > (MAP_LIMIT_HIGH -2))g_map_boundary.north =  MAP_LIMIT_HIGH - 2;
	if(g_map_boundary.south < (MAP_LIMIT_LOW + 2))g_map_boundary.south =  MAP_LIMIT_LOW + 2;
	
	g_map_boundary.east_cnt = Map_CellToCount(g_map_boundary.east);
	g_map_boundary.west_cnt = Map_CellToCount(g_map_boundary.west);
	g_map_boundary.north_cnt = Map_CellToCount(g_map_boundary.north);
	g_map_boundary.south_cnt = Map_CellToCount(g_map_boundary.south);
	
	Usprintf("%s(%d):boundary witdh = %d height = %d \n", __FUNCTION__, __LINE__,g_boundary_width,g_boundary_height);
	Usprintf("%s(%d):boundary east = %d west = %d \n", __FUNCTION__, __LINE__,g_map_boundary.east,g_map_boundary.west);
	Usprintf("%s(%d):boundary north = %d south = %d \n", __FUNCTION__, __LINE__,g_map_boundary.north,g_map_boundary.south);
	
	Map_SetXBoundary(g_map_boundary.east + 2 , g_map_boundary.south,g_map_boundary.north);
	Map_SetXBoundary(g_map_boundary.west - 2, g_map_boundary.south,g_map_boundary.north);
	
  Map_SetYBoundary(g_map_boundary.north + 2, g_map_boundary.west,g_map_boundary.east);
	Map_SetYBoundary(g_map_boundary.south - 2, g_map_boundary.west,g_map_boundary.east);
}

void Map_SetXBoundary(int16_t x,int16_t start_y,int16_t end_y)
{
	for(; start_y <= end_y; start_y++)
	{
		Map_SetCell(x - 1, start_y, BLOCKED_BOUNDARY);
		Map_SetCell(x, start_y, BLOCKED_BOUNDARY);
		Map_SetCell(x + 1, start_y, BLOCKED_BOUNDARY); 
	}
}

void Map_SetYBoundary(int16_t y,int16_t start_x,int16_t end_x)
{
	for(; start_x <= end_x; start_x++)
	{
		Map_SetCell(start_x, y - 1, BLOCKED_BOUNDARY);
		Map_SetCell(start_x, y, BLOCKED_BOUNDARY);
		Map_SetCell(start_x, y + 1, BLOCKED_BOUNDARY);	
	}
}


uint8_t Map_IsBoundaryWidthSet(void)
{
	return g_boundary_width_flag;
}
uint8_t Map_IsBoundaryHeightSet(void)
{
	return g_boundary_height_flag;
}

uint8_t Map_ReachBoundary(void) 
{
	if(g_boundary_width_flag)
	{
		if((g_x_max - g_x_min) > (g_boundary_width - 2))return 1;
	}
	if(g_boundary_height_flag)
	{
		if((g_y_max - g_y_min) > (g_boundary_height - 2))return 1;
	}
	return 0;
}



Point16_t Map_GetHomeCell(void)
{
	return g_home_point;
}
void Map_SetHomeCell(Point16_t home)
{
	g_home_point = home;
}

Boundary_t Map_GetBoundary(void)
{
	return g_map_boundary;
}
int16_t Map_GetBoundaryWest(void)
{
	return g_map_boundary.west;
}

void Map_StoreMapEdge(void)
{
	g_x_max_buffer = g_x_max;
	g_x_min_buffer = g_x_min;
	g_y_max_buffer = g_y_max;
	g_y_min_buffer = g_y_min;
}
void Map_LoadMapEdge(void)
{
	g_x_max = g_x_max_buffer;
	g_x_min = g_x_min_buffer;
	g_y_max = g_y_max_buffer;
	g_y_min = g_y_min_buffer;
}

Point16_t Map_GridToCell(int16_t x,int16_t y)
{
	return (Point16_t){x,y};
}
uint8_t Math_TwoCell_Equal(Point16_t start_cell,Point16_t exit_cell)
{	
	if(start_cell.X != exit_cell.X)return 0;
	if(start_cell.Y != exit_cell.Y)return 0;
	return 1;
}

uint8_t Map_IsReach_NewCell(Point16_t cur_cell,Point16_t *pre_cell)
{
	uint8_t retval = 0;
	
	if((pre_cell->X != cur_cell.X)||(pre_cell->Y != cur_cell.Y))
	{
		retval = 1;
		*pre_cell = cur_cell;
	}	
	
	return retval;
}

void Map_SetTarget_Cleaned(int16_t x,int16_t y)
{
	int8_t i  = 0,j = 0;
	
	for(i = -1;i <= 1;i++)
		for(j = -1;j <= 1;j++)
		{
			if(Map_GetCell(x+i,y+j) != CLEANED)
			{
				Map_SetCell(x+i,y+j,CLEANED);
			}
		}	
}
  
uint8_t Path_IsCell_ReachBlocked(uint8_t move_type,Point16_t cur_cell,int16_t heading)
{
	Point16_t cell = Map_GetRelativeByCell(cur_cell,heading,0,1*CELL_SIZE);//3
//	Usprintf("block cell____________:(%d,%d)",cell.X,cell.Y);
	if(move_type==SHORT_PATH)
	{
		return Map_IsBlock_Access2(1,cell.X,cell.Y)? 0: 1;
	}
	else
	{
		return Map_IsBlock_Access2(2,cell.X,cell.Y)? 0: 1;
	}	
}





