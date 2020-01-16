#include "Map.h"
#include "MyMath.h"
#include "Debug.h"
#include "Movement.h"

#define DEBUG_MSG_SIZE	1 // 20

uint16_t map_size = MAP_SIZE;

uint8_t map[MAP_SIZE][(MAP_SIZE+1) / 4];

//int16_t homeX, homeY;
extern int16_t	WheelCount_Left, WheelCount_Right;

double xCount, yCount, relative_sin, relative_cos;
uint16_t relative_theta = 3600;
int16_t xMin, xMax, yMin, yMax;
int16_t xRangeMin, xRangeMax, yRangeMin, yRangeMax;

//int16_t debugMsg[DEBUG_MSG_SIZE][3];
//uint8_t debugMsg_idx;

void Map_Initialize(void) {
	uint8_t c, d;

	for(c = 0; c < map_size; ++c) {
		for(d = 0; d < (map_size + 1) / 4; ++d) {
			map[c][d] = 0;
		}
	}

	xMin = xMax = yMin = yMax = 0;
	xRangeMin = xMin - (map_size - (xMax - xMin + 1));
	xRangeMax = xMax + (map_size - (xMax - xMin + 1));
	yRangeMin = yMin - (map_size - (yMax - yMin + 1));
	yRangeMax = yMax + (map_size - (yMax - yMin + 1));

	xCount = 0;
	yCount = 0;

	WheelCount_Left = WheelCount_Right = 0;
}

int16_t Map_GetEstimatedRoomSize(void) {
	int16_t i, j;

	i = xMax - xMin;
	j = yMax - yMin;

	if(i < j) {
		i = yMax - yMin;
		j = xMax - xMin;
	}

	if(i * 2 > j * 3) {
		i = (i * i) / 27;				//Convert no of cell to tenth m^2
	} else {
		i = (i * j) / 18;				//Convert no of cell to tenth m^2
	}

	return i;
}

int32_t Map_GetXCount(void) {
	return (int32_t)round(xCount);
}

int32_t Map_GetYCount(void) {
	return (int32_t)round(yCount);
}

int16_t Map_GetXPos(void) {
	return countToCell(xCount);
}

int16_t Map_GetYPos(void) {
	return countToCell(yCount);
}

/*------------------- Edit By ZZ --------------------*/
Point16_t Map_GetCurrentCell(void)
{
	Point16_t temp;
	temp.X = Map_GetXPos();
	temp.Y = Map_GetYPos();
	return temp;
}

Point32_t Map_GetCurrentPoint(void)
{
	Point32_t temp;
	temp.X = Map_GetXCount();
	temp.Y = Map_GetYCount();
	return temp;
}
/********************************************************/

void Map_MoveTo(double d_x, double d_y) {
	xCount += d_x;
	yCount += d_y;
}

void Map_SetPosition(double x, double y) {
	xCount = x;
	yCount = y;
}

/*
 * Map_GetCell description
 * @param id	Map id
 * @param x	Cell x
 * @param y	Cell y
 * @return	CellState
 */
CellState Map_GetCell(uint8_t id, int16_t x, int16_t y) {
	uint8_t val;

	if(x >= xRangeMin && x <= xRangeMax && y >= yRangeMin && y <= yRangeMax) {
		x += map_size + map_size / 2;
		x %= map_size;
		y += map_size + map_size / 2;
		y %= map_size;


		val = (map[x][y / 4]);

		switch(y % 4)
		{
			case 3:val = ((val>>6) & 0x03);
				break;
			case 2:val = ((val>>4) & 0x03);
				break;
			case 1:val = ((val>>2) & 0x03);
				break;
			case 0:val = ((val>>0) & 0x03);
				break;
			default:
				break;
		}
	} else {
		if(id == MAP) {
			val = BLOCKED_BOUNDARY;
//			if(debugMsg_idx < DEBUG_MSG_SIZE && (x - xRangeMax == 1 || xRangeMin - x == 1 || y - yRangeMax == 1 || yRangeMax - y == 1)) {
//				debugMsg[debugMsg_idx][0] = x;
//				debugMsg[debugMsg_idx][1] = y;
//				debugMsg[debugMsg_idx][2] = BLOCKED_BOUNDARY;
//				debugMsg_idx++;
//			}
		} else {
			val = COST_HIGH;
		}
	}
	return (CellState)val;
}

/*
 * Map_SetCell description
 * @param id		Map id
 * @param x		 Count x
 * @param y		 Count y
 * @param value CellState
 */
void Map_SetCell(uint8_t id, int32_t x, int32_t y, CellState v) {
	uint8_t val;
	int16_t ROW, COLUMN;
	uint8_t value=v;	
	if(id == MAP) {
		if(value == CLEANED) {
			ROW = cellToCount(countToCell(x)) - x;
			COLUMN = cellToCount(countToCell(y)) - y;
			if(abs(ROW) > (CELL_SIZE - 2 * 20) * CELL_COUNT_MUL / (2 * CELL_SIZE) || abs(COLUMN) > (CELL_SIZE - 2 * 20) * CELL_COUNT_MUL / (2 * CELL_SIZE)) {
			}
		}
		x = countToCell(x);
		y = countToCell(y);
	}

	if(id == MAP) {
		if(x >= xRangeMin && x <= xRangeMax && y >= yRangeMin && y <= yRangeMax) {
			if(x < xMin) {
				xMin = x;
				xRangeMin = xMin - (map_size - (xMax - xMin + 1));
				xRangeMax = xMax + (map_size - (xMax - xMin + 1));
			} else if(x > xMax) {
				xMax = x;
				xRangeMin = xMin - (map_size - (xMax - xMin + 1));
				xRangeMax = xMax + (map_size - (xMax - xMin + 1));
			}
			if(y < yMin) {
				yMin = y;
				yRangeMin = yMin - (map_size - (yMax - yMin + 1));
				yRangeMax = yMax + (map_size - (yMax - yMin + 1));
			} else if(y > yMax) {
				yMax = y;
				yRangeMin = yMin - (map_size - (yMax - yMin + 1));
				yRangeMax = yMax + (map_size - (yMax - yMin + 1));
			}

			ROW = x + map_size + map_size / 2;
			ROW %= map_size;
			COLUMN = y + map_size + map_size / 2;
			COLUMN %= map_size;

	val = map[ROW][COLUMN / 4];			
	#ifdef DEBUG_PC_MAP  
	PC_NavDebug(x,y,0,v);		
	#endif	
			
	switch(COLUMN % 4)
	{
		case 3: if(((val >> 6)&0x03)!= value)
						{
								map[ROW][COLUMN / 4] = (((value<<6)&0xc0) | (val&0x3f));
//								if(debugMsg_idx < DEBUG_MSG_SIZE) {
//									debugMsg[debugMsg_idx][0] = x;
//									debugMsg[debugMsg_idx][1] = y;
//									debugMsg[debugMsg_idx][2] = value;
//									debugMsg_idx++;
//								}
						}
			break;
						
		case 2:if(((val >> 4)&0x03)!= value)
						{
								map[ROW][COLUMN / 4] = (((value<<4)&0x30) | (val&0xcf));
//								if(debugMsg_idx < DEBUG_MSG_SIZE) {
//									debugMsg[debugMsg_idx][0] = x;
//									debugMsg[debugMsg_idx][1] = y;
//									debugMsg[debugMsg_idx][2] = value;
//									debugMsg_idx++;
//								}
						}			
			break;
						
		case 1:if(((val >> 2)&0x03)!= value)
						{
								map[ROW][COLUMN / 4] = (((value<<2)&0x0c) | (val&0xf3));
//								if(debugMsg_idx < DEBUG_MSG_SIZE) {
//									debugMsg[debugMsg_idx][0] = x;
//									debugMsg[debugMsg_idx][1] = y;
//									debugMsg[debugMsg_idx][2] = value;
//									debugMsg_idx++;
//								}
						}			
			break;
						
		case 0:if(((val >> 0)&0x03)!= value)
						{
								map[ROW][COLUMN / 4] = (((value<<0)&0x03) | (val&0xfc));
//								if(debugMsg_idx < DEBUG_MSG_SIZE) {
//									debugMsg[debugMsg_idx][0] = x;
//									debugMsg[debugMsg_idx][1] = y;
//									debugMsg[debugMsg_idx][2] = value;
//									debugMsg_idx++;
//								}
						}			
			break;
						
		default:
			break;
	}		
		}

	}
	
}

int32_t Map_GetRelativeX(uint16_t heading, int16_t offset_lat, int16_t offset_long) {
	if(heading != relative_theta) {
		if(heading == 0) {
			relative_sin = 0;
			relative_cos = 1;
		} else if(heading == 900) {
			relative_sin = 1;
			relative_cos = 0;
		} else if(heading == 1800) {
			relative_sin = 0;
			relative_cos = -1;
		} else if(heading == 2700) {
			relative_sin = -1;
			relative_cos = 0;
		} else {
			relative_sin = sin(deg2rad(heading, 10));
			relative_cos = cos(deg2rad(heading, 10));
		}
	}
	return Map_GetXCount() + (int32_t)( ( ((double)offset_long * relative_cos * CELL_COUNT_MUL) -
	                                      ((double)offset_lat	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
}

int32_t Map_GetRelativeY(uint16_t heading, int16_t offset_lat, int16_t offset_long) {
	if(heading != relative_theta) {
		if(heading == 0) {
			relative_sin = 0;
			relative_cos = 1;
		} else if(heading == 900) {
			relative_sin = 1;
			relative_cos = 0;
		} else if(heading == 1800) {
			relative_sin = 0;
			relative_cos = -1;
		} else if(heading == 2700) {
			relative_sin = -1;
			relative_cos = 0;
		} else {
			relative_sin = sin(deg2rad(heading, 10));
			relative_cos = cos(deg2rad(heading, 10));
		}
	}
	return Map_GetYCount() + (int32_t)( ( ((double)offset_long * relative_sin * CELL_COUNT_MUL) +
	                                      ((double)offset_lat *	relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
}

/*------------------------------- Edit By ZZ ---------------------------------*/
int32_t Map_GetRelativeX_ByXCount(int32_t XCount, uint16_t heading, int16_t offset_lat, int16_t offset_long) {
	if(heading != relative_theta) {
		if(heading == 0) {
			relative_sin = 0;
			relative_cos = 1;
		} else if(heading == 900) {
			relative_sin = 1;
			relative_cos = 0;
		} else if(heading == 1800) {
			relative_sin = 0;
			relative_cos = -1;
		} else if(heading == 2700) {
			relative_sin = -1;
			relative_cos = 0;
		} else {
			relative_sin = sin(deg2rad(heading, 10));
			relative_cos = cos(deg2rad(heading, 10));
		}
	}

	return XCount + (int32_t)( ( ((double)offset_long * relative_cos * CELL_COUNT_MUL) -
	                                      ((double)offset_lat	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
}

int32_t Map_GetRelativeY_ByYCount(int32_t YCount, uint16_t heading, int16_t offset_lat, int16_t offset_long) {
	if(heading != relative_theta) {
		if(heading == 0) {
			relative_sin = 0;
			relative_cos = 1;
		} else if(heading == 900) {
			relative_sin = 1;
			relative_cos = 0;
		} else if(heading == 1800) {
			relative_sin = 0;
			relative_cos = -1;
		} else if(heading == 2700) {
			relative_sin = -1;
			relative_cos = 0;
		} else {
			relative_sin = sin(deg2rad(heading, 10));
			relative_cos = cos(deg2rad(heading, 10));
		}
	}

	return YCount + (int32_t)( ( ((double)offset_long * relative_sin * CELL_COUNT_MUL) +
	                                      ((double)offset_lat *	relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
}
/**********************************************************************************/

int16_t nextXID(uint16_t heading, int16_t offset_lat, int16_t offset_long) {
	return Map_GetXPos() + offset_long * round(cos(deg2rad(heading, 10))) - offset_lat * round(sin(deg2rad(heading, 10)));
}

int16_t nextYID(uint16_t heading, int16_t offset_lat, int16_t offset_long) {
	return Map_GetYPos() + offset_long * round(sin(deg2rad(heading, 10))) + offset_lat * round(cos(deg2rad(heading, 10)));
}

int32_t cellToCount(int16_t i) {
	return i * CELL_COUNT_MUL;
}

int16_t countToCell(double count) {
	if(count < -CELL_COUNT_MUL_1_2) {
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL - 1;
	} else {
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL;
	}
}

Point32_t Map_CellToPoint( Point16_t cell ) {
	Point32_t pnt;
	pnt.X = cellToCount(cell.X);
	pnt.Y = cellToCount(cell.Y);
	return pnt;
}

Point16_t Map_PointToCell( Point32_t pnt ) {
	Point16_t cell;
	cell.X = countToCell(pnt.X);
	cell.Y = countToCell(pnt.Y);
	return cell;
}

void Map_Set_Cells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state)
{
	int8_t i, j;

	for ( i = -(count / 2); i <= count / 2; i++ ) {
		for ( j = -(count / 2); j <= count / 2; j++ ) {
			Map_SetCell(MAP, cellToCount(cell_x + i), cellToCount(cell_y + j), state);
		}
	}
}

#ifdef VIRTUAL_WALL
void Map_SetXCount(int32_t data) {
	xCount = data;
}

void Map_SetYCount(int32_t data) {
	yCount = data;
}

void Draw_VirtualWall_Line(Point32_t start,Point32_t end,CellState value)
{
	Point16_t start_cell,end_cell,temp_cell;
	uint16_t Temp_Target_Course=0, Target_Course=0;
	uint16_t i=0;
	int32_t x,y,temp_x,temp_y;
		
	x = Map_GetXCount();
	y = Map_GetYCount();
	
	start_cell.X = countToCell(start.X);
	start_cell.Y = countToCell(start.Y);
	end_cell.X = countToCell(end.X);
	end_cell.Y = countToCell(end.Y);
	
	Map_SetXCount(start.X);
	Map_SetYCount(start.Y);
		
	if(!((start_cell.X==end_cell.X)&&(start_cell.Y==end_cell.Y)))
	{
		Target_Course = course2dest(start.X, start.Y, end.X, end.Y);
		Temp_Target_Course = Target_Course;
		while(1)
		{
			temp_x = Map_GetXCount();
			temp_y = Map_GetYCount();
			
			Target_Course = course2dest(temp_x, temp_y, end.X, end.Y);
			#ifdef RIGHT_WALL
			temp_x = Map_GetRelativeX(Target_Course, -CELL_SIZE, 0);
			temp_y = Map_GetRelativeY(Target_Course, -CELL_SIZE, 0);	
			#endif
			Map_SetCell(MAP, temp_x, temp_y, value);
			temp_cell.X = countToCell(temp_x);
			temp_cell.Y = countToCell(temp_y);			

			if((TwoPointsDistance(end_cell.X, end_cell.Y, temp_cell.X, temp_cell.Y)<3))
			{			
				break;
			}	
			#ifdef RIGHT_WALL
			temp_x = Map_GetRelativeX(Target_Course, -CELL_SIZE_2, 0);
			temp_y = Map_GetRelativeY(Target_Course, -CELL_SIZE_2, 0);
			#endif			
			Map_SetCell(MAP, temp_x, temp_y, value);
			temp_cell.X = countToCell(temp_x);
			temp_cell.Y = countToCell(temp_y);			

			if((TwoPointsDistance(end_cell.X, end_cell.Y, temp_cell.X, temp_cell.Y)<3))
			{			
				break;
			}
			#ifdef RIGHT_WALL
			temp_x = Map_GetRelativeX(Target_Course, -CELL_SIZE_3, 0);
			temp_y = Map_GetRelativeY(Target_Course, -CELL_SIZE_3, 0);	
			#endif			
			Map_SetCell(MAP, temp_x, temp_y, value);
			temp_cell.X = countToCell(temp_x);
			temp_cell.Y = countToCell(temp_y);

			if((TwoPointsDistance(end_cell.X, end_cell.Y, temp_cell.X, temp_cell.Y)<3))
			{			
				break;
			}
			#ifdef RIGHT_WALL//not edit
			temp_x = Map_GetRelativeX(Target_Course, 0, CELL_SIZE);
			temp_y = Map_GetRelativeY(Target_Course, 0, CELL_SIZE);	
			#endif
			Map_SetCell(MAP, temp_x, temp_y, value);			

			temp_cell.X = countToCell(temp_x);
			temp_cell.Y = countToCell(temp_y);			

			if((TwoPointsDistance(end_cell.X, end_cell.Y, temp_cell.X, temp_cell.Y)<3))
			{			
				break;
			}		
			Map_SetXCount(temp_x);
			Map_SetYCount(temp_y);
		}
	}	
	
	start.X = end.X;
	start.Y = end.Y;
	
	start_cell.X = countToCell(start.X);
	start_cell.Y = countToCell(start.Y);
	
	while(1)
	{
		temp_x = Map_GetXCount();
		temp_y = Map_GetYCount();
		#ifdef RIGHT_WALL//not edit
		temp_x = Map_GetRelativeX(Temp_Target_Course, 0, CELL_SIZE);
		temp_y = Map_GetRelativeY(Temp_Target_Course, 0, CELL_SIZE);
		#endif
		temp_cell.X = countToCell(temp_x);
		temp_cell.Y = countToCell(temp_y);
		end.X = temp_x;
		end.Y = temp_y;
		if((TwoPointsDistance(start_cell.X, start_cell.Y, temp_cell.X, temp_cell.Y)>5))
		{			
			break;
		}	
		i++;
		if(i>100)break;
		
		Map_SetXCount(temp_x);
		Map_SetYCount(temp_y);		
	}
	
	if(value==BLOCKED_BOUNDARY)
	{
		end_cell.X = countToCell(end.X);
		end_cell.Y = countToCell(end.Y);
		
		Map_SetXCount(start.X);
		Map_SetYCount(start.Y);
		
		if(!((start_cell.X==end_cell.X)&&(start_cell.Y==end_cell.Y)))
		{
			Target_Course = course2dest(start.X, start.Y, end.X, end.Y);
				
			while(1)
			{
				temp_x = Map_GetXCount();
				temp_y = Map_GetYCount();
				
				Target_Course = course2dest(temp_x, temp_y, end.X, end.Y);
				#ifdef RIGHT_WALL
				temp_x = Map_GetRelativeX(Target_Course, -CELL_SIZE, 0);
				temp_y = Map_GetRelativeY(Target_Course, -CELL_SIZE, 0);		
				#endif
				Map_SetCell(MAP, temp_x, temp_y, value);	
				temp_cell.X = countToCell(temp_x);
				temp_cell.Y = countToCell(temp_y);			
			if((TwoPointsDistance(end_cell.X, end_cell.Y, temp_cell.X, temp_cell.Y)<3))
			{			
				break;
			}				
				#ifdef RIGHT_WALL
				temp_x = Map_GetRelativeX(Target_Course, -CELL_SIZE_2, 0);
				temp_y = Map_GetRelativeY(Target_Course, -CELL_SIZE_2, 0);		
				#endif
				Map_SetCell(MAP, temp_x, temp_y, value);
				temp_cell.X = countToCell(temp_x);
				temp_cell.Y = countToCell(temp_y);			
			if((TwoPointsDistance(end_cell.X, end_cell.Y, temp_cell.X, temp_cell.Y)<3))
			{			
				break;
			}
				#ifdef RIGHT_WALL
				temp_x = Map_GetRelativeX(Target_Course, -CELL_SIZE_3, 0);
				temp_y = Map_GetRelativeY(Target_Course, -CELL_SIZE_3, 0);
				#endif
				Map_SetCell(MAP, temp_x, temp_y, value);
				temp_cell.X = countToCell(temp_x);
				temp_cell.Y = countToCell(temp_y);	
			if((TwoPointsDistance(end_cell.X, end_cell.Y, temp_cell.X, temp_cell.Y)<3))
			{			
				break;
			}
				#ifdef RIGHT_WALL//not edit
				temp_x = Map_GetRelativeX(Target_Course, 0, CELL_SIZE);
				temp_y = Map_GetRelativeY(Target_Course, 0, CELL_SIZE);
				#endif
				Map_SetCell(MAP, temp_x, temp_y, value);
				
				temp_cell.X = countToCell(temp_x);
				temp_cell.Y = countToCell(temp_y);			

			if((TwoPointsDistance(end_cell.X, end_cell.Y, temp_cell.X, temp_cell.Y)<3))
			{			
				break;
			}	

				Map_SetXCount(temp_x);
				Map_SetYCount(temp_y);	
			}
		}
	}
					
	Map_SetXCount(x);
	Map_SetYCount(y);	
	Map_SetCell(MAP, x, y, CLEANED);
}

void Map_Set_Cells_IgnoreCells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state, CellState IgnoreCellState)
{
	int8_t i, j;

	for ( i = -(count / 2); i <= count / 2; i++ ) {
		for ( j = -(count / 2); j <= count / 2; j++ ) {
			if(Map_GetCell(MAP , cell_x + i , cell_y + j) != IgnoreCellState)
				Map_SetCell(MAP, cellToCount(cell_x + i), cellToCount(cell_y + j), state);
		}
	}
}
#endif


