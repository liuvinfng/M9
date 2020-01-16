/**
******************************************************************************
* @file    AI Cleaning Robot
* @author  Wfliu
* @version V1.0
* @date    17-Nov-2018
* @brief   Move near the wall on the left in a certain distance
******************************************************************************
* <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "Charge.h"
#include "Display.h"
#include "Home.h"
#include "Movement.h"
#include "Rcon.h"
#include "Speaker.h"
#include "Spot.h"
#include "TouchPad.h"
#include "USART.h"
#include "Gyro.h"
#include "Map.h"
#include "MyMath.h"
#include "PathPlanning.h"
#include "Zone.h"
#include "CorMove.h"
#include "WallFollowMulti.h"

/*---------------------Old Alignment------------------------*/
#define POSITIONS_COUNT	(MAP_SIZE * 1)
#define POSITIONS_WALL_COUNT	(12)
#define POSITIONS_WALL_ERROR	(5)

/*---------------------Alignment------------------------*/
//Alignment array data size
#define ALIGNMENT_COUNT		(ZONE_SIZE * 8)

//Segment size, if larger than MIN_SEGMENT_SIZE, it will calculate the optimal line
#define MIN_SEGMENT_SIZE	(10)	//(ZONE_DIMENSION_CELL / 2)

//Delta angle
//Maximum delta angle for data segment, if gyro larger than this value, it will segment the wall follow data
#define MAX_SEGMENT_DELTA_ANGLE	(100)
//Maximum delta angle for line matching
#define MAX_LINE_MATCH_DELTA_ANGLE	(100)
//Maximum delta angle for angle matching
#define MAX_ANGLE_MATCH_DELTA_ANGLE	(50)

//Maximum alignment offset cell count, if the offset is smaller than MAX_ALIGNMENT_CELL_COUNT * CELL_COUNT_MUL,
//then it will apply the offset
#define MAX_ALIGNMENT_CELL_COUNT	(3)

//For wall angle offset
#define MAX_WALL_DELTA_ANGLE	(200)

//Alignment data step for collect wall follow data, uint is mm
#define ALIGNMENT_DATA_STEP	(260)//268=50*CELL_COUNT_MUL / CELL_SIZE

//Line data size
#define LINES_SIZE	(10)

//Max error encoder count of findLine
#define ALIGNMENT_POLYFIT_MAX_ERROR			(20)

//Max findLine try time
#define ALIGNMENT_POLYFIT_MAX_TIME		(5)
/*--------------------------End Alignment----------------------*/


/************************************************************************
 * ZONE_WALLFOLLOW ENABLE
 ************************************************************************/
extern int16_t xMin_Temp,xMax_Temp,yMin_Temp,yMax_Temp;
extern ZoneWallFollowType zones[ZONE_MAX];
extern uint8_t zones_cnt;

#ifdef ZONE_WALLFOLLOW
//Return type: 0: Exit; 1: Entrance
static uint8_t returnCellType = 0;

//Flags
static uint8_t isWallFollowTooFar = 0, isNeedToMarkBoundary = 1;

//Mini room
static uint8_t isMiniRoom = 0;
static int16_t isNeedCheckMiniRoom = 0;

/*---------------------Alignment------------------------*/
//Alignment data
Point32_t positionAlignment[ALIGNMENT_COUNT] = {{0, 0}};
uint16_t angleAlignment[ALIGNMENT_COUNT] = {0};
uint16_t alignmentPtr = 0;

Point32_t point32AryTmp[ALIGNMENT_COUNT/2] = {{0, 0}};
uint16_t u16AryTmp[ALIGNMENT_COUNT/2] = {0};

//Lines of first dataset
static LineABC lineABCFirst[LINES_SIZE] = {{0.0, 0.0, 0.0}};
//static Point32_t signPntFirst[LINES_SIZE] = {{0, 0}};
static uint8_t lineABCFirstPtr = 0;

//Offset data
static int16_t angleOffset = 0;

//Calculate heading using alignment data
static uint16_t angles[POSITIONS_COUNT] = {0};

//Use in alignment functions
int16_t lineAngle[LINES_SIZE] = {0}, groupAngle[LINES_SIZE] = {0}, groupSize[LINES_SIZE] = {0};
int16_t angleOffsetTmp[LINES_SIZE] = {0};
Point32_t positionFirstIntersecPntTmp[LINES_SIZE-1], positionSecondIntersecPntTmp[LINES_SIZE-1];
double angleTmp1[LINES_SIZE-1], angleTmp2[LINES_SIZE-1];
int16_t tmpAngleArray[LINES_SIZE];
/*--------------------------End Alignment----------------------*/

Point16_t wallFollowCell = {0, 0};
//Distance
//Note this distance is the count of 4 cell, real cell distance is 4 * zoneWallFollowDistance
uint16_t zoneWallFollowDistanceCount = 0;
uint16_t OutTrapWallFollowDistanceCount = 0;

//Up, down, right and left
static Point16_t wallUppestCell, wallDownestCell, wallRightestCell, wallLeftestCell;
static uint16_t angleUppestCell, angleDownestCell, angleRightestCell, angleLeftestCell;
extern int16_t xMin, xMax, yMin, yMax;
#endif
/************************************************************************
 * ZONE_WALLFOLLOW ENABLE End
 ************************************************************************/


extern volatile Point16_t path_blocks[ZONE_SIZE * 8];
extern volatile int16_t path_blocks_angle[ZONE_SIZE * 8];
extern volatile uint16_t path_blocks_size;

/************************************************************************
 * Normal
 ************************************************************************/
//Find wall old method
static Point32_t positions_wall[POSITIONS_WALL_COUNT] = {{0, 0}};
static uint8_t positions_wall_index = 0;
static int16_t positions_start_index = 0;
static int16_t positions_index = 0;
static Point16_t positions[POSITIONS_COUNT] = {{0, 0}};

//Find wall method using alignment data
uint16_t rub1[POSITIONS_WALL_COUNT] = {0};
Point32_t posAry[POSITIONS_WALL_COUNT] = {{0, 0}};


static MapFindWallType wallFoundStatus = Map_Find_Wall_Not_Found;
static int8_t	started;
static uint8_t isDirectWallFollow = 0, isIsolatedWallFollow = 0;
//Map_Wall_Follow_Distance
volatile int32_t Map_Wall_Follow_Distance = 0;

//Follow type
MapWallFollowType g_follow_type;

//Timer
uint32_t escape_trapped_timer;
uint32_t escape_trapped_timer_ZZ;
uint32_t wall_follow_to_zone_cell_timer;
extern volatile uint32_t Work_Timer;


extern volatile uint8_t CorMove_Rcon;
extern volatile uint8_t WallFollow_Point_Exit;
extern uint8_t	from_station;

#ifdef VIRTUAL_WALL
volatile uint8_t WallFollowMulti_End_VirtualWall=0;
volatile uint8_t WallFollowMulti_Pass_VirtualWall=0;
Point32_t Start_WallFollowMulti_VirtualWall_Point,End_WallFollowMulti_VirtualWall_Point;
#endif

/*----------------- Edit By ZZ --------------------------*/
uint8_t Found_Charger=0;
Point16_t OutTrapStartCell = {0, 0};
uint8_t WallFollowARoundFlag=0;
uint32_t SamePositionTime=0;
uint8_t SendFlag_wallfollow=1;
uint8_t SendFlag_OutTrap = 1;
Point16_t StartCell_WallFollow[6];
uint16_t StartAngle_WallFollow[6];
/***********************************************************/

//ACTION TYPE
typedef enum {
	ACTION_NONE	= 0x01,
	ACTION_GO	  = 0x02,
	ACTION_BACK	= 0x04,
	ACTION_LT	  = 0x08,
	ACTION_RT	  = 0x10,
} ActionType;

//MFW setting
static const MapWallFollowSetting MFW_Setting[6]= {{1200, 250, 150 },
							{1200, 250, 150},
							{1200, 250, 150},
							{1200, 250, 70},
							{1200, 250, 150},
							{1200, 250, 150},};

extern int16_t WheelCount_Left, WheelCount_Right;

/************************************************************************
 * Normal End
 ************************************************************************/

void WFM_move_back(uint16_t dist);
void WFM_wall_move_back(uint16_t distance);
void WFM_update_position(uint16_t heading_0, int16_t heading_1, int16_t left, int16_t right);


#define STOP_BRIFLY	{				\
				Wheel_Stop();		\
				delay(800);		\
				WFM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1), WheelCount_Left, WheelCount_Right);	\
			}

void WFM_update_position(uint16_t heading_0, int16_t heading_1, int16_t left, int16_t right) 
{
#ifdef ZONE_WALLFOLLOW
	if (g_follow_type == Map_Wall_Follow_Left_Zone) {
		if ( isNeedToMarkBoundary == 1) 
		{
			#ifdef RIGHT_WALL
			Map_SetCell(MAP, Map_GetRelativeX(heading_0, -CELL_SIZE_2, 0), Map_GetRelativeY(heading_0, -CELL_SIZE_2, CELL_SIZE), BLOCKED);
			#endif
		}
	} else if ( g_follow_type == Map_Wall_Follow_Left_Target && Get_RWall_ADC() > 200  ) {//&&(Get_OBS_Status() & Status_Right_OBS) 
	          
		#ifdef RIGHT_WALL
		Map_SetCell(MAP, Map_GetRelativeX(heading_0, -CELL_SIZE_2, CELL_SIZE), Map_GetRelativeY(heading_0, -CELL_SIZE_2, CELL_SIZE), BLOCKED);
		Map_SetCell(MAP, Map_GetRelativeX(heading_0, CELL_SIZE_2, CELL_SIZE), Map_GetRelativeY(heading_0, CELL_SIZE_2, CELL_SIZE), CLEANED);
		#endif
	} else if ((g_follow_type == Map_Wall_Follow_Escape_Trapped_ZZ) || (g_follow_type == Map_Wall_Follow_To_Zone_Entrance) || (g_follow_type == Map_Wall_Follow_To_Zone_Exit)) {
		if ( isNeedToMarkBoundary == 1  ) 
		{
			#ifdef RIGHT_WALL			
			Map_SetCell(MAP, Map_GetRelativeX(heading_0, -CELL_SIZE_2, CELL_SIZE), Map_GetRelativeY(heading_0, -CELL_SIZE_2, CELL_SIZE), BLOCKED);
			#endif
		}
	}
#endif
}

int16_t WFM_check_positions(void)
{
	int16_t	i, retval = 0;
	if (positions_index < 50) {
		return retval;
	}
	for (i = 1; i < positions_index / 2; i++) {
		if (positions[positions_index - 1].X  == positions[i].X && positions[positions_index - 1].Y == positions[i].Y) {
			break;
		}
	}

	if (i == positions_index) {
		return retval;
	}
	if (i < positions_index / 2) {
		retval = 1;
	}
	return retval;
}

//Initial
void Map_Wall_Follow_Initialize(void) {
	int16_t i = 0;

	wallFoundStatus = Map_Find_Wall_Not_Found;
	isDirectWallFollow = 0;

	for (i = 0; i < POSITIONS_WALL_COUNT; ++i ) {
		positions_wall[i].X = 0;
		positions_wall[i].Y = 0;
	}
	positions_wall_index = 0;

	for (i = 0; i < POSITIONS_COUNT; ++i ) {
		positions[i].X = 0;
		positions[i].Y = 0;
#ifdef ZONE_WALLFOLLOW
		angles[i] = 0;
#endif
	}
	positions_start_index = 0;
	positions_index = 0;

#ifdef ZONE_WALLFOLLOW
	returnCellType = 0;
	isIsolatedWallFollow = 0;
	isWallFollowTooFar = 0;
	isMiniRoom = 0;
	isNeedCheckMiniRoom = 1;
	isNeedToMarkBoundary = 1;

	wallFollowCell.X = 0;
	wallFollowCell.Y = 0;

	zoneWallFollowDistanceCount = 0;
	OutTrapWallFollowDistanceCount = 0;

	wallUppestCell.X = 0; wallUppestCell.Y = 0;
	wallDownestCell.X = 0; wallDownestCell.Y = 0;
	wallRightestCell.X = 0; wallRightestCell.Y = 0;
	wallLeftestCell.X = 0; wallLeftestCell.Y = 0;

	angleUppestCell = 0;
	angleDownestCell = 0;
	angleRightestCell = 0;
	angleLeftestCell = 0;
#endif
}

//Fit lines
uint8_t findLine( Point32_t *point32AryPtr, uint16_t *u16AryPtr, uint16_t point32Size,
                  uint8_t mode, LineABC *lineabcTmp, Point32_t *cdnTmp, int16_t *int16Tmp, double *angleVec ) {
	uint8_t i = 0, j = 0, retval = 0, fitTimes = 0, meetRequirement = 0;
	double  tmp,  xys = 0.0, xs = 0.0, ys = 0.0, x2s = 0.0, y2s = 0.0;
	uint32_t fitErr;
	uint16_t k;

	USPRINTF("Begin to find lines...\n")
	while ( point32Size > MIN_SEGMENT_SIZE && fitTimes < ALIGNMENT_POLYFIT_MAX_TIME ) {
		fitTimes++;
		USPRINTF("Times of fitting line: %d\n", fitTimes);

		//Formula
		//Line: y = a * x + b
		//a = ( Sum(x * y) - Sum(x) * Sum(y) / N ) / ( Sum(x^2) - Sum(x)^2 / N );

		xys = 0.0; xs = 0.0; ys = 0.0; x2s = 0.0; y2s = 0.0;

	 	//Calclate sum of x, y x*y, x*x and y*y
		for ( k = 0; k < point32Size; ++k ) {
			xs  += (double)point32AryPtr[k].X;
			ys  += (double)point32AryPtr[k].Y;
			xys += (double)point32AryPtr[k].X * point32AryPtr[k].Y;
			x2s += (double)point32AryPtr[k].X * point32AryPtr[k].X;
			y2s += (double)point32AryPtr[k].Y * point32AryPtr[k].Y;
		}

		//If not vertical
		tmp = ( xys / xs - ys / point32Size ) / ( x2s / xs - xs / point32Size );
		if ( tmp <= 1.0 && tmp >= -1.0 ) {
			USPRINTF("%s %d FindLine Horizontal!\n", __FUNCTION__, __LINE__);
			lineabcTmp->A = - tmp;
			lineabcTmp->B = 1.0;
			lineabcTmp->C = - ys / point32Size - lineabcTmp->A / point32Size * xs;

			USPRINTF("%s %d: Line: A: %d\tB: %d\tC: %d\n", __FUNCTION__, __LINE__,
							 (int32_t)(lineabcTmp->A * 1000), (int32_t)(lineabcTmp->B * 1000), (int32_t)(lineabcTmp->C * 1000));
		} else if ( tmp > 1.0 || tmp < -1.0 ) {
			USPRINTF("%s %d FindLine Vertical!\n", __FUNCTION__, __LINE__);
			lineabcTmp->A = 1.0;
			lineabcTmp->B = - ( xys / ys - xs / point32Size ) / ( y2s / ys - ys / point32Size );
			lineabcTmp->C = - xs / point32Size - lineabcTmp->B / point32Size * ys;

			USPRINTF("%s %d: Line: A: %d\tB: %d\tC: %d\n", __FUNCTION__, __LINE__,
							 (int32_t)(lineabcTmp->A * 1000), (int32_t)(lineabcTmp->B * 1000), (int32_t)(lineabcTmp->C * 1000));
		}
		//Calculate fit error
		meetRequirement = 1;
		USPRINTF("Fit error of lines: Size: %d\n", point32Size);
		for ( j = 0; j < point32Size; ++j ) {
			fitErr = (uint32_t)( absolute( lineabcTmp->A * (double)(point32AryPtr[j].X) +
			                               lineabcTmp->B * (double)(point32AryPtr[j].Y) + lineabcTmp->C ) /
			                        sqrt( (lineabcTmp->A) * (lineabcTmp->A) + (lineabcTmp->B) * (lineabcTmp->B) ) );

			USPRINTF("Idx: %d\tError: %d\n", j, fitErr);
#ifdef ENABLE_DEBUG
			delay(10);
#endif

			//Delete this point if error too large
			//Mode 1: Will dynamic calculate the error, delete some large error point and find the line again until it converaged
			//Mode Ohters: Only fit the line use the points. If it is larger than the max error, then no line fits these data
			if ( fitErr > ALIGNMENT_POLYFIT_MAX_ERROR ) {
				if ( mode == 1 ) {
					for ( i = j; i < point32Size - 1; ++i ) {
						point32AryPtr[i] = point32AryPtr[i + 1];
						u16AryPtr[i] = u16AryPtr[i + 1];
					}
					USPRINTF("Remove Idx: %d\n", j);
					point32Size--;
					meetRequirement = 0;
					--j;
				} else {
					meetRequirement = 0;
					break;
				}
			}
		}

		//all error is smaller enough
		if ( meetRequirement == 1 ) {
			retval = 1;
			break;
		}

		//If mode is not 1, then break
		if ( mode != 1 ) {
			break;
		}
	}
	return retval;
}

MapFindWallType WFM_find_wall(void)
{
	uint8_t i, isFind = 0;
	int16_t rub3;
	double angle;
	LineABC lineabcTmp;
	Point32_t rub2;

	MapFindWallType state = Map_Find_Wall_Not_Found;
	if (positions_index == MAP_FIND_WALL_DISTANCE) {
		USPRINTF("%s %d: find wall distance is over: %d (%d)\n", __FUNCTION__, __LINE__, positions_index, MAP_FIND_WALL_DISTANCE);
		state = Map_Find_Wall_Over_Distance;
	}
	else if (countToCell(positions_wall[positions_wall_index - 1].X) != Map_GetXPos() ||
	         countToCell(positions_wall[positions_wall_index - 1].Y) != Map_GetYPos()) {

		//Clone data
		if ( positions_wall_index >= POSITIONS_WALL_COUNT ) {
			for ( i = 0; i < POSITIONS_WALL_COUNT - 1; ++i ) {
				positions_wall[i].X = positions_wall[i + 1].X;
				positions_wall[i].Y = positions_wall[i + 1].Y;
			}
			positions_wall_index = POSITIONS_WALL_COUNT - 1;
		}

		positions_wall[positions_wall_index].X = Map_GetXCount();
		positions_wall[positions_wall_index].Y = Map_GetYCount();
		positions_wall_index++;

		//Initial
		for ( i = 0; i < POSITIONS_WALL_COUNT; ++i ) {
			rub1[i] = 0;
			posAry[i] = positions_wall[i];
		}

		if (positions_index - positions_start_index > POSITIONS_WALL_COUNT) {
			isFind = findLine( posAry, rub1, POSITIONS_WALL_COUNT, 1, &lineabcTmp, &rub2, &rub3, &angle );

			if ( isFind == 1 ) {
				USPRINTF("%s %d Line found!\n", __FUNCTION__, __LINE__);

				state = Map_Find_Wall_Found;

				USPRINTF("%s %d Angle: %d\n", __FUNCTION__, __LINE__, (int16_t)(angle * 1800 / PI));
				CM_SetGyroOffset( (int16_t)(angle * 1800 / PI) );
			}
		}
	}
	return state;
}

#ifdef ZONE_WALLFOLLOW
/**
 * Find wall based on alignment data
 * @param  lineABC input lines
 * @return         return MapFindWallType: Map_Find_Wall_Not_Found,
 *                                         Map_Find_Wall_Timeout,
 *                                         Map_Find_Wall_Over_Distance,
 *                                         Map_Find_Wall_Found,
 *                                         Map_Find_Wall_No_Need_To_Find,
 */
MapFindWallType WFM_FindWallBasedOnAlignment( LineABC *lineABC )
{
	MapFindWallType state = Map_Find_Wall_Not_Found;
	uint8_t lineSize = 0, i = 0, j = 0, groupAngleSize = 0, isFind = 0, maxSize = 0;
	int16_t tmpAngle = 0, offsetAngle = 0;

	//Check which data
	if ( lineABC == lineABCFirst ) {
		lineSize = lineABCFirstPtr;
	} 
	//No lines
	if ( lineSize == 0 ) {
		return state;
	}

	//Initial group data
	for ( i = 0; i < LINES_SIZE; ++i ) {
		groupAngle[i] = 0;
		groupSize[i] = 0;
	}

	//Find all lines angle
	for ( i = 0; i < lineSize; ++i ) {
		isFind = 0;
		lineAngle[i] = (int16_t)( LineAngle(lineABC[i], 1) * 1800 / PI );
		//Group lines angle
		for ( j = 0; j < groupAngleSize; ++j ) {
			tmpAngle = groupAngle[j] - lineAngle[i];

			USPRINTF("%s %d Group %d angle: %d", __FUNCTION__, __LINE__, j, groupAngle[j]);
			USPRINTF("%s %d Delta line angle: %d", __FUNCTION__, __LINE__, tmpAngle);

			if ( abs(tmpAngle) < 50 ) {
				USPRINTF("%s %d Case 1: < 50\n", __FUNCTION__, __LINE__);
				USPRINTF("%s %d Line angle: %d", __FUNCTION__, __LINE__, lineAngle[i]);
				groupAngle[j] = ( groupAngle[j] + lineAngle[i] ) / 2;
				++groupSize[j];
				isFind = 1;
			} else if ( tmpAngle < 950 && tmpAngle > 850 ) {
				USPRINTF("%s %d Case 2: < 950 && > 850\n", __FUNCTION__, __LINE__);
				USPRINTF("%s %d Line angle: %d", __FUNCTION__, __LINE__, lineAngle[i]);
				groupAngle[j] = ( groupAngle[j] + lineAngle[i] + 900 ) / 2;
				++groupSize[j];
				isFind = 1;
			} else if ( tmpAngle > -950 && tmpAngle < -850 ) {
				USPRINTF("%s %d Case 3: > -950 && < -850\n", __FUNCTION__, __LINE__);
				USPRINTF("%s %d Line angle: %d", __FUNCTION__, __LINE__, lineAngle[i]);
				groupAngle[j] = ( groupAngle[j] + lineAngle[i] - 900 ) / 2;
				++groupSize[j];
				isFind = 1;
			}
		}
		//Cannot find, create a new group
		if ( isFind == 0 ) {
			groupAngle[groupAngleSize] = lineAngle[i];
			++groupAngleSize;
			groupSize[groupAngleSize] = 1;
		}
	}
	//Find the min
	maxSize = groupSize[0];
	offsetAngle = groupAngle[0];
	USPRINTF("%s %d Group 0 angle: %d", __FUNCTION__, __LINE__, maxSize);
	for ( i = 1; i < groupAngleSize; ++i ) {
		USPRINTF("%s %d Group %d angle: %d", __FUNCTION__, __LINE__, j, groupAngle[j]);
		if ( groupSize[i] > maxSize ) {
			maxSize = groupSize[i];
			offsetAngle = groupAngle[i];
		}
	}
	USPRINTF("%s %d Offset angle: %d", __FUNCTION__, __LINE__, offsetAngle);
	CM_SetGyroOffset( offsetAngle );
	return Map_Find_Wall_Found;
}
#endif

MapEscapeTrappedType WFM_check_trapped(void)
{
	int8_t		val;
	uint32_t	left_speed, right_speed;
	static int16_t	current_x = 0, current_y = 0;

	MapEscapeTrappedType state = Map_Escape_Trapped_Trapped;

	if (abs(current_x - Map_GetXPos()) >= 2 || abs(current_y - Map_GetYPos()) >= 2) {
		left_speed = Get_LeftWheel_Step();
		right_speed = Get_RightWheel_Step();
		STOP_BRIFLY;

		val = path_escape_trapped();
		if (val == 1) {
			state = Map_Escape_Trapped_Escaped;
		} else if (val == -1) {
			state = Map_Escape_Trapped_Timeout;
		}
		current_x = Map_GetXPos();
		current_y = Map_GetYPos();
		Set_Wheel_Step(left_speed, right_speed);
	}

	return state;
}

/*---------------------------------------------------------*/
MapEscapeTrappedType WFM_check_trapped_ZZ(void)
{
	int32_t x,y;
	int8_t i;
	/*--------------------- Edit By ZZ -----------------------*/
	if((Work_Timer % 2 == 0) && (SendFlag_OutTrap))
	{
		SendFlag_OutTrap = 0;
		USPRINTF("CurrentCell:(%d,%d)  DistanceCount:%d  Direction:%d\n",Map_GetXPos(),Map_GetYPos(),OutTrapWallFollowDistanceCount,Map_Wall_Follow_GetWallFollowDirection());
	}
	else if(Work_Timer % 2 != 0)
		SendFlag_OutTrap = 1;
	//绕了一圈也没发现未清扫区域
	if ( OutTrapWallFollowDistanceCount > 20 &&
			 Map_Wall_Follow_GetWallFollowDirection() == 2 &&
			 ((TwoPointsDistance( OutTrapStartCell.X, OutTrapStartCell.Y, Map_GetXPos(), Map_GetYPos() ) < 3) ||
				WFM_IsWallFollowSamePointAndHeading(Map_GetCurrentCell(), Gyro_GetAngle(0)) == 1)
		 ) 
	{
		STOP_BRIFLY;
		USPRINTF("StartCell:(%d,%d)\n",OutTrapStartCell.X,OutTrapStartCell.Y);
		USPRINTF("%s %d Wall follow in mini room! End after cleaning\n", __FUNCTION__, __LINE__);	
		WallFollowARoundFlag = 1;
		return Map_Escape_Trapped_Escaped;
	}
	
	for(i = -1; i <= 1;i++)
	{
		#ifdef RIGHT_WALL
		CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE_2, CELL_SIZE*i, &x, &y);
		#endif
		if (Map_GetCell(MAP, countToCell(x), countToCell(y)) != UNCLEAN)
			return Map_Escape_Trapped_Trapped;
	}
	/**********************************************************/	
	USPRINTF("%s %d:Program Run at here!\n",__FUNCTION__,__LINE__);
	return Map_Escape_Trapped_Escaped;
}
/*************************************************************/

int8_t WFM_update()
{
#ifdef ZONE_WALLFOLLOW
	int16_t i = 0;
	double sinTmp = 0.0, cosTmp = 0.0;
#endif
	WFM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1), WheelCount_Left, WheelCount_Right);
	if (positions_index == POSITIONS_COUNT) {
		return 1;
	}
	if (positions[positions_index - 1].X != Map_GetXPos() || positions[positions_index - 1].Y != Map_GetYPos() || positions_index == 0 ) {
		positions[positions_index].X = Map_GetXPos();
		positions[positions_index].Y = Map_GetYPos();

#ifdef ZONE_WALLFOLLOW
		if ( CM_IsSingleRoomMode() == 0 ) {
		//Wall follow robot heading
		if ( alignmentPtr > 4 ) {
			for ( i = alignmentPtr - 1; i >= alignmentPtr - 5; --i ) {
				sinTmp += sin((double)angleAlignment[i] * PI / 1800);
				cosTmp += cos((double)angleAlignment[i] * PI / 1800);
			}
			angles[positions_index] = (uint16_t)( atan2(sinTmp, cosTmp) * 1800 / PI + 3600 ) % 3600;
		} else {
			angles[positions_index] = Gyro_GetAngle(0);
		}
		}
#endif
		positions_index++;
		if (positions_index == POSITIONS_COUNT || (started == 1 && WFM_check_positions() == 1)) {
			return 1;
		}
	}
	return 0;
}

uint8_t WFM_boundary_check(void)
{
	uint8_t boundary_reach = 0;
	int16_t	j;
	int32_t	x, y;
	for (j = -1; boundary_reach == 0 && j <= 1; j++) {
		#ifdef RIGHT_WALL//not edit
		x = Map_GetRelativeX(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_2);
		y = Map_GetRelativeY(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_2);
		#endif
		if (Map_GetCell(MAP, countToCell(x), countToCell(y)) != BLOCKED_BOUNDARY) 
		{
			#ifdef RIGHT_WALL//not edit
			x = Map_GetRelativeX(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_3);
			y = Map_GetRelativeY(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_3);
			#endif
		}
		if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY) {
			boundary_reach = 1;
			Set_Wheel_Speed(0, 0);
			delay(10);
			Turn_Left(Turn_Speed, 600);
			WFM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1), WheelCount_Left, WheelCount_Right);
			Move_Forward(0, 0);
			return 1;
		}
	}
	return 0;
}

#ifdef ZONE_WALLFOLLOW
/*-----------------------------------Alignment-------------------------------------*/
/**
 * Inital alignemnt data
 */
void AlignmentInitial() {
	int16_t i = 0;

	for ( i = 0; i < ALIGNMENT_COUNT; ++i ) {
		positionAlignment[i].X = 0;
		positionAlignment[i].Y = 0;
		angleAlignment[i] = 0;
	}
	alignmentPtr = 0;

	for ( i = 0; i < LINES_SIZE; ++i ) {
		lineABCFirst[i].A = 0.0;
		lineABCFirst[i].B = 0.0;
		lineABCFirst[i].C = 0.0;
	}
	lineABCFirstPtr = 0;
	angleOffset = 0;
}

/**
 * Collect alignment data
 */
void Map_AlignmentDataCollect(void)
{
	if ( ( alignmentPtr < ALIGNMENT_COUNT &&
	       TwoPointsDistance( positionAlignment[alignmentPtr - 1].X, positionAlignment[alignmentPtr - 1].Y,
														Map_GetXCount(), Map_GetYCount()) >= ALIGNMENT_DATA_STEP  ) ||
	     alignmentPtr == 0 ) {
		positionAlignment[alignmentPtr].X = Map_GetXCount();
		positionAlignment[alignmentPtr].Y = Map_GetYCount();
		angleAlignment[alignmentPtr] = Gyro_GetAngle(0);
		alignmentPtr++;
	}
}

/**
 * Calculate the angle offset
 * @param  lineabc lines
 * @return         1: find optimal angle offset
 *            others: not find
 */
uint8_t calculateAngleOffset( LineABC *lineabc ) {
	uint8_t lineABCPtr, i, angleOffsetPtr = 0, retval = 0;
	int16_t angleTmp;
	double sinTmp, cosTmp;

	//Check which data
	if ( lineabc == lineABCFirst ) {
		lineABCPtr = lineABCFirstPtr;
	} 

	//Find which line is near 0, 90, 180 and 360 degree and save them
	for ( i = 0; i < lineABCPtr; ++i ) {
		angleTmp = (int16_t)(LineAngle(lineabc[i], 1) * 1800 / PI);
		USPRINTF("%s %d: Line %d Angle: %d\n", __FUNCTION__, __LINE__, i, angleTmp);

		if ( abs(angleTmp) < MAX_WALL_DELTA_ANGLE ) {
			angleOffsetTmp[angleOffsetPtr] = -angleTmp;
			angleOffsetPtr++;
		} else if ( abs(angleTmp) > 900 - MAX_WALL_DELTA_ANGLE ) {
			if ( angleTmp < 0 ) {
				angleTmp += 1800;
			}
			angleOffsetTmp[angleOffsetPtr] = 900 - angleTmp;
			angleOffsetPtr++;
		}
	}

	//Calculate mean
	angleTmp = 0; sinTmp = 0.0; cosTmp = 0.0;
	for ( i = 0; i < angleOffsetPtr; ++i ) {
		sinTmp += sin( (double)angleOffsetTmp[i] * PI / 1800 );
		cosTmp += cos( (double)angleOffsetTmp[i] * PI / 1800 );
		USPRINTF("%s %d: Line Angle %d Offset: %d\n", __FUNCTION__, __LINE__, i, angleOffsetTmp[i]);
	}

	//Set offset
	angleOffset = -(int16_t)( atan2(sinTmp, cosTmp) * 1800 / PI );
	USPRINTF("%s %d: Angle Offset: %d\n", __FUNCTION__, __LINE__, angleOffset);

	if ( angleOffsetPtr > 0 ) {
		retval = 1;
	}
	return retval;
}

/**
 * Lines segment
 * @param  lineABC lines
 * @return         1: success segment lines
 *            others: fail
 */
uint8_t lineSegment( LineABC *lineABC ) {
	uint8_t retval = 0, isFit = 0;
	int16_t i = 0, j = 0, int16Tmp, delta = 0;
	uint16_t k = 0, point32Size = 0;
	Point32_t cdnTmp;
	LineABC lineabcTmp;
	double angleVec;
	uint8_t error=0;
	
	USPRINTF("%s %d Line Segment...\n", __FUNCTION__, __LINE__);
	for ( i = 0; i < alignmentPtr - MIN_SEGMENT_SIZE; ++i ) {
		if ( lineABC == lineABCFirst && lineABCFirstPtr >= LINES_SIZE ) {
			break;
		} 

		//Find line segment
		for ( j = i + 1; j < alignmentPtr; ++j ) {
		//USPRINTF("%s %d j: %d\n", __FUNCTION__, __LINE__, j);
			delta = degreeDeltaAngleVector( angleAlignment[j], angleAlignment[i] );
			if ( delta >= MAX_SEGMENT_DELTA_ANGLE || delta < -MAX_SEGMENT_DELTA_ANGLE || j == alignmentPtr - 1 ) {
				if ( j - i > MIN_SEGMENT_SIZE ) {
					USPRINTF("%s %d Segment Idx: %d ~ %d\n", __FUNCTION__, __LINE__, i, j - 1);

					//Copy array
					point32Size = j - i;
					if(point32Size>ALIGNMENT_COUNT/2)
					{
						error = 1;
						break;
					}
					if((i+point32Size)>ALIGNMENT_COUNT)
					{
						error = 1;
						break;
					}
					for ( k = 0; k < point32Size; ++k ) {
						point32AryTmp[k] = positionAlignment[i + k];
						u16AryTmp[k] = angleAlignment[i + k];
					}
					//Fit the line segment
					isFit = findLine( point32AryTmp, u16AryTmp, point32Size, 1, &lineabcTmp, &cdnTmp, &int16Tmp, &angleVec);

					//Find line
					if ( isFit == 1 ) {
						if ( lineABC == lineABCFirst ) {
							if ( lineABCFirstPtr < LINES_SIZE ) {
								lineABCFirst[lineABCFirstPtr] = lineabcTmp;
								lineABCFirstPtr++;
							} else {
						 		USPRINTF("First: Too Many lines!\n");
						 		break;
							}
						} 
					}
				}

				i = j - 1;
				break;
			}
		}
		if(error==1)
		{
			break;
		}
	}

	//Set return value
	if ( lineABC == lineABCFirst && lineABCFirstPtr == 0 ) {
		retval = 0;
	} 
	else if ( lineABC == lineABCFirst && lineABCFirstPtr > 0 ) {
		retval = 1;
	} 
	else {
		retval = 0;
	}
	return retval;
}

uint8_t WFM_IsWallFollowSamePointAndHeading(Point16_t crtCell, uint16_t crtHeading) {
	int16_t i;
	for ( i = 1; i < positions_index - 10; ++i ) {
		if ( IsSamePointAndAngle( Map_CellToPoint(crtCell), crtHeading,
		                          Map_CellToPoint(positions[i]), angles[i], CELL_COUNT_MUL * 3, 200 ) == 1 ) {
			USPRINTF("Current Pnt: (%d, %d) Heading: %d\n", crtCell.X, crtCell.Y, crtHeading);
			USPRINTF("No.%d Pnt: (%d, %d) Heading: %d\n", i, positions[i].X,  positions[i].Y, angles[i]);
			return 1;
		}
	}
	return 0;
}

/*---------------------------------- Edit By ZZ -----------------------------*/
uint8_t WFM_IsWallFollowSamePointAndHeading_ZZ(Point16_t crtCell, uint16_t crtHeading) {
	int16_t i;
	for ( i = 1; i < positions_index - 4; ++i ) {
		if ( IsSamePointAndAngle( Map_CellToPoint(crtCell), crtHeading,
		                          Map_CellToPoint(positions[i]), angles[i], CELL_COUNT_MUL * 3, 200 ) == 1 ) {
			USPRINTF("Current Pnt: (%d, %d) Heading: %d\n", crtCell.X, crtCell.Y, crtHeading);
			USPRINTF("No.%d Pnt: (%d, %d) Heading: %d\n", i, positions[i].X,  positions[i].Y, angles[i]);
			return 1;
		}
	}
	return 0;
}
/******************************************************************************/

/*------------------------------------------------------------------End Alignment--------------------------*/

//Set uppest, downest, rightest, leftest and angle
void Map_Wall_Follow_SetUDRLA( Point16_t pnt16, uint16_t angle ) {
	//Renew the upest, downest, rightest and leftest point
	if ( pnt16.X >= wallUppestCell.X ) {
		wallUppestCell = pnt16;
		angleUppestCell = angle;
	}
	if ( pnt16.X <= wallDownestCell.X ) {
		wallDownestCell = pnt16;
		angleDownestCell = angle;
	}
	if ( pnt16.Y <= wallRightestCell.Y ) {
		wallRightestCell = pnt16;
		angleRightestCell = angle;
	}
	if ( pnt16.Y >= wallLeftestCell.Y ) {
		wallLeftestCell = pnt16;
		angleLeftestCell = angle;
	}
}

//Get wall follow direction, 1: clock wise; 2. anti clock wise
uint8_t Map_Wall_Follow_GetWallFollowDirection(void) {
	uint8_t retval = 1, clkCnt = 0, antiClkCnt = 0;

	if ( angleUppestCell < 1800 ) {
		antiClkCnt++;
	} else {
		clkCnt++;
	}
	
	if ( angleDownestCell < 1800 ) {
		clkCnt++;
	} else {
		antiClkCnt++;
	}

	if ( angleRightestCell >= 900 && angleRightestCell < 2700 ) {
		clkCnt++;
	} else {
		antiClkCnt++;
	}

	if ( angleLeftestCell >= 900 && angleLeftestCell < 2700 ) {
		antiClkCnt++;
	} else {
		clkCnt++;
	}

	if ( clkCnt < antiClkCnt ) {
		retval = 2;
	}
	return retval;
}
#endif

/*---------------------------- Edit By ZZ --------------------*/
uint8_t Check_Wall_Follow_Boundary(uint8_t LeftOffset)
{
	int32_t x,y;
	uint16_t Direction = 0;
	int8_t i = 0;	
	for(i=LeftOffset;i>=-2;i--)
	{
		CM_count_normalize(Gyro_GetAngle(0),i*CELL_SIZE,CELL_SIZE_2,&x,&y);
		x = countToCell(x);
		y = countToCell(y);
		if(Map_GetCell(MAP,x,y) != BLOCKED_BOUNDARY)
		{
			CM_count_normalize(Gyro_GetAngle(0),i*CELL_SIZE,CELL_SIZE_3,&x,&y);
			x = countToCell(x);
			y = countToCell(y);
		}
		if(Map_GetCell(MAP,x,y) == BLOCKED_BOUNDARY)
		{
			if((x == zones[Zone_GetCurrentZoneIdx()].zone.X + ZONE_SIZE_HALF + BOUNDARY_INCREMENT) || (x > xRangeMax))			
				Direction = 900;//2700
			else if((x == zones[Zone_GetCurrentZoneIdx()].zone.X - ZONE_SIZE_HALF - BOUNDARY_INCREMENT) || (x < xRangeMin))			
				Direction = 2700;//900
			else if((y == zones[Zone_GetCurrentZoneIdx()].zone.Y + ZONE_SIZE_HALF + BOUNDARY_INCREMENT) || (y > yRangeMax))			
				Direction = 1800;//0
			else if((y == zones[Zone_GetCurrentZoneIdx()].zone.Y - ZONE_SIZE_HALF - BOUNDARY_INCREMENT) || (y < yRangeMin))		
				Direction = 0;//1800
			else
			{
				USPRINTF("%s %d:Program Run at here!\n",__FUNCTION__,__LINE__);
				return 0;
			}
			USPRINTF("%s %d:dircction at here!\n",__FUNCTION__,__LINE__);
			USPRINTF("%%%%%%robot touch boundary%%%%%%!\n");
			CM_HeadToCourse(ROTATE_TOP_SPEED_10,Direction);//这里可以换位U型转向
			return 1;
		}
	}
	return 0;
}

uint8_t Match_StartCellAndStartAngle()
{
	uint8_t i;
	for(i=0;i<=6;i++)
	{
		if((TwoPointsDistance( StartCell_WallFollow[i].X, StartCell_WallFollow[i].Y, Map_GetXPos(), Map_GetYPos()) < 3) && 
				(abs(StartAngle_WallFollow[i] - Gyro_GetAngle(0)) <= 200))
		{
			USPRINTF("StartCell_WallFollow:(%d,%d),StartAngle_WallFollow:%d\n",StartCell_WallFollow[i].X,StartCell_WallFollow[i].Y,StartAngle_WallFollow[i]);
			return 1;
		}
	}
	return 0;
}
/**************************************************************/

/*------------------------------------------------------------------ Wall Follow --------------------------*/
uint8_t Map_Wall_Follow(MapWallFollowType follow_type)
{
	//Movement
	volatile uint8_t Motor_Check_Code = 0,special_flag=0;
	static volatile int32_t Wall_Distance = Wall_High_Limit;
	volatile int32_t Wall_Straight_Distance = 2*DISTANCE_1CM, Left_Wall_Speed = 0, Right_Wall_Speed = 0;
	int32_t Proportion = 0, Delta = 0, Previous = 0, R = 0;

	uint8_t Temp_Counter = 0, Jam = 0; 
	#ifdef MOBILITY
	uint8_t Mobility_Temp_Error = 0;
	uint32_t Temp_Mobility_Distance = 0;
	#endif
	/*-------------------------- Edit By ZZ --------------------*/
	Point16_t LastPosition=Map_GetCurrentCell();
	uint8_t BoundaryReachFlag = 0;
	uint8_t ZoneBoundaryReachFlag = 0;
	Point16_t ZoneCell;
	uint16_t ZoneCellAngle;
	/***********************************************************/
	
	#ifdef ZONE_WALLFOLLOW
	uint8_t bumperCount = 0;
	#endif

	int16_t Left_Wall_Buffer[3] = {0};
	uint32_t Temp_Rcon_Status = 0;

	Point16_t tmpCell2 = {0, 0};
	int32_t rightWheelOffset = 0, leftWheelOffset = 0;
		
	Point16_t startCell = {0, 0}, tmpCell = {0, 0},
	          tmpCell3 = {0, 0}, crtCell = {0, 0};

	uint16_t wallFollowCount = 0, wallFollowdDist = 0;

		//Calculate heading based on alignment
	uint16_t revEntranceHeading = 0, revWallFollowHeading = 0;
	double sinTmp = 0.0, cosTmp = 0.0;
	int16_t tmp;
	int32_t	x, y;
#ifdef ALIGNMENT_ANGLE_WALL_ENABLE
	uint8_t flag;
#endif

//	static MapWallFollowType follow_type_buffer;
	MapEscapeTrappedType escape_state;
	uint16_t i = 0;
	Point16_t ChargeRcon_Cell = {0, 0};
	uint8_t chargercon_enable_r=0;
	
#ifdef VIRTUAL_WALL
	Point16_t Virtual_Cell = {0, 0};
	uint8_t Pass_Virtual_Wall=0;
	uint8_t enable_br_bl_rcon=1;
#endif
	
	uint8_t same_positionAlignment_counter=0;
	uint8_t same_positionAlignment=7;

	Point16_t last_Cell = {0, 0};
	Point16_t same_position_Cell = {0, 0};
	
	uint8_t pass_station = 0;
	uint16_t same_position_value=7,same_angle_value=200;
	
	Wall_Distance=Wall_High_Limit;

	if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;
	if(Wall_Distance<Wall_Low_Limit)Wall_Distance=Wall_Low_Limit;

#ifdef ZONE_WALLFOLLOW
	isIsolatedWallFollow = 0;
	isNeedToMarkBoundary = 1;
#endif

	Reset_Bumper_Error();
	Work_Motor_Configure();
	g_follow_type = follow_type;
	
	Move_Forward(RUN_SPEED_15, RUN_SPEED_15);
	Reset_Wall_Step();
	Reset_Touch();
	Reset_Rcon_Remote();
	Reset_Average_Counter();
	Set_Vac_Speed();
	
	CorMove_Rcon = 0;
	/*------------------------------------------------------Wall Follow ---------------------------------------------------------------------------------*/
	Reset_Rcon_Status();
	#ifdef MOBILITY
	Temp_Mobility_Distance = Get_Move_Distance();
	#endif
	Reset_Wheel_Step();
	Set_Mobility_Step(1000);
	Reset_Wall_Step();
	Reset_WallAccelerate();
	Wall_Straight_Distance =5*DISTANCE_1CM;

	positions[0].X = positions[0].Y = 0;
	positions_index = 1;
	positions_start_index = 0;
	started = 0;

	//Move forward to hit
	if ( follow_type != Map_Wall_Follow_Escape_Trapped && isDirectWallFollow == 0 ) {
		Left_Wall_Speed=RUN_SPEED_5;
		Reset_OBST_Value();
#ifdef ZONE_WALLFOLLOW
		//No need to mark boundary when move forward
		isNeedToMarkBoundary = 0;
#endif

		while (1) {
			
			if(OBS_SLOW||Is_OBS_Near())
			{
				Left_Wall_Speed-=2;
			}
			else
			{
				i++;
				if(i>5)
				{
					i=0;
					if(Left_Wall_Speed<RUN_SPEED_13)Left_Wall_Speed++;
				}
			}
			if(Left_Wall_Speed<RUN_SPEED_5)Left_Wall_Speed=RUN_SPEED_5;			
			Move_Forward(Left_Wall_Speed,Left_Wall_Speed);
			
		#ifdef MOBILITY
		/*-------------------------------------Mobility----------------------------------------------*/
    if(Get_LeftWheel_Step()<500)
    {
      Temp_Mobility_Distance = Get_Move_Distance();
    }
    else
    {
      if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
      {
        Temp_Mobility_Distance = Get_Move_Distance();
        Check_Mobility();
        Reset_Mobility_Step();
      }
    }
		#endif
			
			#ifdef WALL_DYNAMIC
			Wall_Dynamic_Base(30);
			#endif
			
			#ifdef OBS_DYNAMIC
			OBS_Dynamic_Base(300);
			#endif

			WFM_update();

			if((follow_type == Map_Wall_Follow_Escape_Trapped_ZZ) || (follow_type == Map_Wall_Follow_To_Zone_Exit))
			{
				if(Check_Wall_Follow_Boundary(2))
				{
					ZoneBoundaryReachFlag = 1;
					break;
				}
			}			
			else if(WFM_boundary_check())
			{
				BoundaryReachFlag = 1;
				Reset_WallAccelerate();
				Wall_Straight_Distance =5*DISTANCE_1CM;
				break;
			}

			Temp_Rcon_Status = Get_Rcon_Status();

			if ( Temp_Rcon_Status & (RconFR_HomeT | RconFL_HomeT | RconL_HomeT | RconR_HomeT) ) 
			//if ( Temp_Rcon_Status & (RconL_HomeL | RconL_HomeR | RconR_HomeL | RconR_HomeR) )
			{
				CM_SetStationHome();
				USPRINTF("%s %d: home detected!\n", __FUNCTION__, __LINE__);
			}

			if (Temp_Rcon_Status & (Rcon_Signal_All_T)) {
				Reset_Rcon_Status();
				break;
			}

#ifdef VIRTUAL_WALL
			/*
			 * When checking virtual wall signals, ignore the RconBL_Wall signal.
			 */
			if (Temp_Rcon_Status & (RconL_Wall | RconR_Wall | RconBR_Wall | RconFR_Wall | RconFL_Wall|RconL_Wall_T | RconR_Wall_T  | RconFR_Wall_T | RconFL_Wall_T)) {

				OBS_OFF();
				USPRINTF("%s %d: virtual wall detected! BumperCount = COMPLICATED_AREA_BUMPER_MAX_COUNT + 1! %d\n", __FUNCTION__, __LINE__, Temp_Rcon_Status);

#ifdef ZONE_WALLFOLLOW
				bumperCount = COMPLICATED_AREA_BUMPER_MAX_COUNT + 1;
				#ifdef RIGHT_WALL
				x = Map_GetRelativeX(Gyro_GetAngle(0), -CELL_SIZE_2, 0);
				y = Map_GetRelativeY(Gyro_GetAngle(0), -CELL_SIZE_2, 0);
				#endif
				Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
#endif
			#ifdef RIGHT_WALL
			if (Temp_Rcon_Status & (RconFR_Wall | RconFL_Wall|RconFR_Wall_T | RconFL_Wall_T)) {
				Turn_Left(Turn_Speed, 450);
			} else if (Temp_Rcon_Status & (RconR_Wall|RconR_Wall_T)) {
				Turn_Left(Turn_Speed, 300);
			} else if (Temp_Rcon_Status & (RconBR_Wall)) {
				Turn_Left(Turn_Speed, 300);
			} else if (Temp_Rcon_Status & (RconL_Wall|RconL_Wall_T)) {
				Turn_Left(Turn_Speed, 600);
			}
			#endif
				Reset_VirtualWall();
				USPRINTF("%s %d: Check: virtual wall! Break!\n", __FUNCTION__, __LINE__);
				break;
			}
#endif

			if (Get_Cliff_Trig()) {
				OBS_ON();
				if (Get_Cliff_Trig()) {
					Set_Wheel_Speed(0,0);
					Set_Dir_Backward();
					delay(300);					
					Stop_Brifly();
					USPRINTF("%s %d: Check: Get Cliff! Break!\n", __FUNCTION__, __LINE__);
					break;
				}
			}

			/*check motor current.*/
			Motor_Check_Code = Check_Motor_Current();
			if (Motor_Check_Code) {
				if (Self_Check(Motor_Check_Code)) {
					CM_TouringCancel();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					USPRINTF("%s %d: Check: motor current fail! return 0\n", __FUNCTION__, __LINE__);
					return 0;
				}
				else{
					Left_Wall_Speed=RUN_SPEED_8;
				}
				Initialize_Motor();
			}

#ifdef OBS_DYNAMIC
			if (Get_Bumper_Status() || Is_FrontOBS_Trig()) {
#else
			if (Get_Bumper_Status()||(Get_FrontOBS() > Get_FrontOBST_Value())) {
#endif
				positions_start_index = positions_index;
				USPRINTF("%s %d: Check: Get_Bumper_Status! Break!\n", __FUNCTION__, __LINE__);
				break;
			}

			if (Touch_Detect()) {
				CM_TouringCancel();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				USPRINTF("%s %d: Check: Touch detect! return 0!\n", __FUNCTION__, __LINE__);
				return 0;
			}
			#ifdef BLDC_INSTALL
			if (Remote_Key(Remote_Max)) {
				if (CM_IsLowBattery() == 0) {
					Switch_VacMode();
				}
			}
			#endif

			if (Remote_Key(Remote_Home)){
				Set_BLDC_Speed(Vac_Speed_NormalL);
				Deceleration();
				Stop_Brifly();
				CM_SetGoHome(1);

				//Mark robot cell to extend the map for shortest path searching
				Map_SetCell(MAP, Map_GetXCount(), Map_GetYCount(), CLEANED);
				USPRINTF("%s %d: Check: Remote_Home! return 0!\n", __FUNCTION__, __LINE__);
				return 0;
			}
			delay(10);
		}
	}//end of hit to wall
	
	#ifdef ZONE_WALLFOLLOW
	if ( CM_IsSingleRoomMode() == 0 ) {
	//Need to mark boundary
	isNeedToMarkBoundary = 1;
	}
	#endif

	if ( follow_type == Map_Wall_Follow_Left_Zone ) 
	{
#ifdef ZONE_WALLFOLLOW
		if ( CM_IsSingleRoomMode() == 0 ) {
		isWallFollowTooFar = 0;
		returnCellType = 0;

		//Set wall follow zone entrance
		//Each wall follow distance count start
		startCell.X = tmpCell.X = tmpCell2.X = tmpCell3.X = wallFollowCell.X = Map_GetXPos();
		startCell.Y = tmpCell.Y = tmpCell2.Y = tmpCell3.Y = wallFollowCell.Y = Map_GetYPos();

		//Reset zone UDRL cell
		Zone_ResetZoneUDRL(startCell);
		angleUppestCell = angleDownestCell = angleLeftestCell = angleRightestCell = Gyro_GetAngle(0);

		//Initial new alignment
		AlignmentInitial();

		//Initial new alignment data colletion
		alignmentPtr = 0;
		for (i = 0; i < ALIGNMENT_COUNT; i++) {
			positionAlignment[i].X = positionAlignment[i].Y = 0;
			angleAlignment[i] = 0;
		}

		//Initial wall follow distance count
		zoneWallFollowDistanceCount = 0;
		}
#endif
	} 
	else if ( follow_type == Map_Wall_Follow_Left_Target ) 
	{
#ifdef ZONE_WALLFOLLOW
		if ( CM_IsSingleRoomMode() == 0 ) {
		//Each wall follow distance count start
		startCell.X = tmpCell.X = tmpCell2.X = tmpCell3.X = countToCell(Map_GetXCount());
		startCell.Y = tmpCell.Y = tmpCell2.Y = tmpCell3.Y = countToCell(Map_GetYCount());

		//Reset wall follow UDRL cell
		wallUppestCell.X = wallDownestCell.X = wallRightestCell.X = wallLeftestCell.X = Map_GetXPos();
		wallUppestCell.Y = wallDownestCell.Y = wallRightestCell.Y = wallLeftestCell.Y = Map_GetYPos();

		wallFollowCount = zoneWallFollowDistanceCount;

		//Initial new alignment data colletion
		alignmentPtr = 0;
		for (i = 0; i < ALIGNMENT_COUNT; i++) {
			positionAlignment[i].X = positionAlignment[i].Y = 0;
			angleAlignment[i] = 0;
		}
	} 
#endif
	}
	else if ( (follow_type == Map_Wall_Follow_Escape_Trapped_ZZ) || (follow_type == Map_Wall_Follow_To_Zone_Exit) || (follow_type == Map_Wall_Follow_To_Zone_Entrance)) 
	{
		/*----------------------- Edit By ZZ ------------------*/
		AlignmentInitial();
		OutTrapStartCell = wallFollowCell = Map_GetCurrentCell();
		SamePositionTime = 0;
		USPRINTF("%s %d: startCell:(%d,%d)\n",__FUNCTION__,__LINE__,OutTrapStartCell.X,OutTrapStartCell.Y);
		for (i = 0; i < POSITIONS_COUNT; ++i ) 
		{
			positions[i].X = 0;
			positions[i].Y = 0;
			angles[i] = 0;
		}
		
		wallUppestCell.X = 0; wallUppestCell.Y = 0;
		wallDownestCell.X = 0; wallDownestCell.Y = 0;
		wallRightestCell.X = 0; wallRightestCell.Y = 0;
		wallLeftestCell.X = 0; wallLeftestCell.Y = 0;

		angleUppestCell = 0;
		angleDownestCell = 0;
		angleRightestCell = 0;
		angleLeftestCell = 0;
		
		isNeedToMarkBoundary = 1;
		OutTrapWallFollowDistanceCount = 0;
		WallFollowARoundFlag = 0;
		/*********************************************************/
	}
	else if ( follow_type == Map_Wall_Follow_Escape_Trapped ) 
	{
		CM_HeadToCourse(ROTATE_TOP_SPEED_10, Gyro_GetAngle(0) - 900);
		USPRINTF("%s %d Reference Cell: (%d, %d)\n", __FUNCTION__, __LINE__,
		         path_escape_get_trapped_cell()[0].X, path_escape_get_trapped_cell()[0].Y);
	}

	isDirectWallFollow = 0;

	/* Set escape trapped timer when it is in Map_Wall_Follow_Escape_Trapped mode. */
	escape_trapped_timer = (follow_type == Map_Wall_Follow_Escape_Trapped) ? Work_Timer : 0;
	Reset_Rcon_Status();

	if(Zone_GetZoneSize()>=8)
	{
		same_positionAlignment = 2;		
	}
	else if(Zone_GetZoneSize()>=7)
	{
		same_positionAlignment = 3;		
	}
	else if(Zone_GetZoneSize()>=5)
	{
		same_positionAlignment = 4;		
	}	
	else if(Zone_GetZoneSize()>=4)
	{
		same_positionAlignment = 5;		
	}	
	else if(Zone_GetZoneSize()>=3)
	{
		same_positionAlignment = 6;		
	}	
	else
	{
		same_positionAlignment = 7;
	}	
	
	
	#ifdef VIRTUAL_WALL
	WallFollowMulti_Pass_VirtualWall=0;
	End_WallFollowMulti_VirtualWall_Point.X = 0;
	End_WallFollowMulti_VirtualWall_Point.Y = 0;
	Start_WallFollowMulti_VirtualWall_Point =End_WallFollowMulti_VirtualWall_Point;
	#endif	
	while (1) {
		
		#ifdef MOBILITY
		/*-------------------------------------Mobility----------------------------------------------*/
    if(Get_LeftWheel_Step()<500)
    {
      Temp_Mobility_Distance = Get_Move_Distance();
    }
    else
    {
      if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
      {
        Temp_Mobility_Distance = Get_Move_Distance();
        Check_Mobility();
        Reset_Mobility_Step();
      }
    }
		#endif
		
		WFM_update();

#ifdef ZONE_WALLFOLLOW
		crtCell.X = Map_GetXPos();
		crtCell.Y = Map_GetYPos();
		if((Work_Timer % 2 == 0) && (SendFlag_wallfollow))
		{
			SendFlag_wallfollow = 0;
			USPRINTF("WallFollow Record Time:%d Current Cell:(%d,%d)\n",Work_Timer,crtCell.X,crtCell.Y);
		}
		else if(Work_Timer % 2 != 0)
			SendFlag_wallfollow = 1;
		
		//Alignment Data Collect
		if((follow_type != Map_Wall_Follow_Escape_Trapped_ZZ) && (follow_type != Map_Wall_Follow_To_Zone_Exit) && (follow_type != Map_Wall_Follow_To_Zone_Entrance))
			Map_AlignmentDataCollect();

		//Wall follow entrance robot heading
		sinTmp = 0.0, cosTmp = 0.0;
		/*------------------- Edit By ZZ --------------------*/
		if ( alignmentPtr == 2 ) {
			startCell = Map_GetCurrentCell();
			revEntranceHeading = Gyro_GetAngle(0);
			USPRINTF("StartCell: (%d,%d),Entrance cell heading: %d\n", startCell.X,startCell.Y,revEntranceHeading);
		}
		if((alignmentPtr <= 6) && (alignmentPtr >= 1))
		{
			StartCell_WallFollow[alignmentPtr-1] = Map_GetCurrentCell();
			StartAngle_WallFollow[alignmentPtr-1] = Gyro_GetAngle(0);
		}
		/****************************************************/

		//Wall follow robot heading
		if ( alignmentPtr > 4 ) {
			for ( i = alignmentPtr - 1; i >= alignmentPtr - 4; --i ) {
				sinTmp += sin((double)angleAlignment[i] * PI / 1800);
				cosTmp += cos((double)angleAlignment[i] * PI / 1800);
			}
			revWallFollowHeading = (uint16_t)( atan2(sinTmp, cosTmp) * 1800 / PI + 3600 ) % 3600;
		} else {
			revWallFollowHeading = Gyro_GetAngle(0);
		}

		//Set uppest, downest, rightest, leftest and angle
		Map_Wall_Follow_SetUDRLA(crtCell, revWallFollowHeading);
#endif

		if (follow_type == Map_Wall_Follow_Escape_Trapped_ZZ)
		{
			wallFollowdDist = (uint16_t)(TwoPointsDistance( wallFollowCell.X, wallFollowCell.Y, crtCell.X, crtCell.Y ));
			if ( wallFollowdDist > ROBOT_SIZE - 2 )
			{
				OutTrapWallFollowDistanceCount++;
				wallFollowCell = crtCell;
			}
			
			if((LastPosition.X != Map_GetXPos()) || (LastPosition.Y != Map_GetYPos()))
			{
				SamePositionTime = 0;
				LastPosition = Map_GetCurrentCell();
			}
			
			if (((Work_Timer - escape_trapped_timer_ZZ) > ESCAPE_TRAPPED_TIME_ZZ) || (SamePositionTime > 30))	//在同一个点超过5秒
			{
				USPRINTF("%s %d: escape timeout %d(%d, %d)\n", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME_ZZ, Work_Timer, escape_trapped_timer_ZZ);
				USPRINTF("%s %d: Check: Escape! Return 2\n", __FUNCTION__, __LINE__);
				return 2;
			}
				
			escape_state = WFM_check_trapped_ZZ();

			if (escape_state == Map_Escape_Trapped_Escaped)
			{
				Stop_Brifly();
				USPRINTF("%s %d: Check: Escape! Return 0\n", __FUNCTION__, __LINE__);
				return 0;
			} 
		}
		
		if ((follow_type == Map_Wall_Follow_To_Zone_Exit) || (follow_type == Map_Wall_Follow_To_Zone_Entrance))
		{
			if(follow_type == Map_Wall_Follow_To_Zone_Exit)
			{
				ZoneCellAngle = Zone_GetCurrentZoneExitRobotHeading();
				ZoneCell = Zone_GetCurrentZoneExit();
			}
			else
			{
				ZoneCellAngle = Zone_GetCurrentZoneEntranceRobotHeading();
				ZoneCell = Zone_GetCurrentZoneEntrance();
			}
			if((Work_Timer % 2 == 0) && (SendFlag_OutTrap))
			{
				SendFlag_OutTrap = 0;
				USPRINTF("CurrentCell:(%d,%d)  ZoneCell:(%d,%d)\n",Map_GetXPos(),Map_GetYPos(),ZoneCell.X,ZoneCell.Y);
			}
			else if(Work_Timer % 2 != 0)
				SendFlag_OutTrap = 1;
			wallFollowdDist = (uint16_t)(TwoPointsDistance( wallFollowCell.X, wallFollowCell.Y, crtCell.X, crtCell.Y ));
			if ( wallFollowdDist > ROBOT_SIZE - 2 )
			{
				OutTrapWallFollowDistanceCount++;
				wallFollowCell = crtCell;
			}
			
			if((LastPosition.X != Map_GetXPos()) || (LastPosition.Y != Map_GetYPos()))
			{
				SamePositionTime = 0;
				LastPosition = Map_GetCurrentCell();
			}
			
			if (((Work_Timer - wall_follow_to_zone_cell_timer) > ESCAPE_TRAPPED_TIME_ZZ) || (SamePositionTime > 30))			//在同一个点超过5秒
			{
				USPRINTF("%s %d: escape timeout %d(%d, %d)\n", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME_ZZ, Work_Timer, wall_follow_to_zone_cell_timer);
				USPRINTF("%s %d: Check: Escape! Return 2\n", __FUNCTION__, __LINE__);
				return 2;
			}
			
			//转一圈还未到达分区终点
			if ( OutTrapWallFollowDistanceCount > 20 &&
			 Map_Wall_Follow_GetWallFollowDirection() == 2 &&
			 ((TwoPointsDistance( OutTrapStartCell.X, OutTrapStartCell.Y, Map_GetXPos(), Map_GetYPos() ) < 3) ||
				WFM_IsWallFollowSamePointAndHeading(Map_GetCurrentCell(), Gyro_GetAngle(0)) == 1)) 
			{
				STOP_BRIFLY;
				USPRINTF("%s %d Wall follow in mini room! End after cleaning\n", __FUNCTION__, __LINE__);	
				WallFollowARoundFlag = 1;
				return 0;
			}			
			//到达分区终点附近
			if((TwoPointsDistance( ZoneCell.X, ZoneCell.Y, Map_GetXPos(), Map_GetYPos()) < 5) &&
				(abs(ZoneCellAngle - Gyro_GetAngle(0)) <= 200))
			{
				STOP_BRIFLY;
				USPRINTF("%s %d: Arrive at zone's cell\n", __FUNCTION__, __LINE__);	
				return 0;
			}
		}	
				
		if (follow_type == Map_Wall_Follow_Escape_Trapped) 
		{
			if (escape_trapped_timer != 0 && (Work_Timer - escape_trapped_timer) > ESCAPE_TRAPPED_TIME) {
				USPRINTF("%s %d: escape timeout %d(%d, %d)\n", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME, Work_Timer, escape_trapped_timer);
				USPRINTF("%s %d: Check: Escape! Return 2\n", __FUNCTION__, __LINE__);
				return 2;
			}

			escape_state = WFM_check_trapped();

			if (escape_state == Map_Escape_Trapped_Escaped) {
				USPRINTF("%s %d: Check: Escape! Return 0\n", __FUNCTION__, __LINE__);
				return 0;
			} else if (escape_state == Map_Escape_Trapped_Timeout) {
				USPRINTF("%s %d: Check: Escape! Return 2\n", __FUNCTION__, __LINE__);
				return 2;
			}
		}

#ifdef ZONE_WALLFOLLOW
		if ( CM_IsSingleRoomMode() == 0 ) {

		if ( follow_type == Map_Wall_Follow_Left_Zone ) 
		{			
			//Stop wall follow method 6
			if ( Zone_GetZoneSize() > 2 && Zone_GetTrappedZoneSize() == 0 )
			{
			if(path_blocks_size>10)
			{
				if ( TwoPointsDistance( last_Cell.X,last_Cell.Y, crtCell.X, crtCell.Y ) > 0)
				{
					last_Cell.X = crtCell.X;
					last_Cell.Y = crtCell.Y;
					for(i=0;i<path_blocks_size;i++)
					{		
						if((i+10)>path_blocks_size)
						{							
							break;
						}						
						if ( ( TwoPointsDistance( path_blocks[i].X,path_blocks[i].Y, crtCell.X, crtCell.Y ) < same_position_value &&
								 abs( degreeDeltaAngleVector(path_blocks_angle[i], revWallFollowHeading) ) < same_angle_value )
						 ) {											
									if((TwoPointsDistance( same_position_Cell.X,same_position_Cell.Y, crtCell.X, crtCell.Y ) > 1))
									{
										same_positionAlignment_counter++;
										
										same_position_Cell.X = crtCell.X;
										same_position_Cell.Y = crtCell.Y;
										
										USPRINTF("Same positionAlignment same_positionAlignment_counter:%d;  i:%d;  path_blocks[%d].X:%d;  path_blocks[%d].y:%d;  crtCell.X:%d;  crtCell.y%d;  path_blocks_angle[%d]:%d; revWallFollowHeading:%d", same_positionAlignment_counter, i,i,path_blocks[i].X,i,path_blocks[i].Y,crtCell.X,crtCell.Y,i,path_blocks_angle[i],revWallFollowHeading);
										if(((same_positionAlignment_counter>(2))&&(Get_Station_Position()==1))||(same_positionAlignment_counter>same_positionAlignment))
										{
																								
											USPRINTF("%s %d Same positionAlignment as cleaning entrance/Exit to stop wall follow!\n", __FUNCTION__, __LINE__);

											Zone_ForceAddZone( startCell.X, startCell.Y, Map_GetXPos(), Map_GetYPos() );

											if((Zone_GetZoneSize() > 2))
											{											
												if(Get_Station_Position()==1)
												{
													if(pass_station==0)												
													{
														same_positionAlignment_counter = 0;														
													}
													else
													{
														Zone_SetZoneWallFollowStatus(1);
													}
												}
												else
												{
													Zone_SetZoneWallFollowStatus(1);
												}
												
											}
											#ifdef ALIGNMENT_ANGLE_WALL_ENABLE
											if ( lineSegment(lineABCFirst) == 1 ) {
												calculateAngleOffset(lineABCFirst);
												Gyro_SetOffset((int16_t)(angleOffset));
											}
											#endif
											return 0;
										}										
										break;
									}																	
								break;
							}
						}
					}
				}
				else
				{
					  //Stop wall follow method 4				
						if ( ( TwoPointsDistance( Zone_GetZoneEntrance(0).X, Zone_GetZoneEntrance(0).Y, crtCell.X, crtCell.Y ) < 7 &&
									 abs( degreeDeltaAngleVector(Zone_GetZoneEntranceRobotHeading(0), revWallFollowHeading) ) < 200 ) ||
								 ( TwoPointsDistance( Zone_GetZoneExit(0).X, Zone_GetZoneExit(0).Y, crtCell.X, crtCell.Y ) < 7 &&
									 abs( degreeDeltaAngleVector(Zone_GetZoneExitRobotHeading(0), revWallFollowHeading) ) < 200 &&
									 Zone_GetCurrentContinualZoneIdx() > 1 )
							 ) {								 
										USPRINTF("%s %d Same position as cleaning entrance/Exit to stop wall follow!\n", __FUNCTION__, __LINE__);
										Zone_ForceAddZone( startCell.X, startCell.Y, Map_GetXPos(), Map_GetYPos() );
										if((Zone_GetZoneSize() > 2))
										{
											Zone_SetZoneWallFollowStatus(1);
										}

										#ifdef ALIGNMENT_ANGLE_WALL_ENABLE
										if ( lineSegment(lineABCFirst) == 1 ) {
											calculateAngleOffset(lineABCFirst);
											Gyro_SetOffset((int16_t)(angleOffset));
										}
										#endif
										return 0;
							 }								 
				}
			}

			//Save wall follow data
			wallFollowdDist = (uint16_t)(TwoPointsDistance( wallFollowCell.X, wallFollowCell.Y, crtCell.X, crtCell.Y ));
			if ( wallFollowdDist > ROBOT_SIZE - 2 ) {
				zoneWallFollowDistanceCount++;
				wallFollowCell = crtCell;
			}

			//Wall follow too far
			if ( wallFollowdDist > ROBOT_SIZE - 2 ) {
				if ( zoneWallFollowDistanceCount > ZONE_SIZE * 4 / (ROBOT_SIZE - 1) ) {
					STOP_BRIFLY;
					USPRINTF("Wall Follow Too Far!\n");
					isWallFollowTooFar = 1;

					if ( isMiniRoom != 1 && Zone_GetZoneSize() == 0 )
						isNeedCheckMiniRoom = 1;
					else {
						isNeedCheckMiniRoom = 0;
					}

					//if ( Zone_GetZoneSize() != 0 )
					Zone_ForceAddZone( startCell.X, startCell.Y, Map_GetXPos(), Map_GetYPos() );
					USPRINTF("%s %d: Check: Wall follow too far! Break!\n", __FUNCTION__, __LINE__);
					return 0;
				}
			}

			//Mini room mode
				if ( zoneWallFollowDistanceCount > 15 &&
			     isNeedCheckMiniRoom == 1 &&
			     Map_Wall_Follow_GetWallFollowDirection() == 2 &&
			     (((TwoPointsDistance( startCell.X, startCell.Y, crtCell.X, crtCell.Y ) < 3) && 
						(abs(revEntranceHeading - Gyro_GetAngle(0)) <= 400)) ||
						(Match_StartCellAndStartAngle() == 1)/* ||
						WFM_IsWallFollowSamePointAndHeading(crtCell, revWallFollowHeading) == 1*/)
			   ) {

				STOP_BRIFLY;
				USPRINTF("%s %d Wall follow in mini room! End after cleaning\n", __FUNCTION__, __LINE__);
				isMiniRoom = 1;
				Zone_ForceAddZone( startCell.X, startCell.Y, Map_GetXPos(), Map_GetYPos() );
				isNeedCheckMiniRoom = 0;

				USPRINTF("Set Entrance Robot Heading!\n");
				Zone_SetZoneEntranceRobotHeading(revEntranceHeading, Zone_GetCurrentZoneIdx());

				if ( lineSegment(lineABCFirst) == 1 ) {
					//Find wall
					if ( wallFoundStatus == Map_Find_Wall_Not_Found ) {
						wallFoundStatus = WFM_FindWallBasedOnAlignment(lineABCFirst);
					}
				}
				USPRINTF("%s %d: Check: Mini room! Break!\n", __FUNCTION__, __LINE__);
				return 0;
			}

			//Check wall follow
			if ( TwoPointsDistance( startCell.X, startCell.Y, crtCell.X, crtCell.Y ) < 3 &&
			     zoneWallFollowDistanceCount > 10 &&
			     Map_Wall_Follow_GetWallFollowDirection() == 1 ) {
				STOP_BRIFLY;
				USPRINTF("In isolated island! Wall follow too far set!\n");
				isWallFollowTooFar = 1;
				if ( Zone_GetZoneSize() != 0 ) {
					Zone_ForceAddZone( startCell.X, startCell.Y, Map_GetXPos(), Map_GetYPos() );
					if ( isMiniRoom != 1 )
						isNeedCheckMiniRoom = 0;

					#ifdef ALIGNMENT_ANGLE_WALL_ENABLE
					if ( lineSegment(lineABCFirst) == 1 ) {
						calculateAngleOffset(lineABCFirst);
						Gyro_SetOffset((int16_t)(angleOffset));
					}
					#endif
				}

				//Set zone entrance robot heading
				USPRINTF("Set Entrance Robot Heading!\n");
				Zone_SetZoneEntranceRobotHeading(revEntranceHeading, Zone_GetCurrentZoneIdx());
				USPRINTF("%s %d: Check: Isolated area! Break!\n", __FUNCTION__, __LINE__);
				return 0;
			}

			tmp = Zone_IsAddNewZone( startCell.X, startCell.Y, Map_GetXPos(), Map_GetYPos() );
			if ( tmp == 1 ) {
				//No need to check mini room
				isNeedCheckMiniRoom = 0;
				Deceleration();
				STOP_BRIFLY;
				WFM_update();
				//Set zone entrance robot heading
				Zone_SetZoneEntranceRobotHeading(revEntranceHeading, Zone_GetCurrentZoneIdx());

				//Save the count of wall follow cells
				//zoneWallFollowDistanceCount = wallFollowCell_cnt;
				USPRINTF("Current Zone Exit: x: %d\ty: %d\n", countToCell(Map_GetXCount()), countToCell(Map_GetYCount()) );

				//Line Segment
				#ifdef ALIGNMENT_ENABLE
				if ( lineSegment(lineABCFirst) == 1 ) {
					Speaker(23);
					returnCellType = 1;
					USPRINTF("%s %d Set return type - Entrance!\n", __FUNCTION__, __LINE__);
				#ifdef ALIGNMENT_ANGLE_WALL_ENABLE

					//Find wall
					if ( wallFoundStatus == Map_Find_Wall_Not_Found ) {
						wallFoundStatus = WFM_FindWallBasedOnAlignment(lineABCFirst);
					}
					if ( wallFoundStatus == Map_Find_Wall_Found ) {
						returnCellType = 0;
						USPRINTF("%s %d Set return type - Exit!\n", __FUNCTION__, __LINE__);
					}
					calculateAngleOffset(lineABCFirst);
					Gyro_SetOffset((int16_t)(angleOffset));
				#endif
				} else {
					returnCellType = 0;
					USPRINTF("%s %d Set return type - Exit!\n", __FUNCTION__, __LINE__);
				}
				#endif
				if ( bumperCount > COMPLICATED_AREA_BUMPER_MAX_COUNT ) {
					returnCellType = 0;
					USPRINTF("%s %d Set return type - Exit!\n", __FUNCTION__, __LINE__);
				}
				USPRINTF("%s %d: Check: Wall follow left zone end! Return!\n", __FUNCTION__, __LINE__);
				return 0;
			}
		}

		//If follow type is left wall follow to target
		if ( follow_type == Map_Wall_Follow_Left_Target ) {
			if ( TwoPointsDistance(tmpCell.X, tmpCell.Y, Map_GetXPos(), Map_GetYPos() ) > ROBOT_SIZE - 2 ) {
				tmpCell.X = Map_GetXPos();
				tmpCell.Y = Map_GetYPos();
				wallFollowCount--;
			}

		if ( 0 == wallFollowCount || TwoPointsDistance( Zone_GetCurrentZoneExit().X,
		                                                Zone_GetCurrentZoneExit().Y,
		                                                Map_GetXPos(), Map_GetYPos() ) < 3 ) 
			{
			  //3
			  //STOP_BRIFLY;
				for (tmp = 0; tmp < alignmentPtr; ++tmp) {
					USPRINTF("Alignment Points: %d x:%d\ty:%d\tangle:%d\n", tmp, positionAlignment[tmp].X, positionAlignment[tmp].Y, angleAlignment[tmp]);
				}
				//Alignment
				#ifdef ALIGNMENT_ENABLE
				angleOffset = 0;
				lineSegment(lineABCFirst);
				#ifdef ALIGNMENT_ANGLE_WALL_ENABLE
				flag = calculateAngleOffset(lineABCFirst);
				if ( flag == 1 )
					Gyro_SetOffset((int16_t)(angleOffset));
				angleOffset = 0;
				#endif
				#endif
				USPRINTF("%s %d: Check: Map_Wall_Follow_Left_Target! break\n", __FUNCTION__, __LINE__);
				return 0;
			}
		}

		if ( follow_type != Map_Wall_Follow_Left_Target &&
		     follow_type != Map_Wall_Follow_Left_Zone && 
				 follow_type != Map_Wall_Follow_Escape_Trapped_ZZ && 
				 follow_type != Map_Wall_Follow_To_Zone_Exit &&
				 follow_type != Map_Wall_Follow_To_Zone_Entrance) 
			{
				if (started == 0 && ((Get_RightWall_Step() > Map_Wall_Follow_Distance) || (Get_LeftWall_Step() > Map_Wall_Follow_Distance))) {
					USPRINTF("%s %d: %d %d %d\n", __FUNCTION__, __LINE__, Get_RightWall_Step(), Get_LeftWall_Step(), Map_Wall_Follow_Distance);
					started = 1;
				}
			}
		}
#endif

#ifdef OBS_DYNAMIC
		OBS_Dynamic_Base(100);
#endif

		#ifdef MOBILITY
		if (Get_LeftWheel_Step() < 500) {
			Mobility_Temp_Error = 0;
			Temp_Mobility_Distance = Get_Move_Distance();
		} else {
			if ((Get_Move_Distance() - Temp_Mobility_Distance) > 500) {
				Temp_Mobility_Distance = Get_Move_Distance();
				if (Get_Mobility_Step() < 1) {
					Mobility_Temp_Error++;
					if (Mobility_Temp_Error > 5) {
						Mob_Error_Add();
						if (Get_Mob_Error() < 4) {
							STOP_BRIFLY;
							WFM_update();
							WFM_move_back(5*DISTANCE_1CM);
							WFM_update();
							STOP_BRIFLY;
							Turn_Right(Turn_Speed, 1200);
							STOP_BRIFLY;
							WFM_update();
							Reset_Wheel_Step();
							Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
							ZoneBoundaryReachFlag = 0;
						}
						Mobility_Temp_Error = 0;
					}
				} else {
					Mobility_Temp_Error = 0;
				}
				Reset_Mobility_Step();
			}
		}
		#endif
		/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code = Check_Motor_Current();
		if (Motor_Check_Code) {
			if (Self_Check(Motor_Check_Code)) {
				CM_TouringCancel();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				USPRINTF("%s %d: Check: Clean Mode! break\n", __FUNCTION__, __LINE__);
				break;
			}
			Initialize_Motor();
		}
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if (Touch_Detect()) {
			CM_TouringCancel();
			Set_Clean_Mode(Clean_Mode_Userinterface);
			USPRINTF("%s %d: Check: Touch Clean Mode! break\n", __FUNCTION__, __LINE__);
			break;
		}
		if (Get_Rcon_Remote() != 0) {
			#ifdef BLDC_INSTALL
			if (Remote_Key(Remote_Max)) {
				Switch_VacMode();
			}
			#endif
			if (Remote_Key(Remote_Home)) {	
				Set_BLDC_Speed(Vac_Speed_NormalL);
				Display_Content(LED_Home, 100, 100, 0, 7);
				Set_Clean_Mode(Clean_Mode_GoHome);
				Reset_Rcon_Remote();
				SetHomeRemote();
				CM_SetGoHome(1);
				USPRINTF("%s %d: Check: Remote! Return 0\n", __FUNCTION__, __LINE__);
				return 0;
			}
			Reset_Rcon_Remote();
		}
		/*------------------------------------------------------Check Battery-----------------------*/
			if ((Check_Bat_Home() == 1)) {
			Set_BLDC_Speed(Vac_Speed_NormalL);
			Display_Content(LED_Home, 100, 100, 0, 7);
			Set_Clean_Mode(Clean_Mode_GoHome);
			Reset_Rcon_Remote();
			SetHomeRemote();
			CM_SetGoHome(1);
			USPRINTF("%s %d: Check: low power! Return 0\n", __FUNCTION__, __LINE__);
			return 0;
		}
				
		if (Check_Bat_SetMotors(Clean_Vac_Power, Clean_SideBrush_Power, Clean_MainBrush_Power)) {	
			Display_Battery_Status(Display_Low);	
			delay(3000);
			Set_Clean_Mode(Clean_Mode_Userinterface);
			CM_TouringCancel();
			USPRINTF("%s %d: Check: Battery! break\n", __FUNCTION__, __LINE__);
			break;
		}
		/*------------------------------------------------------Virtual Wall Event-----------------------*/
		Temp_Rcon_Status = Get_Rcon_Status();
		#ifdef VIRTUAL_WALL
		Temp_Rcon_Status = Temp_Rcon_Status &(Rcon_Signal_WALL_T|Rcon_Signal_All_T);
		#else
		Temp_Rcon_Status = Temp_Rcon_Status &(Rcon_Signal_All_T);
		#endif
		if (Temp_Rcon_Status) {			
			if (follow_type == Map_Wall_Follow_Left_Target || follow_type == Map_Wall_Follow_Left_Zone||(follow_type == Map_Wall_Follow_Escape_Trapped_ZZ) || (follow_type == Map_Wall_Follow_To_Zone_Exit) || (follow_type == Map_Wall_Follow_To_Zone_Entrance)) 
			{
				#ifdef ZONE_WALLFOLLOW
				if ( CM_IsSingleRoomMode() == 0 ) {
				if ( Temp_Rcon_Status & (RconFR_HomeT | RconFL_HomeT | RconL_HomeT | RconR_HomeT) ) 
				//if ( Temp_Rcon_Status & (RconL_HomeL | RconL_HomeR | RconR_HomeL | RconR_HomeR) )
				{
					CM_SetStationHome();
					pass_station = 1;					
					bumperCount = COMPLICATED_AREA_BUMPER_MAX_COUNT + 1;
					USPRINTF("%s %d: BumperCount = COMPLICATED_AREA_BUMPER_MAX_COUNT + 1!\n", __FUNCTION__, __LINE__);
					USPRINTF("%s %d: Home detected!\n", __FUNCTION__, __LINE__);
				}				
				if ( TwoPointsDistance(ChargeRcon_Cell.X, ChargeRcon_Cell.Y, Map_GetXPos(), Map_GetYPos()) > 0 )
				{					
					if (Temp_Rcon_Status & Rcon_Signal_All_T) 
					{
						#ifdef RIGHT_WALL
						x = Map_GetRelativeX(Gyro_GetAngle(0), -CELL_SIZE_2, 0);
						y = Map_GetRelativeY(Gyro_GetAngle(0), -CELL_SIZE_2, 0);
						#endif
						Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
						ZoneBoundaryReachFlag = 0;
						if (Temp_Rcon_Status & (RconFR_HomeT | RconFL_HomeT)) 
						{
							if (( TwoPointsDistance(ChargeRcon_Cell.X, ChargeRcon_Cell.Y, Map_GetXPos(), Map_GetYPos()) > 2 )||((from_station==1)&&(Zone_GetZoneSize()==0)))
							{
								#ifdef RIGHT_WALL
								Turn_Left(Turn_Speed, 600);
								#endif
								ChargeRcon_Cell.X = Map_GetXPos();
								ChargeRcon_Cell.Y = Map_GetYPos();
								chargercon_enable_r = 1;
							}
						}
						else if (Temp_Rcon_Status & RconL_HomeT) 
						{
							if (( TwoPointsDistance(ChargeRcon_Cell.X, ChargeRcon_Cell.Y, Map_GetXPos(), Map_GetYPos()) > 2 )||((from_station==1)&&(Zone_GetZoneSize()==0)))
							{
								#ifdef RIGHT_WALL
								Turn_Left(Turn_Speed, 800);
								#endif
								ChargeRcon_Cell.X = Map_GetXPos();
								ChargeRcon_Cell.Y = Map_GetYPos();
								chargercon_enable_r = 1;
							}
						} 
						else if (Temp_Rcon_Status & RconR_HomeT) 
						{
							if (( TwoPointsDistance(ChargeRcon_Cell.X, ChargeRcon_Cell.Y, Map_GetXPos(), Map_GetYPos()) > 2 )||((from_station==1)&&(Zone_GetZoneSize()==0)))
							{
								if(chargercon_enable_r==1)
								{
									#ifdef RIGHT_WALL
									Turn_Left(Turn_Speed, 450);
									#endif
								}
								ChargeRcon_Cell.X = Map_GetXPos();
							  ChargeRcon_Cell.Y = Map_GetYPos();
							}
						}
						Reset_Charge_Rcon();						
					}
				}
				}
				
				#ifdef VIRTUAL_WALL
				/*
				 * When checking virtual wall signals, ignore the RconBL_Wall signal.
				 */						
				if ( TwoPointsDistance(Virtual_Cell.X, Virtual_Cell.Y, Map_GetXPos(), Map_GetYPos()) > 0 )
				{
					if (Temp_Rcon_Status & (RconL_Wall_T | RconR_Wall_T  | RconFR_Wall_T | RconFL_Wall_T))
					{
						OBS_OFF();
						Reset_Top_VirtualWall();
						#ifdef RIGHT_WALL
						Turn_Left(Turn_Speed, 450);
						#endif
						Virtual_Cell.X = Map_GetXPos();
						Virtual_Cell.Y = Map_GetYPos();
						Pass_Virtual_Wall = 1;
						enable_br_bl_rcon = 2;
						ZoneBoundaryReachFlag = 0;
						#ifdef VIRTUAL_WALL
						WallFollowMulti_End_VirtualWall = 1;						
						if(WallFollowMulti_Pass_VirtualWall==0)							
						{
							WallFollowMulti_Pass_VirtualWall = 1;
							Start_WallFollowMulti_VirtualWall_Point.X = Map_GetXCount();
							Start_WallFollowMulti_VirtualWall_Point.Y = Map_GetYCount();							
							End_WallFollowMulti_VirtualWall_Point = Start_WallFollowMulti_VirtualWall_Point;
						}
						else
						{
							End_WallFollowMulti_VirtualWall_Point.X = Map_GetXCount();
							End_WallFollowMulti_VirtualWall_Point.Y = Map_GetYCount();
						}
						#endif
					}			
					if ( TwoPointsDistance(startCell.X, startCell.Y, Map_GetXPos(), Map_GetYPos()) < 3 )
					{
						if(enable_br_bl_rcon!=2)
						{
							enable_br_bl_rcon = 1;
						}
					}
					
					if (Temp_Rcon_Status & (RconL_Wall | RconR_Wall  | RconFR_Wall | RconFL_Wall)) {
						bumperCount = COMPLICATED_AREA_BUMPER_MAX_COUNT + 1;
						USPRINTF("%s %d: virtual wall detected! BumperCount = COMPLICATED_AREA_BUMPER_MAX_COUNT + 1! %d\n", __FUNCTION__, __LINE__, Temp_Rcon_Status);
						OBS_OFF();
						#ifdef VIRTUAL_WALL
						WallFollowMulti_End_VirtualWall = 1;						
						if(WallFollowMulti_Pass_VirtualWall==0)							
						{
							WallFollowMulti_Pass_VirtualWall = 1;
							Start_WallFollowMulti_VirtualWall_Point.X = Map_GetXCount();
							Start_WallFollowMulti_VirtualWall_Point.Y = Map_GetYCount();							
							End_WallFollowMulti_VirtualWall_Point = Start_WallFollowMulti_VirtualWall_Point;
						}
						else
						{
							End_WallFollowMulti_VirtualWall_Point.X = Map_GetXCount();
							End_WallFollowMulti_VirtualWall_Point.Y = Map_GetYCount();
						}
						#endif
						Pass_Virtual_Wall = 1;
						if (Temp_Rcon_Status & (RconFR_Wall )) {
								Reset_Front_VirtualWall();
								#ifdef RIGHT_WALL
								Turn_Left(Turn_Speed, 450);
								#endif
								Virtual_Cell.X = Map_GetXPos();
								Virtual_Cell.Y = Map_GetYPos();
								enable_br_bl_rcon = 2;							
						} else if (Temp_Rcon_Status & RconR_Wall) {
							if ( TwoPointsDistance(Virtual_Cell.X, Virtual_Cell.Y, Map_GetXPos(), Map_GetYPos()) > 1 )
							{
								Virtual_Cell.X = Map_GetXPos();
								Virtual_Cell.Y = Map_GetYPos();
								Reset_Front_VirtualWall();
								#ifdef RIGHT_WALL
								Turn_Left(Turn_Speed, 700);
								#endif							
								enable_br_bl_rcon = 2;
							}
						} else if (Temp_Rcon_Status & (RconL_Wall| RconFL_Wall)) {
							if ( TwoPointsDistance(Virtual_Cell.X, Virtual_Cell.Y, Map_GetXPos(), Map_GetYPos()) > 1 )
							{
								Virtual_Cell.X = Map_GetXPos();
								Virtual_Cell.Y = Map_GetYPos();
								Reset_Front_VirtualWall();
								#ifdef RIGHT_WALL
								Turn_Left(Turn_Speed, 300);
								#endif							
								enable_br_bl_rcon = 2;
							}							
						} 
					}
					else if ((Temp_Rcon_Status & (RconBR_Wall|RconBL_Wall ))&&(enable_br_bl_rcon==1))
					{		
						OBS_OFF();
						Reset_Front_VirtualWall();
						if((Temp_Rcon_Status & (RconBR_Wall|RconBL_Wall ))!=(RconBR_Wall|RconBL_Wall ))
						{
							#ifdef RIGHT_WALL
							Turn_Left(Turn_Speed, 700);
							#endif	
						}							
						Virtual_Cell.X = Map_GetXPos();
						Virtual_Cell.Y = Map_GetYPos();										
						Pass_Virtual_Wall = 1;
						enable_br_bl_rcon = 0;						
					}
				}
				else
				{
					Reset_Front_VirtualWall();	
				}
				if ( TwoPointsDistance(Virtual_Cell.X, Virtual_Cell.Y, Map_GetXPos(), Map_GetYPos()) > 2 )
				{		
						if(enable_br_bl_rcon==2)
						{
							enable_br_bl_rcon = 1;
						}
						
						if ( TwoPointsDistance(Virtual_Cell.X, Virtual_Cell.Y, Map_GetXPos(), Map_GetYPos()) > 5 )
						{
							Pass_Virtual_Wall = 1;
							#ifdef VIRTUAL_WALL
							WallFollowMulti_End_VirtualWall = 0;
							#endif
						}
				}
				#endif
				#endif
			} else {
				if (Temp_Rcon_Status & Rcon_Signal_ALL) {
					if (Is_WorkFinish(Get_Room_Mode())) {
						Set_Clean_Mode(Clean_Mode_GoHome);
						ResetHomeRemote();
						USPRINTF("%s %d: Check: Virtual! break\n", __FUNCTION__, __LINE__);
						break;
					}
				}
				if (Temp_Rcon_Status & Rcon_Signal_All_T) {
					if (Is_MoveWithRemote()) {
						Set_Clean_Mode(Clean_Mode_GoHome);
						ResetHomeRemote();
						USPRINTF("%s %d: Check: Virtual 2! break\n", __FUNCTION__, __LINE__);
						break;
					}
					STOP_BRIFLY;
					#ifdef RIGHT_WALL
					if (Temp_Rcon_Status & RconFR_HomeT) {
						Turn_Left(Turn_Speed, 1200);
					} else if (Temp_Rcon_Status & RconFL_HomeT) {
						Turn_Left(Turn_Speed, 1300);
					} else if (Temp_Rcon_Status & RconL_HomeT) {
						Turn_Left(Turn_Speed, 1500);
					} else if (Temp_Rcon_Status & RconR_HomeT) {
						Turn_Left(Turn_Speed, 900);
					}
					#endif
					STOP_BRIFLY;
					Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
					ZoneBoundaryReachFlag = 0;
					Reset_Rcon_Status();
					Wall_Straight_Distance =DISTANCE_1CM;
					Reset_WallAccelerate();
				}
			}
		}

		#ifdef ZONE_WALLFOLLOW
		/*------------------------------------- Map Boundary -------------------------------------*/
		if (follow_type != Map_Wall_Follow_Left_Target && follow_type != Map_Wall_Follow_Left_Zone) {
			if(Check_Wall_Follow_Boundary(0))
			{
				ZoneBoundaryReachFlag = 1;
			}
		}
		else {
			if(WFM_boundary_check())
			{
				BoundaryReachFlag = 1;
				Reset_WallAccelerate();
				Wall_Straight_Distance =5*DISTANCE_1CM;
			}
		}
#else
		if(WFM_boundary_check())
		{
			BoundaryReachFlag = 1;
			Reset_WallAccelerate();
			Wall_Straight_Distance =5*DISTANCE_1CM;
		}
#endif

		/*---------------------------------------------------Bumper Event-----------------------*/
		if (Get_Bumper_Status() & LeftBumperTrig) {
			ZoneBoundaryReachFlag = 0;
			chargercon_enable_r = 0;
			STOP_BRIFLY;
			Stop_Brifly();
			CheckGyroCalibrationTime(120);
			OBS_ON();
			#ifdef VIRTUAL_WALL
			WallFollowMulti_End_VirtualWall = 0;
			#endif
			if(Get_WallAccelerate()>2000)
			{
				Jam=0;
			}			
			WFM_update();			
			/*WFM_wall_move_back(7*DISTANCE_1CM);*///vin
			WFM_Left_move_back(RUN_SPEED_8,5*DISTANCE_1CM,700);						
			WFM_update();
			if (Get_WallAccelerate() > 200) {
				if (Is_Bumper_Jamed()) {
					Reset_Touch();
					CM_TouringCancel();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					USPRINTF("%s %d: Check: Bumper 1! break\n", __FUNCTION__, __LINE__);
					break;
				}
			}
			Reset_Wheel_Step();
			Reset_WallAccelerate();				
			/*STOP_BRIFLY;
			Turn_Left(Turn_Speed, 700);
			STOP_BRIFLY;
			WFM_update();
			Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
			Wall_Straight_Distance =6*DISTANCE_1CM;*///vin
			Wall_Straight_Distance=5*DISTANCE_1CM;
			Wall_Distance=Wall_High_Limit;	
		}

		if (Get_Bumper_Status() & RightBumperTrig) {
			ZoneBoundaryReachFlag = 0;
			chargercon_enable_r = 0;
			OBS_ON();
			#ifdef VIRTUAL_WALL
			WallFollowMulti_End_VirtualWall = 0;
			#endif			
			Set_Wheel_Speed(0, 0);			
			Reset_TempPWM();
			delay(10);
			Stop_Brifly();
			CheckGyroCalibrationTime(120);
			#ifdef XP_WALL
			  if(Get_Xp_RWall_ADC()>(Wall_Low_Limit)){
					Wall_Distance = Get_Xp_RWall_ADC()/2;//3
				}
				else {
					Wall_Distance+=200;
				}			
			#else
			  if(Get_RWall_ADC()>(500)){
					Wall_Distance = Get_RWall_ADC();//3
				}
				else {
					Wall_Distance-=200;
				}			
			#endif

				WFM_update();
				if ((Get_Bumper_Status() & LeftBumperTrig)||Is_FrontOBS_Trig()) {
					/*WFM_move_back(7*DISTANCE_1CM);*///vin
					WFM_Left_move_back(RUN_SPEED_8,5*DISTANCE_1CM,700);	
					WFM_update();
					if (Is_Bumper_Jamed()) {
						Reset_Touch();
						CM_TouringCancel();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						USPRINTF("%s %d: Check: Bumper 2! break\n", __FUNCTION__, __LINE__);
						break;
					}
					/*STOP_BRIFLY;
					Turn_Left(Turn_Speed , 600);*///vin
					#ifdef ZONE_WALLFOLLOW
					if ( CM_IsSingleRoomMode() == 0 ) {
					if ( TwoPointsDistance(tmpCell3.X, tmpCell3.Y, Map_GetXPos(), Map_GetYPos()) < 3 ){
						bumperCount++;
					} else {
						tmpCell3.X = Map_GetXPos();
						tmpCell3.Y = Map_GetYPos();
					}
					USPRINTF("%s %d Bumper count: %d!\n",  __FUNCTION__, __LINE__, bumperCount);
					}
					#endif
					Wall_Straight_Distance = MFW_Setting[follow_type].right_bumper_val; //150;
													
					Wall_Straight_Distance=5*DISTANCE_1CM;
					Wall_Distance=Wall_High_Limit;						
				} else {//right
					WFM_update();
					/*WFM_wall_move_back(7*DISTANCE_1CM);
					Wall_Distance=Wall_Low_Limit;	*///vin
					if(Jam<20)
					{
//						special_flag=1;	
						WFM_Right_move_back(RUN_SPEED_6,5*DISTANCE_1CM,500);
						Wall_Straight_Distance=5*DISTANCE_1CM;
						Reset_WallAccelerate();						
					}
					else
					{ 
						WFM_Left_move_back(RUN_SPEED_8,5*DISTANCE_1CM,700);
					}						
					WFM_update();
					if (Is_Bumper_Jamed()) {
						Reset_Touch();
						CM_TouringCancel();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						USPRINTF("%s %d: Check: Bumper 3! break\n", __FUNCTION__, __LINE__);
						break;
					}
					/*STOP_BRIFLY;*///vin
					#ifdef ZONE_WALLFOLLOW
					if ( CM_IsSingleRoomMode() == 0 ) {
					if ( TwoPointsDistance(tmpCell3.X, tmpCell3.Y, Map_GetXPos(), Map_GetYPos()) < 3 ){
						bumperCount++;
					} else {
						tmpCell3.X = Map_GetXPos();
						tmpCell3.Y = Map_GetYPos();
					}
					USPRINTF("%s %d Bumper count: %d!\n",  __FUNCTION__, __LINE__, bumperCount);
					}
					#endif
				}
				if (Get_WallAccelerate() < 1000) {
					Jam++;
				} else {
					Jam = 0;
				}
				Reset_WallAccelerate();
				/*Wall_Straight_Distance = 5*DISTANCE_1CM;
				STOP_BRIFLY;*/
				WFM_update();
				Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
				for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++) {
					Left_Wall_Buffer[Temp_Counter] = 0;
				}
				Reset_Wheel_Step();
				if (Wall_Distance < Wall_Low_Limit)Wall_Distance = Wall_Low_Limit;
				if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;								
			}
			if(Get_WallAccelerate()>1000)
			{
				Jam=0;
			} 		
			if (Jam > 80) {
				Set_Clean_Mode(Clean_Mode_Userinterface);
				Set_Error_Code(Error_Code_Stuck);
				CM_TouringCancel();
				USPRINTF("%s %d: Check: Bumper 4! break\n", __FUNCTION__, __LINE__);
				break;
			}

		/*------------------------------------------------------Cliff Event-----------------------*/
		if (Get_Cliff_Trig()) {
			{
				ZoneBoundaryReachFlag = 0;
				chargercon_enable_r = 0;
				OBS_ON();
				#ifdef VIRTUAL_WALL
				WallFollowMulti_End_VirtualWall = 0;
				#endif
				WFM_update();
				WFM_move_back(9*DISTANCE_1CM);
				WFM_update();
				if (Get_Cliff_Trig() == (Status_Cliff_All)) {
					Set_Clean_Mode(Clean_Mode_Userinterface);
					CM_TouringCancel();
					USPRINTF("%s %d: Check: Cliff 1! break\n", __FUNCTION__, __LINE__);
					break;
				}
				if (Get_Cliff_Trig()) {
					if (Cliff_Escape()) {
						Set_Error_Code(Error_Code_Cliff);
						Set_Clean_Mode(Clean_Mode_Userinterface);
						CM_TouringCancel();
						USPRINTF("%s %d: Check: Cliff 2! break\n", __FUNCTION__, __LINE__);
						break;
					}
				}
					STOP_BRIFLY;
					Turn_Left(Turn_Speed , 750);
					STOP_BRIFLY;
					Stop_Brifly();
					CheckGyroCalibrationTime(120);
					WFM_update();
					Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
					Reset_WallAccelerate();
					Wall_Straight_Distance = 5*DISTANCE_1CM;

				for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++) {
					Left_Wall_Buffer[Temp_Counter] = 0;
				}
				Reset_Wheel_Step();
			}
		}
		
		if (Wall_Distance >=Wall_Low_Limit && (ZoneBoundaryReachFlag == 0)) {//200
			Left_Wall_Buffer[2] = Left_Wall_Buffer[1];
			Left_Wall_Buffer[1] = Left_Wall_Buffer[0];
			#ifdef XP_WALL
			Left_Wall_Buffer[0] = Get_Xp_RWall_ADC();			
			#else
			Left_Wall_Buffer[0] = Get_RWall_ADC();
			#endif		
			if (Left_Wall_Buffer[0] < 100) {
				if ((Left_Wall_Buffer[1] - Left_Wall_Buffer[0]) > (Wall_Distance / 25)) {
					if ((Left_Wall_Buffer[2] - Left_Wall_Buffer[1]) > (Wall_Distance / 25)) {
						if (Get_WallAccelerate() > 300) {
							if ((Get_LeftWheel_Speed() - Get_RightWheel_Speed()) >= -3) {
								WFM_update();
								Move_Forward(RUN_SPEED_9, RUN_SPEED_9);
								delay(10);
								WFM_update();
								Reset_WallAccelerate();
								Wall_Straight_Distance = 6*DISTANCE_1CM;
							}
						}
					}
				}
			}
		}

		/*------------------------------------------------------Short Distance Move-----------------------*/		
		if (((Get_WallAccelerate() < (uint32_t) Wall_Straight_Distance) && ((follow_type == Map_Wall_Follow_Left_Zone) || (BoundaryReachFlag))) || (ZoneBoundaryReachFlag))
		{
			WFM_update();
			if(special_flag)
			{
				Move_Forward(RUN_SPEED_3, RUN_SPEED_8);
			}
			else
			{
				if (Get_RightWheel_Step() < 500) {
					if (Get_WallAccelerate() < 100) {
						Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
					} else {
						Move_Forward(RUN_SPEED_9, RUN_SPEED_9);
					}
				} else {
					if(Get_WallAccelerate()<20*DISTANCE_1CM)
					{
						Move_Forward(RUN_SPEED_10, RUN_SPEED_10);
					}
					else Move_Forward(RUN_SPEED_14, RUN_SPEED_14);
				}			
			}
			
			WFM_update();
		} else {
			special_flag=0;
			/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
			BoundaryReachFlag = 0;			
			#ifdef OBS_DYNAMIC
			if (!Is_FrontOBS_Trig()) 
			#else
			if (Get_FrontOBS() < MFW_Setting[follow_type].front_obs_val) 
			#endif
			{
				#ifdef XP_WALL
				Proportion = Get_Xp_RWall_ADC()/2;			
				#else
				Proportion = Get_RWall_ADC();
				#endif

				Proportion = Proportion * 100 / Wall_Distance;

				Proportion -= 100;

				Delta = Proportion - Previous;
				
				Delta /= 3;
				
				if (Wall_Distance > 300)//over left
				{
					Right_Wall_Speed = RUN_SPEED_12 + Proportion / 12 + Delta / 5;
					Left_Wall_Speed = RUN_SPEED_12 - Proportion / 10 - Delta / 5;
					if(Right_Wall_Speed > RUN_SPEED_14)Right_Wall_Speed = RUN_SPEED_14;
					if (Left_Wall_Speed > RUN_SPEED_14) {
						Right_Wall_Speed = RUN_SPEED_4;
						Left_Wall_Speed = RUN_SPEED_14;
					}
				} else if (Wall_Distance > 200){
					Right_Wall_Speed  = RUN_SPEED_10 + Proportion / 15 + Delta / 7;
					Left_Wall_Speed = RUN_SPEED_10 - Proportion / 12 - Delta / 7;
					if(Right_Wall_Speed > RUN_SPEED_12)Right_Wall_Speed = RUN_SPEED_12;
					if (Left_Wall_Speed > RUN_SPEED_12) {
						Right_Wall_Speed = RUN_SPEED_3;
						Left_Wall_Speed = RUN_SPEED_13;
					}
				}
			  else{
					Right_Wall_Speed  = RUN_SPEED_8 + Proportion / 18 + Delta / 10;
					Left_Wall_Speed = RUN_SPEED_8 - Proportion / 15 - Delta / 10;
					if(Right_Wall_Speed > RUN_SPEED_9)Right_Wall_Speed = RUN_SPEED_9;
					if(Left_Wall_Speed > RUN_SPEED_9) {
						Right_Wall_Speed = RUN_SPEED_3;//3
						Left_Wall_Speed = RUN_SPEED_9;
					}
					/*if(Left_Wall_Speed>20)Left_Wall_Speed=20;
					if(Left_Wall_Speed<4)Left_Wall_Speed=4;
					if(Right_Wall_Speed<4)Right_Wall_Speed=4;
					if((Left_Wall_Speed-Right_Wall_Speed)>5)
					{
						Left_Wall_Speed = Right_Wall_Speed+5;
					}*/
				}

			  /*slow move if left obs near a wall*/				
				if(Get_RWall_ADC()>800)
				{
					Wall_Distance+=300;
					if(Wall_Distance>1000)Wall_Distance=1000;
				}
				else if(Get_RWall_ADC()>400)
				{
					Wall_Distance+=10;
					if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;
				}
				else if(Get_RWall_ADC()>200)
				{
					Wall_Distance+=5;
					if(Wall_Distance>400)Wall_Distance=400;
				}
				else
				{
					Wall_Distance+=1;
					if(Wall_Distance>300)Wall_Distance=300;				
				}
				
				if(Is_WallOBS_Near()){
					Left_Wall_Speed = Left_Wall_Speed/2;
					Right_Wall_Speed = Right_Wall_Speed/2;				
				}

				Previous = Proportion;

				if (Right_Wall_Speed < 0) {
					Right_Wall_Speed = 0;
				}
				if (Left_Wall_Speed < 0) {
					Left_Wall_Speed = 0;
				}				
				if (Right_Wall_Speed > RUN_SPEED_15) {
					Right_Wall_Speed = RUN_SPEED_15;
				}


				#ifdef VIRTUAL_WALL
				if(Pass_Virtual_Wall)
				{
					if((Left_Wall_Speed - Right_Wall_Speed)>Right_Wall_Speed)
					{
						Right_Wall_Speed = Left_Wall_Speed/2;//2
					}
				}
				#endif
//				if(Get_Rcon_Status()&(RconR_RIGHT|RconFR_RIGHT|RconFL_RIGHT|RconFR_LEFT|RconFL_LEFT))//Rcon_Signal_WALL_R
//				{
//					Move_Forward(RUN_SPEED_3, RUN_SPEED_9);
//					delay(1500);
//					Wall_Distance=Wall_High_Limit;
//					Reset_Rcon_Status();
//				}
//				else if(Get_Rcon_Status()&(RconR_LEFT))
//				{
//					Move_Forward(RUN_SPEED_3, RUN_SPEED_9);
//					Wall_Distance=Wall_High_Limit;
//					Reset_Rcon_Status();				
//				}
//				else
				{
					Move_Forward(Left_Wall_Speed, Right_Wall_Speed);
				}
				

				//If wall follow a long distance, then save offset
				if ( TwoPointsDistance(tmpCell2.X, tmpCell2.Y, Map_GetXPos(), Map_GetYPos()) > 5 ) {
					tmpCell2.X = Map_GetXPos();
					tmpCell2.Y = Map_GetYPos();
				}

				//Rotating in a short distance area
				if (Get_LeftWall_Step() - leftWheelOffset > Get_RightWall_Step() - rightWheelOffset) {
					R =Get_LeftWall_Step() - leftWheelOffset  - ( Get_RightWall_Step() - rightWheelOffset );
				}

				//If turing around at the same point
				if ((R > TURN_AROUND_CNT) && (WFM_IsWallFollowSamePointAndHeading_ZZ(crtCell, revWallFollowHeading) == 1))
				{
					USPRINTF("Isolated Wall Follow!\n");
					Reset_Wall_Step();				
					if ((follow_type == Map_Wall_Follow_Escape_Trapped_ZZ) || (follow_type == Map_Wall_Follow_To_Zone_Exit) || (follow_type == Map_Wall_Follow_To_Zone_Entrance))
					{
						if (follow_type == Map_Wall_Follow_Escape_Trapped_ZZ)
						{
							if (((Work_Timer - escape_trapped_timer_ZZ) > ESCAPE_TRAPPED_TIME_ZZ) || (SamePositionTime > 30))	//在同一个点超过5秒
							{
								USPRINTF("%s %d: escape timeout %d(%d, %d)\n", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME_ZZ, Work_Timer, escape_trapped_timer_ZZ);
								USPRINTF("%s %d: Check: Escape! Return 2\n", __FUNCTION__, __LINE__);
								return 2;
							}
						}
						
						if ((follow_type == Map_Wall_Follow_To_Zone_Exit) || (follow_type == Map_Wall_Follow_To_Zone_Entrance))
						{							
							if (((Work_Timer - wall_follow_to_zone_cell_timer) > ESCAPE_TRAPPED_TIME_ZZ) || (SamePositionTime > 30))//在同一个点超过5秒
							{
								USPRINTF("%s %d: escape timeout %d(%d, %d)\n", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME_ZZ, Work_Timer, wall_follow_to_zone_cell_timer);
								USPRINTF("%s %d: Check: Escape! Return 2\n", __FUNCTION__, __LINE__);
								return 2;
							}
						}
						Turn_Left(Turn_Speed,900);
						return 100;
					}
					isIsolatedWallFollow = 1;
					#ifdef ZONE_WALLFOLLOW
					//If follow type is left wall follow for zone
					if ( follow_type == Map_Wall_Follow_Left_Zone )
					{
						if ( Zone_GetZoneSize() != 0 ) {
							Zone_ForceAddZone( startCell.X, startCell.Y, Map_GetXPos(), Map_GetYPos() );
							if ( isMiniRoom != 1 )
								isNeedCheckMiniRoom = 0;
								#ifdef ALIGNMENT_ANGLE_WALL_ENABLE
								if ( lineSegment(lineABCFirst) == 1 ) 
								{
									calculateAngleOffset(lineABCFirst);
									Gyro_SetOffset((int16_t)(angleOffset));
								}
								#endif
							//Set zone entrance robot heading
							USPRINTF("Set Entrance Robot Heading!\n");
							Zone_SetZoneEntranceRobotHeading(revEntranceHeading, Zone_GetCurrentZoneIdx());
							USPRINTF("%s %d: Check: Isolated area! Break!\n", __FUNCTION__, __LINE__);
							return 0;
						}
					}
					#endif
					USPRINTF("%s %d: Check: Isolated Wall Follow! break\n", __FUNCTION__, __LINE__);
					break;
				}

				if (Get_WallAccelerate() > 750) {
					Set_Left_Brush(ENABLE);
					Set_Right_Brush(ENABLE);
				}
			} else {
				STOP_BRIFLY;
				if (Get_WallAccelerate() < 2000) {
					Jam++;
				}				
				WFM_Left_move_back(RUN_SPEED_8,8*DISTANCE_1CM,900);
				/*WFM_move_back(5*DISTANCE_1CM);
				Turn_Left(Turn_Speed , 750);
				STOP_BRIFLY;*/
				WFM_update();
				Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
				Reset_Wheel_Step();
				Wall_Distance = Wall_High_Limit;
			}
		}
	}
	USPRINTF("%s %d: Check: End! Return 0\n", __FUNCTION__, __LINE__);
	return 0;
}

/**
 * Set wallfollow distance
 * @param Dis distance
 */
void WFM_SetWallFollowDistance(int32_t Dis)
{
	Map_Wall_Follow_Distance = Dis;
}

void WFM_move_back(uint16_t dist)
{
	uint16_t Counter_Watcher = 0;
	uint16_t Temp_Speed = 10;
	uint8_t motor_check=0;
	STOP_BRIFLY;
	WFM_update();
	Set_Dir_Backward();
	Set_Wheel_Speed(RUN_SPEED_3, RUN_SPEED_3);
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;

	while ((Get_LeftWheel_Step() < dist) || (Get_RightWheel_Step() < dist)) 
	{
		Temp_Speed = Get_LeftWheel_Step() / 3 +  8;
		if (Temp_Speed > RUN_SPEED_5) {
			Temp_Speed = RUN_SPEED_5;
		}
		Set_Wheel_Speed(Temp_Speed,Temp_Speed);

		WFM_update();
		delay(10);
		Counter_Watcher++;
		if (Counter_Watcher > 3000) {
			if(Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if (Touch_Detect()) {
			return;
		}
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}
	WFM_update();
	Reset_TempPWM();
}

void WFM_Right_move_back(uint8_t speed,uint16_t dist,uint16_t angle)
{
	uint16_t Counter_Watcher = 0;
	uint8_t motor_check=0;
	STOP_BRIFLY;
	WFM_update();
	Set_Dir_Backward();
	Set_Wheel_Speed(RUN_SPEED_4, RUN_SPEED_5);
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;

	while ((Get_RightWheel_Step() < 3*DISTANCE_1CM)) 
	{
		WFM_update();
		delay(10);
		Counter_Watcher++;
		if (Counter_Watcher > 5000) {
			if(Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if (Touch_Detect()) {
			return;
		}	
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}
	
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;

	while ((Get_RightWheel_Step() < 6*DISTANCE_1CM)) 
	{
		Set_RightWheel_Step(Get_LeftWheel_Step());
		if(Get_RightWheel_Step()<6*DISTANCE_1CM)
		{
			Set_Dir_Left();
			Set_Wheel_Speed(RUN_SPEED_5, RUN_SPEED_5);		
		}
		else if(Get_RightWheel_Step()<15*DISTANCE_1CM)
		{
			Set_Dir_Left();
			Set_Wheel_Speed(RUN_SPEED_5, RUN_SPEED_7);
		}
		else
		{
			Set_Dir_Forward();
			Set_Wheel_Speed(RUN_SPEED_7, RUN_SPEED_5);		
		}
		
		WFM_update();
		delay(10);
		Counter_Watcher++;
		if (Counter_Watcher > 5000) {
			if(Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if (Touch_Detect()) {
			return;
		}
		
//		if (Get_Bumper_Status()) {
//			return;
//		}		

		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}	
	WFM_update();
}

void WFM_Left_move_back(uint8_t speed,uint16_t dist,uint16_t angle)
{
	uint16_t Counter_Watcher = 0;
	uint8_t motor_check=0,flag=1;
	STOP_BRIFLY;
	WFM_update();
	Set_Dir_Backward();	
	Set_Wheel_Speed(RUN_SPEED_3, RUN_SPEED_3);
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;

	while ((Get_RightWheel_Step() < 12*DISTANCE_1CM))
	{
		WFM_update();
		if(Get_RightWheel_Step()<3*DISTANCE_1CM)
		{
			Set_Wheel_Speed(RUN_SPEED_5,RUN_SPEED_5);
		}
		else if((Get_RightWheel_Step()<=6*DISTANCE_1CM)&&flag)
		{
			if (Get_Bumper_Status())
			{
				Set_RightWheel_Step(6*DISTANCE_1CM);
				Reset_TempPWM();
				flag=0;
			}
			Set_Wheel_Speed(0,RUN_SPEED_7);
		}
		else
		{
			Set_Wheel_Speed(RUN_SPEED_7,RUN_SPEED_7);
		}
		
		delay(10);
		Counter_Watcher++;
		if (Counter_Watcher > 5000) {
			if(Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if (Touch_Detect()) {
			return;
		}	
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}
	
	Set_Dir_Left();
	Set_Wheel_Speed(RUN_SPEED_5, RUN_SPEED_9);
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;

	while ((Get_RightWheel_Step() < angle)) 
	{
		WFM_update();
		delay(10);
		Counter_Watcher++;
		if (Counter_Watcher > 3000) {
			if(Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if (Touch_Detect()) {
			return;
		}
		if (Get_Bumper_Status()) {
			Reset_TempPWM();
			return;
		}
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}	
	WFM_update();
}

void WFM_wall_move_back(uint16_t distance)
{
	uint16_t Temp_Speed = 10;
	uint16_t Counter_Watcher = 0;
	uint8_t motor_check=0;
	
	STOP_BRIFLY;
	WFM_update();
	Set_Dir_Backward();
	Reset_TempPWM();
	Set_Wheel_Speed(RUN_SPEED_3, RUN_SPEED_3);
	delay(200);
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;

	while ((Get_LeftWheel_Step() < distance) || (Get_RightWheel_Step() < distance)) {
		WFM_update();
		Temp_Speed = Get_LeftWheel_Step() / 3 +  8;
		if (Temp_Speed > RUN_SPEED_5) {
			Temp_Speed = RUN_SPEED_5;
		}
		Set_Wheel_Speed(Temp_Speed,Temp_Speed);

		delay(10);
		Counter_Watcher++;
		if (Counter_Watcher > 5000) {
			if (Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if (Touch_Detect()) {
			return;
		}
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}
	WFM_update();
	Set_Dir_Forward();
	Set_Wheel_Speed(0,0);
	Reset_TempPWM();
}


/**
 * Check if the robot is isolated wall follow
 * @return      1: isolated wall follow;
 *         others: not isolated wall follow
 */
uint8_t WFM_IsIsolatedWallFollow()
{
	return isIsolatedWallFollow;
}
#ifdef ZONE_WALLFOLLOW
/**
 * Get Return  Type
 * @return zone return type, if 1, then go to entrance, else go to exit
 */
uint8_t WFM_GetZoneReturnType()
{
	return returnCellType;
}

/**
 * Check if the robot is wall follow too far
 * @return      1: wall follow too far;
 *         others: not too far
 */
uint8_t WFM_IsWallFollowTooFar()
{
	return isWallFollowTooFar;
}

/**
 * Check if the robot is in mini room mode
 * @return      1: the robot is in the mini room mode
 *         others: not in mini room mode
 */
uint8_t WFM_IsMiniRoom() {
	return isMiniRoom;
}

void WFM_ResetIsMiniRoom(void) 
{
	 isMiniRoom = 0;
}

/**
 * Enable or disable mini room checking
 * @param status 1: need to check, others: no need to check
 */
void WFM_SetMiniRoomChecking(uint8_t status) {
	isNeedCheckMiniRoom = status;
}
#endif

/**
 * Set directly wall follow, if val = 1, the robot will directly wall follow without
 * waiting for hitting bumper;
 *
 * @param val      1: Directly wall follow without hitting bumper
 *            others: waiting for hitting bumper then begin wall follow
 */
void WFM_SetDirectWallFollow(uint8_t val) {
	isDirectWallFollow = val;
}

/**
 * Check if the robot find the wall
 * @return MapFindWallType: Map_Find_Wall_Not_Found,
 *	                        Map_Find_Wall_Timeout,
 *	                        Map_Find_Wall_Over_Distance,
 *	                        Map_Find_Wall_Found,
 *	                        Map_Find_Wall_No_Need_To_Find,
 *
 *
 */
MapFindWallType WFM_GetWallFoundStatus(void) {
	return wallFoundStatus;
}

void WFM_SetWallFoundStatus(MapFindWallType status) {
	wallFoundStatus = status;
}


