/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2011
  * @brief   System Initialize
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __Spot_H
#define __Spot_H

#include "SysInitialize.h"
#include "Mymath.h"


#define SPOT_RIGHT_OUT	       1
#define SPOT_RIGHT_IN 	       2
#define SPOT_LEFT_OUT 	       4
#define SPOT_LEFT_IN	         8
#define SPOT_FIRST_ROUND       10

typedef enum
{
	SPOT_DIR_NONE = 0,
	SPOT_DIR_CW = -1,
	SPOT_DIR_CCW = 1,
	SPOT_DIR_IN	= 0,
	SPOT_DIR_OUT = 1,
}SpotDir_t;

typedef enum
{
	SPOT_LANE_NONE = 0,
	SPOT_LANE_EAST = 1,
	SPOT_LANE_NORTH = 2,
	SPOT_LANE_WEST = 3,
	SPOT_LANE_SOUTH = 4,
}SpotLaneDir_t;


void Spot_Mode(void);
uint8_t Random_Dirt_Event(void);


void Spot_SetDir(SpotDir_t dir);
SpotDir_t Spot_GetDir(void);
void Spot_ReverseDir(void);
void Spot_SetSpiralDir(SpotDir_t dir);
SpotDir_t Spot_GetSpiralDir(void);

uint8_t Spot_IsLaneCleaned(uint8_t idx ,Point16_t start_cell, SpotDir_t dir);
void Spot_PathPlanning(Point16_t start_cell);
Point16_t Spot_GetNextLanePos(Point16_t start_cell,uint8_t dir,uint8_t step);

void PathPoint_SortPoints(void);


void Spot_SetClearFlag(uint8_t flag);
uint8_t Spot_GetClearFlag(void);

uint8_t Spot_OverLane(Point32_t start_cnt);
uint8_t Spot_InnerLane(uint8_t idx);
void Spot_SetLaneIdx(uint8_t idx);
uint8_t Spot_GetLaneIdx(void);
void Spot_ClearCleanCells(int8_t idx);
void Spot_ExpandBockedCells(void);
void Spot_UpdateWallCell( Point16_t cell);
void Spot_ResetWallCell(void);
uint8_t Spot_CheckSameWallCell(Point16_t cell);
void Spot_CreateBoundary(uint8_t idx) ;
SpotLaneDir_t Spot_GetPointLaneDir(Point16_t point);
uint16_t Spot_GetPointLaneIdx(Point16_t point) ;
#endif /*----Behaviors------*/





