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
#ifndef __MAP_H__
#define __MAP_H__

#include "SysInitialize.h"
#include "Mymath.h"

#define MAP 0
#define SPMAP 1
#define WALL_MAP_SIZE 100//100 edit by vin


#define ROBOT_ON_CELLCENTER	1
#define ROBOT_ON_CELLFRONT	2
#define ROBOT_ON_CELLLEFT		3
#define ROBOT_ON_CELLRIGHT	4
#define ROBOT_ON_CELLBACK		5


typedef enum {
  UNCLEAN  = 0,
  CLEANED = 1,
  BLOCKED = 2,
  BLOCKED_OBS = 2,
  BLOCKED_BUMPER = 2,
  BLOCKED_CLIFF = 2,
  BLOCKED_BOUNDARY = 3,
} CellState_t;

typedef enum {
  EAST = 0,
  NORTH_EAST = 450,
  NORTH = 900,
  NORTH_WEST = 1350,
  WEST = 1800,
  SOUTH_WEST = 2250,
  SOUTH = 2700,
  SOUTH_EAST = 3150,
  DIR_NONE = 3600,
}DirectionCardinal_t;

typedef struct
{
	int16_t east;
	int16_t west;
	int16_t north;
	int16_t south;
	int32_t east_cnt;
	int32_t west_cnt;
	int32_t north_cnt;
	int32_t south_cnt;
}Boundary_t;



void Map_Initialize(void);
void Map_ResetBoundaryFlag(void);
int32_t Map_GetRobotCountX(void);
int32_t Map_GetRobotCountY(void);
Point32_t Map_GetRobotCount(void);
int16_t Map_GetRobotCellX(void);
int16_t Map_GetRobotCellY(void);
void Map_SetRobotCell(Point16_t cell);
Point16_t Map_GetRobotCell(void);
uint8_t Map_TwoCellMatch(Point16_t cell_a,Point16_t cell_b);

void Map_MoveRobotCountTo(double x, double y);
void Map_SetRobotCount(double x, double y);

int32_t Map_GetRelativeX(uint16_t heading, int16_t offset_lat, int16_t offset_long);
int32_t Map_GetRelativeY(uint16_t heading, int16_t offset_lat, int16_t offset_long);
void Map_GetRelativeXY(uint16_t heading, int16_t offset_lat, int16_t offset_long,int32_t *x,int32_t *y,double rad);
Point16_t Map_GetRelativeXY2(uint16_t heading, int16_t offset_lat, int16_t offset_long);
Point16_t Map_GetRelativeByCell(Point16_t target_cell,int16_t heading, int16_t offset_lat, int16_t offset_long) ;
int16_t Map_GetLateralOffset(uint16_t heading);
int16_t Map_GetLongitudinalOffset(uint16_t heading);

int16_t nextXID(uint16_t heading, int16_t offset_lat, int16_t offset_long);
int16_t nextYID(uint16_t heading, int16_t offset_lat, int16_t offset_long);

void Map_GetRelativePosition(uint16_t heading, int16_t *x, int16_t *y);

CellState_t Map_GetCell(int16_t x, int16_t y);
void Map_SetCell(int16_t x, int16_t y, CellState_t value);
uint8_t Map_SetBlockIfUnclean(uint8_t id,int16_t x,int16_t y,CellState_t value);

void Map_ClearBlocks(void);

int32_t Map_CellToCount(int16_t cell);
int16_t Map_CountToCell(double count);
int16_t Map_CountToCell_My(double count);
Point32_t Map_CellToPoint( Point16_t cell );
Point16_t Map_PointToCell( Point32_t pnt );
Point16_t Map_PointToCell_My( Point32_t pnt );

void Map_Set_Cells(int8_t count, int16_t cell_x, int16_t cell_y, CellState_t state);
void Map_ClearCleanedCells(void);

uint8_t Map_GetMapArray(int16_t x,int16_t y);
void Map_SetMapArray(int16_t x,int16_t y,uint8_t data);
int16_t Map_CellToMap(int16_t data);

uint32_t Map_GetCellAcreage(void);
int16_t Map_GetXMin(void);
int16_t Map_GetXMax(void);
int16_t Map_GetYMin(void);
int16_t Map_GetYMax(void);
void Map_Adjust_Mapsize(int16_t x,int16_t y);


uint8_t Map_IsBlockAccessable(int16_t x,int16_t y,DirectionCardinal_t dir);
int8_t Map_IsBlockCleanable(int16_t x, int16_t y);
uint8_t Map_IsBlock_Access(int16_t x,int16_t y);
uint8_t Map_IsBlock_Access2(uint8_t offset,int16_t x,int16_t y);
int8_t Map_IsBlockAllCleaned(int16_t x, int16_t y);
uint8_t Map_BrushBlockUnclean(int16_t x, int16_t y);
uint8_t Map_Cell_Blocked(int16_t x, int16_t y);
uint8_t Map_IsRobotAccessible(int16_t x, int16_t y);

uint8_t Map_WallTrackUpdate(Point16_t pos);
uint8_t Map_WallTrackExited(Point16_t pos);
uint8_t Map_WallTrackAdd(Point16_t pos);
uint8_t Map_GetWallTrackCnt(void);
void Map_WallTrackClear(void);
void Map_WallTrackShowAll(void);
void Map_WallTrackSetObs(WallDir_t wall_dir);
void Map_FillGap(void);

int16_t Map_GetBoundaryWidth(void);
int16_t Map_GetBoundaryHeight(void);
int32_t Map_GetBoundaryWidthCnt(void);
int32_t Map_GetBoundaryHeightCnt(void);

void Map_SetBoundary(int16_t width,int16_t height);

Point16_t Map_GetHomeCell(void);
void Map_SetHomeCell(Point16_t home);

Boundary_t Map_GetBoundary(void);

void Map_SetXBoundary(int16_t x,int16_t start_y,int16_t end_y);
void Map_SetYBoundary(int16_t y,int16_t start_x,int16_t end_x);
uint8_t Map_ReachBoundary(void);
void Map_AdjustBoundary(void);
void Map_StoreMapEdge(void);
void Map_LoadMapEdge(void);
int16_t Map_GetBoundaryWest(void);
Point16_t Map_GridToCell(int16_t x,int16_t y);
uint8_t Math_TwoCell_Equal(Point16_t start_cell,Point16_t exit_cell);
uint8_t Map_IsReach_NewCell(Point16_t cur_cell,Point16_t *pre_cell);

void Map_SetTarget_Cleaned(int16_t x,int16_t y);

uint8_t Path_IsCell_ReachBlocked(uint8_t move_type,Point16_t cur_cell,int16_t heading);

extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

#endif /* __MAP_H */
