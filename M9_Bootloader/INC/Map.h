#ifndef __MAP_H__
#define __MAP_H__

#include "SysInitialize.h"
#include "config.h"
#include "MyMath.h"

#define MAP 0

typedef enum {
  UNCLEAN  = 0,
  CLEANED = 1,
  BLOCKED = 2,
  BLOCKED_OBS = 2,
  BLOCKED_BUMPER = 2,
  BLOCKED_CLIFF = 2,
  BLOCKED_BOUNDARY = 3,
  TARGET_CLEAN = 13,
  TARGET = 14,
  COST_NO = 0,
  COST_1 = 1,
  COST_2 = 2,
  COST_3 = 3,
  COST_4 = 4,
  COST_5 = 5,
  COST_PATH = 6,
  COST_HIGH = 7,
} CellState;

typedef enum {
  NORTH = 0,
  NORTH_EAST = 450,
  EAST = 900,
  SOUTH_EAET = 1350,
  SOUTH = 1800,
  SOUTH_WEST = 2250,
  WEST = 2700,
  NORTH_WEST = 3150,
  NONE = 3600,
} Direction_Cardinal;

extern int16_t xRangeMin, xRangeMax, yRangeMin, yRangeMax;

void Map_Initialize(void);

int32_t Map_GetXCount(void);
int32_t Map_GetYCount(void);
int16_t Map_GetXPos(void);
int16_t Map_GetYPos(void);
Point16_t Map_GetCurrentCell(void);
Point32_t Map_GetCurrentPoint(void);

void Map_MoveTo(double x, double y);
void Map_SetPosition(double x, double y);

int32_t Map_GetRelativeX(uint16_t heading, int16_t offset_lat, int16_t offset_long);
int32_t Map_GetRelativeY(uint16_t heading, int16_t offset_lat, int16_t offset_long);
int32_t Map_GetRelativeX_ByXCount(int32_t XCount, uint16_t heading, int16_t offset_lat, int16_t offset_long);
int32_t Map_GetRelativeY_ByYCount(int32_t YCount, uint16_t heading, int16_t offset_lat, int16_t offset_long);

int16_t Map_GetLateralOffset(uint16_t heading);
int16_t Map_GetLongitudinalOffset(uint16_t heading);

int16_t nextXID(uint16_t heading, int16_t offset_lat, int16_t offset_long);
int16_t nextYID(uint16_t heading, int16_t offset_lat, int16_t offset_long);

void Map_GetRelativePosition(uint16_t heading, int16_t *x, int16_t *y);

CellState Map_GetCell(uint8_t id, int16_t x, int16_t y);
void Map_SetCell(uint8_t id, int32_t x, int32_t y, CellState value);

void Map_ClearBlocks(void);
int32_t Map_DistanceLimit(uint16_t heading);

int32_t cellToCount(int16_t distance);
int16_t countToCell(double count);
Point32_t Map_CellToPoint( Point16_t cell );
Point16_t Map_PointToCell( Point32_t pnt );

void Map_Set_Cells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state);

void Map_PrintDebug(uint8_t id);
void Map_PrintPosition(void);

#ifdef VIRTUAL_WALL
void Map_SetXCount(int32_t data);
void Map_SetYCount(int32_t data);
void Draw_VirtualWall_Line(Point32_t start,Point32_t end,CellState value);
#endif
void Map_Set_Cells_IgnoreCells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state, CellState IgnoreCellState);

#endif /* __MAP_H */

