#ifndef __ZONE_WALLFOLLOW_H_
#define __ZONE_WALLFOLLOW_H_

#include "MyMath.h"
#include "limits.h"
#include "Map.h"
#include "Debug.h"

typedef struct {
	Point16_t zone;
	Point16_t entranceCell;
	Point16_t exitCell;
	uint16_t entranceHeading;
	uint16_t exitHeading;
} ZoneWallFollowType;

typedef enum {
	ZC_EAST   = 0x1,
	ZC_SOUTH  = 0x2,
	ZC_WEST   = 0x4,
	ZC_NORTH  = 0x8,
} ZoneConnectedState;

//Initial
void Zone_Initialize(void);

//Add zone
int8_t Zone_IsAddNewZone( int16_t start_x, int16_t start_y,
                                     int16_t des_x, int16_t des_y );
int8_t Zone_AddZone( int16_t start_x, int16_t start_y,
                               int16_t exit_x, int16_t exit_y,
                               int16_t zone_x, int16_t zone_y );
int8_t Zone_ForceAddZone( int16_t start_x, int16_t start_y, int16_t des_x, int16_t des_y );
void Zone_SetZoneContinual( uint8_t val );
uint8_t Zone_GetCurrentContinualZoneIdx(void);

//Zone Entrance
void Zone_SetZoneEntrance(Point16_t pnt, uint8_t idx);
void Zone_ResetZoneUDRL(Point16_t cell);
Point16_t Zone_GetZoneEntrance(uint8_t idx);
Point16_t Zone_GetCurrentZoneEntrance(void);

//Zone Exit
void Zone_SetZoneExit(Point16_t cell, uint8_t idx);
Point16_t Zone_GetZoneExit(uint8_t idx);
Point16_t Zone_GetCurrentZoneExit(void);
Point16_t Zone_GetZoneExitWithOffset(uint8_t offset);

//Zone entrance heading
void Zone_SetZoneEntranceRobotHeading(uint16_t heading, uint8_t idx);
uint16_t Zone_GetZoneEntranceRobotHeading(uint8_t idx);
uint16_t Zone_GetCurrentZoneEntranceRobotHeading(void);

//Zone exit heading
void Zone_SetZoneExitRobotHeading(uint16_t heading, uint8_t idx);
uint16_t Zone_GetZoneExitRobotHeading(uint8_t idx);
uint16_t Zone_GetCurrentZoneExitRobotHeading(void);

//Zone wall follow type
uint8_t Zone_GetZoneWallFollowStatus(void);
void Zone_SetZoneWallFollowStatus(uint8_t val);

//Zone
Point16_t Zone_GetZone(uint8_t idx);
Point16_t Zone_GetCurrentZone(void);
Point16_t Zone_GetZoneWithOffset(uint8_t offset);
uint8_t Zone_GetZoneSize(void);
uint8_t Zone_GetCurrentZoneIdx(void);
void Zone_RemoveCurrentZone(void);
uint8_t Zone_IsOverZoneCount(void);

uint16_t Zone_GetLastZoneDirection(void);

//Boundary
void Zone_SetCurrentZoneBoundary(CellState state, uint8_t flag);
void zone_set_east_boundary(int16_t x, int16_t y, CellState state, uint8_t flag);
void zone_set_south_boundary(int16_t x, int16_t y, CellState state, uint8_t flag);
void zone_set_west_boundary(int16_t x, int16_t y, CellState state, uint8_t flag);
void zone_set_north_boundary(int16_t x, int16_t y, CellState state, uint8_t flag);
void Zone_SetBoundary( int16_t x, int16_t y, uint16_t width, uint16_t height, CellState state, uint8_t flag );

//Trapped
uint8_t Zone_GetTrappedZoneSize(void);
uint8_t Zone_GetTrappedZoneIdx( uint8_t id );
void Zone_DeleteTrappedZoneIdx( uint8_t id );
void Zone_AddTrappedZoneIdx(void);

void Zone_GetRange(int16_t *x_min, int16_t *x_max, int16_t *y_min, int16_t *y_max);
void Zone_SetZoneContinual( uint8_t val );
uint8_t Zone_GetCurrentContinualZoneIdx(void);

void Zone_ResetPathBlocks(void);

#endif
