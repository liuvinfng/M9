/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    11-July-2018
  * @brief   System Initialize
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

#ifndef __WallFollowMulti_H
#define __WallFollowMulti_H

#include "SysInitialize.h"
#include "config.h"
#include "MyMath.h"
#include "Debug.h"

typedef enum {
	Map_Wall_Follow_None = 0,
	Map_Wall_Follow_Escape_Trapped,
	Map_Wall_Follow_Escape_Trapped_ZZ,
	Map_Wall_Follow_Left_Zone,
	Map_Wall_Follow_Left_Target,
	Map_Wall_Follow_To_Zone_Exit,
	Map_Wall_Follow_To_Zone_Entrance,
} MapWallFollowType;

typedef enum {
	Map_Find_Wall_Not_Found = 0,
	Map_Find_Wall_Timeout,
	Map_Find_Wall_Over_Distance,
	Map_Find_Wall_Found,
	Map_Find_Wall_No_Need_To_Find,
} MapFindWallType;

typedef enum {
	Map_Escape_Trapped_Trapped,
	Map_Escape_Trapped_Escaped,
	Map_Escape_Trapped_Timeout,
} MapEscapeTrappedType;

typedef struct {
	int32_t front_obs_val;
	int32_t left_bumper_val;
	int32_t right_bumper_val;
} MapWallFollowSetting;

int8_t WFM_update(void);
void WFM_move_back(uint16_t dist);
void WFM_wall_move_back(uint16_t distance);
void WFM_Right_move_back(uint8_t speed,uint16_t dist,uint16_t angle);
void WFM_Left_move_back(uint8_t speed,uint16_t dist,uint16_t angle);

void Map_Wall_Follow_Initialize(void);
uint8_t WFM_IsWallFollowSamePointAndHeading(Point16_t crtCell, uint16_t crtHeading);
uint8_t Map_Wall_Follow_GetWallFollowDirection(void);
uint8_t Map_Wall_Follow(MapWallFollowType follow_type);
void WFM_SetWallFollowDistance(int32_t Dis);
void Map_Set_WallFollow_Target(Point32_t tar);

#ifdef STOP_WALL_FOLLOW_M2
Point16_t *WFM_GetWallFollowCell(void);
uint8_t WFM_GetWallFollowCellCount();
#endif

uint8_t WFM_GetZoneReturnType(void);

uint8_t WFM_IsIsolatedWallFollow(void);

uint8_t WFM_IsWallFollowTooFar(void);
uint8_t WFM_IsMiniRoom(void);
void WFM_ResetIsMiniRoom(void);

void WFM_SetDirectWallFollow(uint8_t val);
MapFindWallType WFM_GetWallFoundStatus(void);
void WFM_SetWallFoundStatus(MapFindWallType status);
void WFM_SetMiniRoomChecking(uint8_t status);

uint8_t WFM_TiltedDetect(void);

#endif/*----Behaviors------*/


