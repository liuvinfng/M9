/**
  ******************************************************************************
  * @file    stm32f10x_exti.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the EXTI firmware
  *          library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CORMOVE_H
#define __CORMOVE_H

#include "SysInitialize.h"
#include "Mymath.h"
#include "wheel.h"
#include "obscliff.h"

//#define MS_Clear 		0x00
//#define MS_OBS   		0x01
//#define MS_Bumper		0x02
//#define MS_Cliff 		0x04
//#define MS_User 		0x08
//#define MS_Home			0x10
//#define MS_Clean 		0x20
//#define MS_Spot 		0x40
//#define MS_Error 		0x80

//#define COR_BACK_20MM		  (136)
//#define COR_BACK_100MM		(680)
//#define COR_BACK_500MM		(3400)

typedef enum {
	MT_NONE = 0,
	MT_BATTERY,
	MT_REMOTE_HOME,
	MT_REMOTE_CLEAN,
	MT_REMOTE_SPOT,
	MT_CLIFF,
	MT_KEY_CLEAN,
	MT_BATTERY_HOME,
	MT_OBSTACLE,
	MT_ARRIVED,
	MT_HALFARRIVED,
	MT_FINDCELL,
	MT_BOUNDARY,
}MapTouring_t;


typedef enum{
	CM_STATE_TOURING=0,
	CM_STATE_GYROINIT,
	CM_STATE_DRYING,
}CMState_t;


typedef enum{
	CM_NORMAL = 0,
	CM_SPECIAL,
}CM_t;


void CM_MapTouring(void);

MapTouring_t CM_MoveToPoint(Point32_t Target, uint8_t vm_ignore);

uint8_t CM_MoveForward(void);

void CM_SetCMType(CM_t c);
CM_t CM_GetCMType(void);

int8_t CM_MoveToCell( int16_t x, int16_t y, uint8_t mode, uint8_t length, uint8_t step );

void CM_CorBack(uint16_t dist);

void CM_SetGoHome(uint8_t remote);
void CM_TouringCancel(void);
void CM_SetGyroOffset(int16_t offset);

void CM_SetHome(int32_t x, int32_t y);
void CM_SetStationHome(void);

void CM_ResetBoundaryBlocks(void);

void CM_AddTargets(Point16_t zone);
uint8_t CM_IsLowBattery(void);

uint8_t CM_CheckLoopBack(Point16_t target);

void CM_Matrix_Rotate(int32_t x_in, int32_t y_in, int32_t *x_out, int32_t *y_out, double theta);

uint8_t CM_IsSingleRoomMode(void);

void CM_SetSingleRoomMode( uint8_t val );

uint8_t CM_IsFromStation(void);

MapTouring_t CM_handleExtEvent(void);

void CM_UpdatePosition(uint16_t heading_0, int16_t heading_1, int16_t left, int16_t right);

void CM_HeadToTarget(Point32_t target);
void ActList_Add_HeadingToTarget(Point32_t target);
void CM_HeadToCourse(uint8_t speed, int16_t angle);

MapTouring_t CM_MoveToPosition(Point32_t target_cnt,WheelDir_t move_dir,uint8_t move_type);
MapTouring_t CM_MoveToMap(Point16_t target_cell,uint8_t move_type);
MapTouring_t CM_WallToMap(Point16_t target_cell, WallDir_t wall_dir ,WallTravel_t travel_state);
MapTouring_t CM_WallFollowByCnt(Point32_t start_cnt, WallDir_t wall_dir, WallTravel_t travel_state);
uint8_t CM_IsCrossLane(WallDir_t wall_dir);
void CM_UpdateMapWallBlock(Point16_t robot_cell, WallDir_t dir);
void CM_UpdateStrBlock(Point16_t robot_cell, WallDir_t dir);
void CM_UpdateSpotWallBlock(WallDir_t dir);
void CM_UpdateMapBumper(Action_t action, uint8_t bumper);
void CM_UpdateMapObs(ObsTrig_t obs);

uint8_t CM_IsBackCrossing(WallDir_t wall_dir,Point32_t start_count);

uint8_t CM_GetBackToStartPointFlag(void);
void CM_SetBackToStartPointFlag(uint8_t back_flag);
WallDir_t CM_SetTrapWallDir(WallDir_t temp_dir);

void CM_StoreCurrentTargetCnt(Point32_t target);
Point32_t CM_GetCurrentTargetCnt(void);

void CM_CellOffset(uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y);
void CM_SetBlockedByOffset(int16_t offset_lat, int16_t offset_long);
void MY_SetBlockedByOffset(Point16_t point, int16_t offset_lat, int16_t offset_long);

void CM_SetWallTrapFlag(uint8_t flag);
uint8_t CM_GetWallTrapFlag(void);

void CM_AbnormalHandler(uint8_t status);

void CM_Drying(void);


void CM_ResetMobilityEventCnt(void);
uint8_t CM_GetMobilityEventCnt(void);
void CM_MobilityEventCntIncrease(void);
uint8_t CM_GetMobilityIgnoreFlag(void);
void CM_ResetMobilityIgnoreFlag(void);
void CM_WaitForGyroCal(void);
uint8_t CM_WallFollowBoundaryEvent(int32_t *wall_proportion,int32_t *wall_distance,Point16_t robot_cell);
uint8_t CM_ReachSpotBoundary(void);

uint8_t CM_CheckMeetBoundary(Point16_t robot_cell);

void CM_WallOutTrapCntIncrease(void);
void CM_ResetWallOutTrapCnt(void);
uint8_t CM_IsWallOutTrapOverLimit(void);

void CM_SetNearTargetFlag(uint8_t flag);
uint8_t CM_GetNearTargetFlag(void);

uint8_t CM_IsLoop_Fault(void);
uint8_t CM_Move_Back(uint8_t speed,uint16_t dis);
uint8_t CM_Move_Turn(uint8_t speed,uint16_t dis);

void Heading_SetTargetAngle(int16_t angle);
int16_t Heading_GetTargetAngle(void);


void CM_SetBackBlock(Point16_t robot_cell);
	
extern volatile uint8_t g_next_t;


#endif



