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
#include "MyMath.h"
#include "Debug.h"

#define COR_BACK_20MM		  (104)
#define COR_BACK_50MM		  (270)
#define COR_BACK_100MM		(520)
#define COR_BACK_400MM		(2080)
#define COR_BACK_500MM		(2600)

typedef enum {
	MT_None = 0,
	MT_Battery,
	MT_Remote_Home,
	MT_Remote_Clean,
	MT_Remote_Spot,
	MT_Cliff,
	MT_Key_Clean,
	MT_Battery_Home,
} MapTouringType;

typedef struct {
	Point16_t	pos;
} VWType;

int32_t CM_ABS(int32_t A, int32_t B);
void CM_HeadToCourse(uint8_t Speed,int16_t Angle);
MapTouringType CM_MoveToPoint(Point32_t Target, uint8_t vw_ignore);
uint8_t CM_MoveForward(void);
uint8_t CM_Touring(void);
uint8_t CM_MoveToTarget_Interrupt(Point32_t Target);
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
void CM_Matrix_Rotate_Cell(int16_t x_in, int16_t y_in, int16_t *x_out, int16_t *y_out, double theta);

void CM_count_normalize(uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y);
void CM_count_normalize_ByXYCount(int32_t XCount, int32_t YCount, uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y);
void CM_update_position(uint16_t heading_0, int16_t heading_1, int32_t left, int32_t right);

uint8_t CM_IsSingleRoomMode(void);

void CM_SetSingleRoomMode( uint8_t val );

uint8_t CM_IsFromStation(void);

MapTouringType CM_handleExtEvent(void);

void Set_Station_Position(uint8_t data);
uint8_t Get_Station_Position(void);

void CheckGyroCalibrationTime(uint16_t time_limited);

#endif


