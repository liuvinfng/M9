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

#ifndef __WallFollow_H
#define __WallFollow_H

#define Wall_High_Limit               800
#define Wall_Low_Limit                500
#define WALL_DEFAULT_DISTANCE	        800

#include "SysInitialize.h"
#include "Mymath.h"

#define Small_Wall_Distance  (int32_t)30000
#define Middle_Wall_Distance (int32_t)55000

#define Small_Wall_Time  (int32_t)20
#define Middle_Wall_Time (int32_t)40


extern volatile int32_t  g_wallfollow_move_distance;
extern volatile int32_t  g_wallfollow_distance;

extern volatile uint8_t  g_wallfollow_first_in_fag;
extern volatile uint8_t  g_wallfollow_circle_flag;
extern volatile uint32_t g_wallfollow_time;

void Wall_Follow_Mode(void);
/*wall follow accelerate*/
uint32_t WallFollow_GetWallAccelerate(void);
void WallFollow_ResetWallAccelerate(void) ;
/*wall follow time*/
void WallFollow_SetTime(uint32_t wft);
void WallFollow_ResetTime(uint32_t wft);
uint32_t WallFollow_GetTime(void);
/*wall follow step*/
int32_t WallFollow_GetLeftwheelStep(void);
int32_t WallFollow_GetRightwheelStep(void);
void WallFollow_ResetWheelStep(void);
/*wall follow distance*/
void WallFollow_SetDistance(uint32_t wd);
int32_t WallFollow_GetDistance(void);
void WallFollow_ResetDistance(void);
/*wall follow circle*/
void WallFollow_SetCircleFlag(void);
void WallFollow_ResetCircleFlag(void); 
uint8_t WallFollow_GetCircleFlag(void);
/*check back home*/
uint8_t WallFollow_CheckHome(Point16_t home_point);
/*check if follow the straight wall*/
uint8_t WallFollow_CheckStraightWall(uint16_t l_speed,uint16_t r_speed);
/*check if follow the big angle wall*/
uint8_t WallFollow_CheckBigAngleWall(uint16_t l_speed,uint16_t r_speed);
/*check if meet the tinter color wall*/
uint8_t WallFollow_CheckTinterWall(int32_t *wall_distance,int32_t wall_adc_distance_diff);
/*check if spinning*/
uint8_t WallFollow_CheckSpinning(uint16_t l_speed,uint16_t r_speed);
void WallFollow_ResetSpinningTime(void);
/*reset wall base line*/
void WallFollow_ResetRightBaseLine(void);

void WallFollow_UpdateCells(Point16_t robot_cell);
void WallFollow_ResetCells(void);
uint8_t WallFollow_CheckSpinningWithObs(void);


#endif /*----Behaviors------*/





