#ifndef __PATHPLANNING_H__
#define __PATHPLANNING_H__


#include "SysInitialize.h"
#include "Mymath.h"
#include "Map.h"

#define PATHLIST_CNT_LENGTH	  40

//typedef struct {
//	int16_t	x;
//	int16_t	y;
//	int16_t	x_target;
//	int16_t y_target;
//	uint16_t dir;
//}Position_Type;

typedef struct{
	Point16_t pos;
	uint16_t dir;
}Pos_t;

typedef enum{
	PATH_STATE_NOTARGET = 0,
	PATH_STATE_NORMAL_CLEAN,
	PATH_STATE_UNREACHABLE,
}PathState_t;



/*path calculating*/
PathState_t Path_Next_V2(uint8_t mode,Point16_t start_cell);
uint8_t Path_NormalClean(Point16_t start_cell,uint8_t short_path_state);
PathState_t Path_WallFineWay(Point16_t start_cell);
uint8_t Path_LaneCleaned(Point16_t *p_lane_pos);

/*path list*/
void PathList_AddNewPoint(PathList_t point);
uint8_t PathList_Out(PathList_t *p_out);
uint8_t PathList_GetNextPathListPoint(Point32_t *point_result);
PathList_t PathList_ReadPath(uint8_t idx);
PathList_t PathList_ReadLastPath(void);
void PathList_DeleteOne(uint8_t idx);
void PathList_Change(uint8_t id1,uint8_t id2);
void PathList_ShowAll(void);
void PathList_Clear(void);
uint8_t PathList_GetCnt(void);     
void Path_SetCurrentTargetPoint(Point32_t P);
Point32_t Path_GetCurrentTargetPoint(void);

/*target list*/
void TargetList_ShowAll(void);
uint8_t TargetList_CreateList(void);
uint8_t TargetList_ReadCnt(void);
void TargetList_AddTarget(int16_t x,int16_t y);
Point16_t TargetList_ReadTarget(uint8_t idx);
void TargetList_DeleteTarget(uint8_t idx);
void TargetList_Clear(void);
void TargetList_Rearrange(void);
void TargetList_StoreCurrentTarget(Point16_t target);
Point16_t TargetList_GetCurrentTarget(void);
Point16_t TargetList_GetGreatestYTarget(void);
uint8_t TargetList_IsBlockTargetable(int16_t x,int16_t y);

/*path robot pos*/
uint8_t Path_IsOnPosition(Point32_t pos_a,Point32_t pos_b);
uint8_t Path_RobotOnLane(int32_t lane_y,int32_t target_y);
uint8_t Path_OverLane(int32_t lane_y,int32_t target_y);
uint8_t Path_RobotNearPosCount(Point32_t target_count);
uint8_t Path_RobotCloseToTargetCount(Point32_t target_cnt);
uint8_t Path_RobotCloseToTargetCell(Point16_t target_cell,uint8_t distance);
uint8_t Path_RobotLeaveToTargetCell(Point16_t target_cell,uint8_t distance);
uint16_t Path_GetRobotDirection(void);
uint16_t Path_GetRobotHeading4(uint16_t angle);
uint16_t Path_GetRobotHeading8(void);
WallDir_t Path_GetNormalWallDir(Point16_t target_cell);
WallDir_t Path_GetShortWallDir(Point16_t target_cell,Point16_t robot_cell);
void Path_UpdateRobotPos(void);
Pos_t Path_GetPreviousRobotPos(void);
uint8_t Path_RobotNotMoving(void);
uint8_t Path_CheckRoundable(int16_t start_x,int16_t start_y,int16_t target_y,uint16_t heading);

/*back to start point*/
void Path_BlockAllTargetPoints(void);
void Path_SetHomeCellEmpty(void);

void Path_SetNoWayHome(uint8_t pathway);
uint8_t Path_IsNoWayHome(void);

DirectionCardinal_t Path_GetCellHeading8(Point16_t start_cell,Point16_t dest_cell);

#endif
