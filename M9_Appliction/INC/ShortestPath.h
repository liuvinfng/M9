#ifndef __SHORTESTPATH_H__
#define __SHORTESTPATH_H__

#include "SysInitialize.h"
#include "Mymath.h"
#include "Map.h"

#define LINE_SPACE_ENOUGH		10
#define LINE_SPACE_AX	1
#define LINE_SPACE_BX	2
#define LINE_SPACE_AY	3
#define LINE_SPACE_BY	4

typedef struct{
	int16_t  y;
	int16_t  x_start;
	uint8_t  x_offset;
	uint8_t  x_first;  
	uint16_t level;
	uint8_t select;	
}Line_t;

/*ShortestPath*/
uint8_t ShortestPath_GeneratePath(Point16_t robot_cell,uint8_t first_line_width);
void ShortestPath_CreatePathPoint(Point16_t robot_cell);
void ShortestPath_CreatePathPointOld(Point16_t robot_cell);
/*PathLine*/
uint8_t PathLine_FindAllLines(void);
Line_t PathLine_FindOneLine(Point16_t start_cell, uint8_t line_width);
uint8_t PathLine_FindNextLines(Line_t current_line);
Line_t PathLine_GetNearLine(Line_t current_line);
void PathLine_SortLines(void);
void PathLine_Reset(void);
void PathLine_AddLine(Line_t line);
uint16_t PathLine_GetLineCnt(void);
Line_t PathLine_ReadLine(uint16_t idx);
void PathLine_EditLine(Line_t L,uint16_t idx);
void PathLine_DeleteLine(uint16_t idx);
void PathLine_DeleteSameLevel(Line_t * line);
uint16_t PathLine_GetLineLevel(void);
void PathLine_IncreaseLevel(void);
void PathLine_ResetLineLevel(void);
uint8_t PathLine_LineExisted(Line_t line);
uint8_t PathLine_SpaceCheck(LineABCInt_t line,Point16_t point_a,Point16_t point_b);
uint8_t PathLine_IsPosOnLine(Point16_t cell,Line_t line);
void PathLine_ShowAllLines(void);
uint32_t PathLine_GetAcreage(void);
uint8_t PathLine_CheckTwoLinesCross(Line_t Line_A,Line_t Line_B);
/*PathPont*/
void PathPoint_Optimize(void);
uint8_t PathPoint_PathYReliableWeight(int16_t xs,int16_t xe,int16_t y);
uint8_t PathPoint_PathXReliableWeight(int16_t ys,int16_t ye,int16_t x);
void PathPoint_AddPathPointToPathList(uint16_t start_idx);
void PathPoints_Rearrange(void);
void PathPoint_AddOnePoint(Point16_t point);
uint16_t PathPoint_GetPointCnt(void);
void PathPoint_EditPoint(Point16_t point,uint16_t idx);
Point16_t PathPoint_ReadPoint(int16_t idx);
Point16_t PathPoint_ReadLastPoint(void);
void PathPoint_ClearAllPoints(void);
void PathPoint_RemoveOnePoint(uint16_t idx);
void PathPoint_RemoveLastPoint(void);
void PathPoint_InsertOnePoint(Point16_t point,uint8_t idx);
void PathPoint_ShowAllPathPoints(void);
void PointList_ShowAllPathPoints(void);
uint8_t LineList_GetNextLowPriorityPoint(int16_t *line_idx,Point16_t *next_cell,uint8_t *x_level,DirectionCardinal_t *next_dir);

#endif
