#ifndef __SHORTESTPATH_H__
#define __SHORTESTPATH_H__

#include "PathPlanning.h"
#include "config.h"
#include "Map.h"
#include "Zone.h"
#include "CorMove.h"

typedef struct{
	uint8_t	x;
	uint8_t	y;
	uint8_t	x_off;
	uint8_t level;
} LineType;

#define MAX_LEVEL (uint8_t)127
#define MIN_LEVEL (uint8_t)1

#define SAME_LINE_SIZE 100
#define NEXT_POS_SIZE SAME_LINE_SIZE/4

void path_position_init(uint8_t data);
void path_position_update(void);

uint16_t path_line_get_count(void);
uint16_t Get_Line_Id_POS(Point16_t pos);
void Set_Path_Line_Count(uint16_t id);
void Init_Path_Line(void);
void Set_Line_Value(uint16_t count,uint8_t flag,uint8_t level,int16_t y,int16_t xmin,int16_t xmax);
uint8_t Path_To_Move(Point16_t pos, Point16_t target, Point16_t* next_pos,uint16_t last_dir);

int8_t path_find_shortest_path(int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound, uint16_t last_dir);
int8_t path_move_to_unclean_area(Point16_t pos, int16_t x, int16_t y, int16_t *x_next, int16_t *y_next, uint16_t last_dir);

int8_t path_move_to_cell( int16_t x, int16_t y, uint8_t mode );

#endif
