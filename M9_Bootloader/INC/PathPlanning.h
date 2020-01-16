#ifndef __PATHPLANNING_H__
#define __PATHPLANNING_H__

#include "config.h"

#include "Map.h"
#include "MyMath.h"
#include "Debug.h"

typedef struct {
	int16_t	x;
	int16_t	y;
	int16_t	x_target;
	int16_t y_target;
	uint16_t dir;
} PositionType;

typedef struct {
	int16_t	x;
	int16_t	y;
	int8_t	x_pos;
	int8_t	y_pos;
	uint8_t	state;
	uint8_t	try_cnt;
} TargetType;

typedef enum {
  NORTH_ZZ = 0,
  WEST_ZZ = 1,
  SOUTH_ZZ = 2,
	EAST_ZZ = 3,
} Direction_NSWE_ZZ;

void PathPlanning_Initialize(int32_t *x, int32_t *y);
int8_t path_next(int32_t *x, int32_t *y);
uint8_t path_home(int32_t *x, int32_t *y);

void path_set_current_pos(void);
void path_set_max_try_cnt(uint8_t val);
uint8_t get_robot_direction_NSWE(uint16_t angle);
uint16_t path_get_robot_direction(void);

void path_get_range(int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max);

void path_reset_last_position(void);

uint8_t is_block_accessible(int16_t x, int16_t y);

uint16_t path_targets_get_count(void);
void path_targets_clear_list(void);
uint8_t path_targets_get_last(int16_t *x, int16_t *y);

uint8_t path_targets_try_add_one(int16_t x, int16_t y, uint8_t accessible);
void path_targets_add_one(int16_t x, int16_t y, uint8_t accessible);
void path_targets_update(void);

void path_update_cells(void);
int8_t path_escape_trapped(void);

 /* Set the around 9 cell as state */
void path_set_9cell(int16_t cell_x, int16_t cell_y, CellState state);

 /* Set the around 9 cell as state */
void path_set_25cell(int16_t cell_x, int16_t cell_y, CellState state);

void path_escape_set_trapped_cell( Point16_t *cell, uint8_t size );
Point16_t *path_escape_get_trapped_cell(void);

int16_t path_get_home_x(void);
int16_t path_get_home_y(void);

uint8_t is_a_block(int16_t x, int16_t y);


#endif


