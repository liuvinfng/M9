/**
******************************************************************************
* @file        Shortest Path
* @author      Wfliu
* @version     Ver 20160118
* @date        18-Jan-2016
* @brief       Function to find the shorest path to target
******************************************************************************
* <h2><center>&copy; COPYRIGHT 2016 ILife CO.LTD</center></h2>
******************************************************************************
*/

/*
 * Path Planning for robot movement is a ZigZag way. When cleaning process starts,
 * robot will try to clean its left hand side first, a new lane must be cleaned
 * in both ends before getting the new cleaning target. Targets will be added to
 * the target list as soon as the robot is moving.
 *
 * After the cleaning process is done, robot will back to its starting point and
 * finishes the cleaning.
 */

#include <string.h>
#include <limits.h>
#include <stdlib.h>

#include "CorMove.h"
#include "Gyro.h"
#include "MyMath.h"
#include "Map.h"
#include "PathPlanning.h"
#include "Sensors.h"
#include "ShortestPath.h"
#include "WallFollowMulti.h"
#include "Zone.h"

extern uint8_t	should_follow_wall;

uint8_t	first_start = 0;

uint8_t try_entrance;

uint8_t	weight_enabled = 0;

/* Threshold about how many unclean block it will be discarded. */
uint8_t	weight_cnt_threshold = 5;

/* Enable direct go when there is no obstcal in between current pos. & dest. */
uint8_t direct_go = 0;

uint8_t max_try_cnt = 6;
uint8_t home_try_cnt = 0;
int16_t home_x, home_y;

uint8_t target_swapped = 0;

uint8_t	clear_block = 0;

PositionType positions[5];

int16_t	last_x_pos;
int16_t	last_y_pos;
uint16_t last_dir;

uint16_t target_list_cnt = 0;

Point16_t trappedCell[ESCAPE_TRAPPED_REF_CELL_SIZE];
uint8_t trappedCellSize = 1;



TargetType targetList[TARGET_TOTAL_COUNT];



extern int16_t xMin, xMax, yMin, yMax;

extern uint16_t map_size;
void path_targets_add_one(int16_t x, int16_t y, uint8_t accessible);

void path_set_current_pos(void);
void path_get_range(int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max);
uint16_t path_get_robot_direction(void);

uint8_t preset_action_count = 0;

#define Robot_GetDirection()	0

#ifdef VIRTUAL_WALL
extern uint8_t vw_pos_cnt;
extern VWType vw_pos[50];
#endif

extern volatile Point16_t ShortestPath_Last_Cell;
extern volatile uint8_t WallFollow_Point_Exit;

extern uint8_t PP_RoundingMovingFlag;

/*
 * Initialization function for path planning, it sets the starting
 * point as the home of the robot.
 *
 * @param *x	Pointer to robot home X coordinate
 * @param *y	Pointer to robot home Y coordinate
 *
 * @return
 */
void PathPlanning_Initialize(int32_t *x, int32_t *y) {
	int16_t i;

	/* Save the starting point as home. */
	home_x = countToCell(*x);
	home_y = countToCell(*y);

	/* Initialize the default settings. */
	preset_action_count = 0;

	weight_enabled = 1;

	for ( i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i ) {
		trappedCell[i].X = home_x;
		trappedCell[i].Y = home_y;
	}
	trappedCellSize = 1;



	weight_cnt_threshold = 3;


	direct_go = 0;

	max_try_cnt = 1;
	home_try_cnt = 0;

	WallFollow_Point_Exit = 0;
	ShortestPath_Last_Cell.X = 250;
	ShortestPath_Last_Cell.Y = 250;
	positions[0].x = positions[0].y = 0;
	last_dir = 0;

	try_entrance = 0;
	first_start = last_x_pos = last_y_pos = 0;
	target_list_cnt = 0;	
	/* Reset the poisition list. */
	for (i = 0; i < 3; i++) {
		positions[i].x = positions[i].y = i + 1;
	}

	/* Initialize the shortest path. */
	path_position_init(direct_go);

#ifndef ZONE_WALLFOLLOW

	/* Set the back as blocked, since robot will align the start angle with the wall. */
	Map_SetCell(MAP, cellToCount(-3), cellToCount(0), BLOCKED_BUMPER);
#endif

	if (CM_IsFromStation() == 0 && CM_IsSingleRoomMode() == 1 ) {
		/* Set the back as blocked, since robot will align the start angle with the wall. */
		Map_SetCell(MAP, cellToCount(-3), cellToCount(0), BLOCKED_BUMPER);
	}

	Map_SetCell(MAP, *x, *y, CLEANED);
	/* Set the starting point as cleaned. */
	Map_SetCell(MAP, cellToCount(-1), cellToCount(-1), CLEANED);
	Map_SetCell(MAP, cellToCount(-1), cellToCount(0), CLEANED);
	Map_SetCell(MAP, cellToCount(-1), cellToCount(1), CLEANED);
	Map_SetCell(MAP, cellToCount(0), cellToCount(-1), CLEANED);
	Map_SetCell(MAP, cellToCount(0), cellToCount(0), CLEANED);
	Map_SetCell(MAP, cellToCount(0), cellToCount(1), CLEANED);
	Map_SetCell(MAP, cellToCount(1), cellToCount(-1), CLEANED);
	Map_SetCell(MAP, cellToCount(1), cellToCount(0), CLEANED);
	Map_SetCell(MAP, cellToCount(1), cellToCount(1), CLEANED);

	/* Add the front, left and right as the target point. */
	path_targets_add_one(2, 0, 0);
	path_targets_add_one(0, 2, 0);
	path_targets_add_one(0, -2, 0);
}

/*
 * Function to enable try the entrance first or not, an entrance only fits the robot
 * to get into, if it is set, the robot will try it first as soon as an entrance is
 * found. This is to increase the possibility to move to the entrance, otherwise, due
 * to the encoder and Gyro drift, it is hard to go into the entrance again after
 * cleaning some area.
 *
 * @param val	Enable or disable the try_entrance flag
 *
 * @return
 */
void path_set_try_entrance(uint8_t val)
{
	try_entrance = val;
}

/* Function to enable or disable the weight check for a target, if it is enable,
 * a target will check against its weight with the threshold value, it it is less
 * than the threshold value, discard the target.
 *
 * @param enable	Enable or disable the weight check for a target
 * @param count		Lower limit threshold that a target must be cleaned
 *
 * @return
 */
void path_set_weight(uint8_t enable, uint8_t count)
{
	weight_enabled = enable;
	weight_cnt_threshold = count;
}

/* Function to enable or disable direction go when moving to the target.
 * When enabled, robot will move to target when only if there is no obstcal
 * in between the robot and target.
 *
 * @param val	Enable or disable direction_go flag.
 *
 * @return
 */
void path_set_direct_go(uint8_t val)
{
	direct_go = val;
}

/*
 * Update current robot positions.
 *
 * @param
 *
 * @return
 */
void path_set_current_pos()
{
	positions[4] = positions[3];
	positions[3] = positions[2];
	positions[2] = positions[1];
	positions[1] = positions[0];

	positions[0].x = Map_GetXPos();
	positions[0].y = Map_GetYPos();
	positions[0].dir = path_get_robot_direction();
}

/*
 * Set the maximum try count for a target.
 *
 * @param val	Maximum try count for a target
 *
 * @return
 */
void path_set_max_try_cnt(uint8_t val)
{
	if (val > 0)
		max_try_cnt = val;
}

uint8_t get_robot_direction_NSWE(uint16_t angle)
{
	if((angle < 450) || (angle >= 3150))
	{
		return NORTH_ZZ;
	}
	else if((angle < 1350) && (angle >= 450))
	{
		return WEST_ZZ;
	}
	else if((angle < 2250) && (angle >= 1350))
	{
		return SOUTH_ZZ;
	}
	else //if((angle < 3150) && (angle >= 2250))
	{
		return EAST_ZZ;
	}
}
/*
 * Function to get the current robot direction.
 * The direction is obtained by using the Gyro value, only 0(360), 90, 180 & 270 will
 * return by using the Gyro value. When fails to use the Gyro value, it will return
 * the vertical direction only.
 *
 * @param
 *
 * @return	Current robot direction
 */
uint16_t path_get_robot_direction()
{
	uint16_t	dir;

	dir = (uint16_t) round(((double)Gyro_GetAngle(0)) / 100);
	switch(dir) {
		case 0:
		case 36:
			dir = NORTH;
			break;
		case 9:
			dir = EAST;
			break;
		case 18:
			dir = SOUTH;
			break;
		case 27:
			dir = WEST;
			break;
		default:
			/*
			 * Failed to use the Gyro value, use the robot displacement to find the direction.
			 * Only handle NORTH & SOUTH direction.
			 */
			if (positions[0].x != positions[1].x && positions[0].y == positions[1].y) {
				if (positions[0].x > positions[1].x) {
					dir = NORTH;
				} else if (positions[0].x < positions[1].x) {
					dir = SOUTH;
				}
			}
			break;
	}
	return dir;
}

/*
 * Function to find the X/Y range of the Map, if the range is to small,
 * use the offset of those value to 3.
 *
 * @param *x_range_min	Pointer for minimum X value of the Map
 * @param *x_range_max	Pointer for maximum X value of the Map
 * @param *y_range_min	Pointer for minimum Y value of the Map
 * @param *y_range_max	Pointer for maximum Y value of the Map
 *
 * @return
 */
void path_get_range(int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max)
{
	*x_range_min = xMin - (abs(xMin - xMax) <= 3 ? 3 : 1);
	*x_range_max = xMax + (abs(xMin - xMax) <= 3 ? 3 : 1);
	*y_range_min = yMin - (abs(yMin - yMax) <= 3? 3 : 1);
	*y_range_max = yMax + (abs(yMin - yMax) <= 3 ? 3 : 1);

	USPRINTF("Get Range:\tx: %d - %d\ty: %d - %d\tx range: %d - %d\ty range: %d - %d\n",
		xMin, xMax, yMin, yMax, *x_range_min, *x_range_max, *y_range_min, *y_range_max);
}

/*
 * Reset the last X/Y & robot direction.
 *
 * @param
 * @param
 *
 * @return
 */
void path_reset_last_position(void)
{
	last_x_pos = Map_GetXPos();
	last_y_pos = Map_GetYPos();
	last_dir = path_get_robot_direction();
}

/*
 * Check whether a given point is an blocked or not.
 *
 * @param x	X coordinate of the give point.
 * @param y	Y coordinate of the give point.
 *
 * @return	0 if the given point is not blocked
 * 		1 if the given point is blocked
 */
uint8_t is_a_block(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	CellState cs;

	cs = Map_GetCell(MAP, x, y);
	if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY)
	//if (cs >= BLOCKED && cs <= BLOCKED_CLIFF)
		retval = 1;

	return retval;
}

/*
 * Check a given point is blocked by bumper and/or cliff or not.
 *
 * @param x	X coordinate of the given point
 * @param y	Y coordinate of the given point
 *
 * @return	0 if it is not blocked by bumper and/or cliff
 *		1 if it is blocked by bumper and/or cliff
 */
uint8_t is_blocked_by_bumper(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i, j;
	CellState cs;

	/* Check the point by using the robot size. */
	for (i = ROBOT_RIGHT_OFFSET; retval == 0 && i <= ROBOT_LEFT_OFFSET; i++) {
		for (j = ROBOT_RIGHT_OFFSET; retval == 0 && j <= ROBOT_LEFT_OFFSET; j++) {
			cs = Map_GetCell(MAP, x + i, y + j);
			//if ((cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) && cs != BLOCKED_OBS) {
			if ((cs >= BLOCKED && cs <= BLOCKED_CLIFF) && cs != BLOCKED_OBS) {
				retval = 1;
			}
		}
	}

	return retval;
}

/*--------------------------- Edit By ZZ ---------------------*/
uint8_t is_near_boundary(int16_t x,int16_t y)
{
	uint8_t retval = 0;
	int16_t i;

	for (i = -2; retval == 0 && i <= 2; i++) {
		if ((Map_GetCell(MAP,x + i, y + 2) == BLOCKED_BOUNDARY) || (Map_GetCell(MAP,x + i, y - 2) == BLOCKED_BOUNDARY)) {
			retval = 1;
		}
	}
	for (i = -1; retval == 0 && i <= 1; i++) {
		if ((Map_GetCell(MAP,x + 2, y + i) == BLOCKED_BOUNDARY) || (Map_GetCell(MAP,x - 2, y + i) == BLOCKED_BOUNDARY)) {
			retval = 1;
		}
	}

	return retval;
}
/***************************************************************/

/*
 * Check a block is accessible or not, a block is defined as have the same size of robot.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not accessible
 *		1 if the block is accessible
 */
uint8_t is_block_accessible(int16_t x, int16_t y)
{
	uint8_t retval = 1;
	int16_t i, j;

	for (i = ROBOT_RIGHT_OFFSET; retval == 1 && i <= ROBOT_LEFT_OFFSET; i++) {
		for (j = ROBOT_RIGHT_OFFSET; retval == 1 && j <= ROBOT_LEFT_OFFSET; j++) {
			if (is_a_block(x + i, y + j) == 1) {
				retval = 0;
			}
		}
	}

	return retval;
}

/*
 * Check a block is cleanable or not, a block is defined as have the same size of brush.
 *
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not cleanable
 *		1 if the block is cleanable
 */
int8_t is_block_cleanable(int16_t x, int16_t y)
{
	int8_t	retval = 1;
	int16_t	i, j;

	for (i = ROBOT_BRUSH_LEFT_OFFSET; retval == 1 && i <= ROBOT_BRUSH_RIGHT_OFFSET; i++) {
		for (j = ROBOT_BRUSH_LEFT_OFFSET; retval == 1 && j <= ROBOT_BRUSH_RIGHT_OFFSET; j++) {
			if (is_a_block(x + i, y + j) == 1) {
				retval = 0;
			}
		}
	}
	return retval;
}

/*
 * Check a block is cleaned or not, a block is defined as have the same size of brush.
 *
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not cleaned
 *		1 if the block is cleaned
 */
int8_t is_block_cleaned(int16_t x, int16_t y)
{
	int8_t	retval = 1;
	int16_t	i, j;

	for (i = ROBOT_BRUSH_LEFT_OFFSET; retval == 1 && i <= ROBOT_BRUSH_RIGHT_OFFSET; i++) {
		for (j = ROBOT_BRUSH_LEFT_OFFSET; retval == 1 && j <= ROBOT_BRUSH_RIGHT_OFFSET; j++) {
			if (Map_GetCell(MAP, x + i, y + j) != CLEANED) {
				retval = 0;
			}
		}
	}
	return retval;
}

/*
 * Check a block is uncleaned or not, a block is defined as have the same size of brush.
 * Since the brush occupies 3 cells, if there is any one of those 3 cells unclean, then the
 * block is treated as unclean.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is cleaned
 *		1 if the block is uncleaned
 */
uint8_t is_brush_block_unclean(int16_t x, int16_t y)
{
	uint8_t retval = 0, count = 0;

	if (Map_GetCell(MAP, x, y) == UNCLEAN)
		count++;

	if (count == 1)
		retval = 1;

	return retval;
}

/*
 * Check a block is on the boundary or not, a block is defined as have the same size of robot.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not on the boundary
 *		1 if the block is on the boundary
 */
uint8_t is_block_boundary(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i;

	for (i = (y + ROBOT_RIGHT_OFFSET); retval == 0 && i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (Map_GetCell(MAP, x, i) == BLOCKED_BOUNDARY) {
			retval = 1;
		}
	}

	return retval;
}

/*
 * Check a block is accessible by the robot or not.
 * A block is defined as have the same size of robot.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not blocked by bumper, obs or cliff
 *		1 if the block is blocked
 */
uint8_t is_block_blocked(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i;

	for (i = (y + ROBOT_RIGHT_OFFSET); retval == 0 && i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (is_a_block(x, i) == 1) {
			retval = 1;
		}
	}

	return retval;
}

/*
 * By give a point, check whether it is a horizontally entrance or not.
 * If the flag "try_entrance" is set, let the robot try to enter such
 * entrance first. A veritcal entrance is defined as
 *
 * 	x
 * 	0
 * 	0
 * 	0
 * 	0
 * 	0
 * 	x
 *
 * where x is any kind of obstcal, and 0 is the uncleaned area.
 *
 * @param x	X coordinate of the entrance
 * @param y	Y coordinate of the entrance
 *
 * @return	0 if it is not a horizontal entrance
 *		1 if it is a horizontal entrance
 */
uint8_t is_narrow_block_horizontal(int16_t x, int16_t y)
{
	uint8_t	left, right, middle, retval;
	int16_t	i, j, min, max;

	left = right = middle = retval = 0;
	min = SHRT_MAX;
	max = SHRT_MIN;

	for (i = -3; i <= 3;  i++) {
		if (is_a_block(x + i, y - 3) == 1) {
			left = 1;
			if (min > x + i)
				min = x + i;
			if (max < x + i)
				max = x + i;
		}

		if (is_a_block(x + i, y + 3) == 1) {
			right = 1;

			if (min > x + i)
				min = x + i;
			if (max < x + i)
				max = x + i;
		}

		for (j = ROBOT_RIGHT_OFFSET; (middle == 0 && j <= ROBOT_LEFT_OFFSET);  j++) {
			if (is_a_block(x + i, y + j) == 1)
				middle = 1;
		}
	}
	if ((left & right) && (middle == 0)) {
		if (x > min && x > max) {
			if (min <= x - 1 && max >= x - 1)
				retval = 1;
		} else if (x <= max && x >= min) {
			retval = 1;
		} else if (min > x && max > x) {
			if (min <= x + 1 && max >= x + 1)
				retval = 1;
		}
	}
	return retval;
}

/*
 * By give a point, check whether it is a vertical entrance or not.
 * If the flag "try_entrance" is set, let the robot try to enter such
 * entrance first. A veritcal entrance is defined as
 *
 * 	x
 * 	0
 * 	0
 * 	0
 * 	0
 * 	0
 * 	x
 *
 * where x is any kind of obstcal, and 0 is the uncleaned area.
 *
 * @param x	X coordinate of the entrance
 * @param y	Y coordinate of the entrance
 *
 * @return	0 if it is not a vertical entrance
 *		1 if it is a vertical entrance
 */
uint8_t is_narrow_block_vertical(int16_t x, int16_t y)
{
	uint8_t	up, down, middle;
	int16_t	i, j, min, max;

	up = down = middle = 0;
	min = SHRT_MAX;
	max = SHRT_MIN;

	for (i = -3; i <= 3;  i++) {
		if (is_a_block(x - 3, y + i) == 1) {
			up = 1;
			if (min > x + i)
				min = x + i;
			if (max < x + i)
				max = x + i;
		}

		if (is_a_block(x + 3, y + i) == 1) {
			down = 1;

			if (min > x + i)
				min = x + i;
			if (max < x + i)
				max = x + i;
		}

		for (j = ROBOT_RIGHT_OFFSET; (middle == 0 && j <= ROBOT_LEFT_OFFSET);  j++) {
			if (is_a_block(x + j, y + i) == 1)
				middle = 1;
		}
	}
	return ((up & down) && (middle == 0) && (min <= x + 2) && (max >= x));
}

/*
 * Get the number of targets.
 *
 * @parm
 *
 * @return	Number of targets in the target list.
 */
uint16_t path_targets_get_count()
{
	return target_list_cnt;
}

/*
 * Function for the robot to clear target list
 */
void path_targets_clear_list() {
	uint16_t i = 0;
	target_list_cnt = 0;
	for ( i = 0; i < TARGET_TOTAL_COUNT; ++i ) {
		targetList[i].x = 0;
		targetList[i].y = 0;
		targetList[i].x_pos = 0;
		targetList[i].y_pos = 0;
		targetList[i].state = 0;
		targetList[i].try_cnt = 0;
	}
}

/*
 * Display all the target point for debugging.
 *
 * @param
 *
 * @return
 */
void path_targets_dump()
{
	int16_t i;

	for (i = 0; i < target_list_cnt; i++) {
		if (targetList[i].state > 0) {
			USPRINTF("%d: (%d, %d) (%d, %d)\t%d\t%d\n", i,
				targetList[i].x, targetList[i].y, targetList[i].x + targetList[i].x_pos, targetList[i].y + targetList[i].y_pos,  targetList[i].state, targetList[i].try_cnt);
		}
	}
	USPRINTF("Target List: %d\n", target_list_cnt);
}

/*
 * Remove a target by using the index.
 *
 * @param index	index of target that going to remove
 *
 * @return
 */
void path_targets_remove(int16_t index)
{
	int16_t i;

	/* If there is no more target, return. */
	if(index < 0) {
		return;
	}

	/* If it is the last target. */
	if (index == target_list_cnt - 1) {
		targetList[index].x = 0;
		targetList[index].y = 0;
		targetList[index].x_pos = 0;
		targetList[index].y_pos = 0;
		targetList[index].state = 0;
		targetList[index].try_cnt = 0;

		target_list_cnt--;

		return;
	}

	/*
	 * Loop for the target list and searching the index of target that should be remove. */
	for (i = index; i < target_list_cnt; i++) {
		if (i + 1 >= target_list_cnt) {
			target_list_cnt--;

			return;
		}

		/* Remove the targets which are not invalid. */
		if (targetList[i + 1].state == 0) {
			targetList[i].x = 0;
			targetList[i].y = 0;
			targetList[i].x_pos = 0;
			targetList[i].y_pos = 0;
			targetList[i].state = 0;
			targetList[i].try_cnt = 0;

			target_list_cnt--;

			break;
		}

		/* Refill the hole that caused by a removed target. */
		targetList[i].x = targetList[i + 1].x;
		targetList[i].y = targetList[i + 1].y;
		targetList[i].x_pos = targetList[i + 1].x_pos;
		targetList[i].y_pos = targetList[i + 1].y_pos;
		targetList[i].state = targetList[i + 1].state;
		targetList[i].try_cnt = targetList[i + 1].try_cnt;
		targetList[i + 1].x = SHRT_MIN;
		targetList[i + 1].y = SHRT_MIN;
		targetList[i + 1].x_pos = 0;
		targetList[i + 1].y_pos = 0;
		targetList[i + 1].state = 0;
		targetList[i + 1].try_cnt = 0;
	}
}

/*
 * Get the last target in the target list.
 *
 * @param *x	Pointer of X coordinate for the last target's X coordinate stores to
 * @param *y	Pointer of Y coordinate for the last target's Y coordinate stores to
 *
 * @return	0 if the last target is not found
 * 		1 if the last target is found
 */
uint8_t path_targets_get_last(int16_t *x, int16_t *y)
{
	uint8_t found = 0;
	int16_t	i;

	for (i = target_list_cnt - 1; i >= 0; i--) {
		if (targetList[i].state == 0)
			continue;

		/* If the target is not directly accessible. */
		if (is_block_accessible(targetList[i].x, targetList[i].y) == 0) {
			/* Remove the target that is no longer cleanable, which is blocked. */
			if (is_block_cleanable(targetList[i].x, targetList[i].y) == 0) {
				USPRINTF("remove\t%d\n", __LINE__);
				path_targets_remove(i);
				continue;
			} else {
				/*
				 * If the target that is not accessible, try to return the coordinate for cleaning it.
				 * If the coordinate for cleaning it is also not accessible, remove it.
				 * */
				if (is_block_accessible(targetList[i].x + targetList[i].x_pos, targetList[i].y + targetList[i].y_pos) == 0) {
					USPRINTF("remove\t%d\n", __LINE__);
					path_targets_remove(i);
					continue;
				} else {
					*x = targetList[i].x + targetList[i].x_pos;
					*y = targetList[i].y + targetList[i].y_pos;
					if (*x == positions[0].x && *y == positions[0].y) {
						USPRINTF("remove\t%d\n", __LINE__);
						path_targets_remove(i);
						continue;
					} else {
						found = 1;
						break;
					}
				}
			}
		} else {
			/* The target is directly accessible. */
			*x = targetList[i].x;
			*y = targetList[i].y;
			if (*x == positions[0].x && *y == positions[0].y) {
				USPRINTF("remove\t%d\n", __LINE__);
				path_targets_remove(i);
				continue;
			} else {
				found = 1;
				break;
			}
		}
	}

	USPRINTF("%s %d %d %d\n", __FUNCTION__, __LINE__, i, found);
	return found;
}

/*
 * Get the index of the target in the target list.
 *
 * @param x	X coordinate of the target
 * @param y	Y coordinate of the target
 *
 * @return	-1 if the target is not in the target list,
 * 		otherwise the index of target in the target list
 */
int16_t path_targets_get_index(int16_t x, int16_t y)
{
	int16_t	i = -1;

	for (i = target_list_cnt - 1; i >= 0; i--) {
		if (targetList[i].state == 0)
			continue;

		if ((targetList[i].x + targetList[i].x_pos) == x && (targetList[i].y + targetList[i].y_pos) == y)
			break;
	}

	return i;
}

/*
 * Get the number of targets that have the same Y coordinate in the target list.
 * Searching from both before & after the index of the given target, stop searching
 * when the Y coordinate is different from the Y coordinate of the target.
 *
 * @param x	X coordinate of the target
 * @param y	Y coordinate of the target
 *
 * @return	The number of target that have the same Y coordinate as the give target
 */
uint16_t path_targets_same_lane_count(int16_t x, int16_t y)
{
	int16_t i, j, index;

	i = j = 0;

	/* Get the index of the given target. If not found, return. */
	if ((index = path_targets_get_index(x, y)) <= 0)
		return 0;//index;

	/* Search the targets before the give target in the list. */
	for (i = 1; (index - i) >= 0; i++) {
		if (abs(targetList[index - i].y + targetList[index - i].y_pos - y) > 1)
		//if (targetList[index - i].y_pos != y)
			break;
	}

	/* Search the targets after the give target in the list. */
	for (j = 1; (index + j) < target_list_cnt; j++) {
		if ((targetList[index + j].state == 0) || ((targetList[index + j].y + targetList[index + j].y_pos) != y))
			break;
	}

	return (i + j - 1);
}

/*
 * Check whether a give target is not to add to the target list or not.
 *
 * @param x	X coordinate of the target that going to be added
 * @param y	Y coordinate of the target that going to be added
 * @param accessible	flag to ensure the give coodinate must be accessible or not
 *
 * @return	0 if it is not ok to add the target
 * 		1 if it is ok to add the target
 */
uint8_t path_targets_try_add_one(int16_t x, int16_t y, uint8_t accessible)
{
	int8_t	x_pos, y_pos;
	int16_t	i;

	x_pos = y_pos = 0;

	/*--------------------------- Edit By ZZ -----------------------*/
	if(is_near_boundary(x,y) == 1)
		return 0;
	/*****************************************************************/
	
	/*
	 * If the target is not accessible and either accessible flag or it is not cleanable,
	 * return fails. Otherwise, try to adjust the coordinate for cleaning the target.
	 */
	if (is_block_accessible(x, y) == 0) {
		if (accessible == 1 || is_block_cleanable(x, y) == 0) {
			return 0;
		} else {
			/* Check the 3 cells on the left hand side of the target is a block or not. */
			if (is_a_block(x - 2, y - 1) == 1 || is_a_block(x - 2, y) == 1 || is_a_block(x - 2, y + 1) == 1)
				x_pos += 1;

			/* Check the 3 cells on the right hand side of the target is a block or not. */
			if (is_a_block(x + 2, y - 1) == 1 || is_a_block(x + 2, y) == 1 || is_a_block(x + 2, y + 1) == 1)
				x_pos -= 1;

			/* Check the 3 cells in the back of the target is a block or not. */
			if (is_a_block(x - 1, y - 2) == 1 || is_a_block(x, y - 2) == 1 || is_a_block(x + 1, y - 2) == 1)
				y_pos += 1;

			/* Check the 3 cells in the front of the target is a block or not. */
			if (is_a_block(x - 1, y + 2) == 1 || is_a_block(x, y + 2) == 1 || is_a_block(x + 1, y + 2) == 1)
				y_pos -= 1;
		}
	}

	/* Get the index for the new target. */
	i = 0;
	while (i < target_list_cnt) {
		if (targetList[i].state > 0) {
			if (targetList[i].x == x && targetList[i].y == y) {
				path_targets_remove(i);
				//return 0;
			} else {
				i++;
			}
		} else {
			break;
		}
	}

	/* Check whether the target list is overflow. */
	if (i >= TARGET_TOTAL_COUNT) {
		return 0;
	}

	/* Final check the target is accessible or not by using the adjusted coordinate. */
	return ((is_block_accessible(x + x_pos, y + y_pos) == 1) ? 1 : 0);
}

/*
 * Add the give target to the target list.
 *
 * @param x	X coordinate of the target that going to be added
 * @param y	Y coordinate of the target that going to be added
 * @param accessible	flag to ensure the give coodinate must be accessible or not
 *
 * @return
 */
void path_targets_add_one(int16_t x, int16_t y, uint8_t accessible)
{
	int8_t	x_pos, y_pos;
	int16_t	i;

	x_pos = y_pos = 0;
	/*
	 * If the target is not accessible and either accessible flag or it is not cleanable,
	 * return fails. Otherwise, try to adjust the coordinate for cleaning the target.
	 */
	if (is_block_accessible(x, y) == 0) {
		if (accessible == 1 || is_block_cleanable(x, y) == 0) {
			return;
		} else {
			/* Check the 3 cells on the left hand side of the target is a block or not. */
			if (is_a_block(x - 2, y - 1) == 1 || is_a_block(x - 2, y) == 1 || is_a_block(x - 2, y + 1) == 1)
				x_pos += 1;

			/* Check the 3 cells on the right hand side of the target is a block or not. */
			if (is_a_block(x + 2, y - 1) == 1 || is_a_block(x + 2, y) == 1 || is_a_block(x + 2, y + 1) == 1)
				x_pos -= 1;

			/* Check the 3 cells in the back of the target is a block or not. */
			if (is_a_block(x - 1, y - 2) == 1 || is_a_block(x, y - 2) == 1 || is_a_block(x + 1, y - 2) == 1)
				y_pos += 1;

			/* Check the 3 cells in the front of the target is a block or not. */
			if (is_a_block(x - 1, y + 2) == 1 || is_a_block(x, y + 2) == 1 || is_a_block(x + 1, y + 2) == 1)
				y_pos -= 1;
		}
	}

	/* Get the index for the new target. */
	i = 0;
	while (i < target_list_cnt) {
		if (targetList[i].state > 0) {
			if (targetList[i].x == x && targetList[i].y == y) {
				/* remove the target which already in list */
				USPRINTF("remove\t%d\n", __LINE__);
				path_targets_remove(i);
			} else {
				i++;
			}
		} else {
			break;
		}
	}

	/* Check whether the target list is overflow. */
	if (i >= TARGET_TOTAL_COUNT) {
		USPRINTF("Target List is too long\n");
		path_targets_dump();
		return;
	}

	/* Final check the target is accessible or not by using the adjusted coordinate. */
	if (is_block_accessible(x + x_pos, y + y_pos) == 1) {
		//USPRINTF("Inserted: %d (%d, %d) (%d, %d)\n\n", i, x, y, x_pos, y_pos);
		target_list_cnt++;

		/* FIXME: i == target_list_cnt - 1? */
		targetList[i].x = x;
		targetList[i].y = y;
		targetList[i].x_pos = x_pos;
		targetList[i].y_pos = y_pos;
		targetList[i].state = 1;
		targetList[i].try_cnt = 0;

		/*
		 * Check whether the target is located in the entrance, if it is,
		 * set the target state to 2. If try_entrance flag is set, targets with state
		 * equal to 2 will be handled first.
		 */
		if (is_narrow_block_horizontal(x + x_pos, y + y_pos) || is_narrow_block_vertical(x + x_pos, y + y_pos))
			targetList[i].state = 2;
	} else {
		USPRINTF("Failed to insert: %d (%d, %d) (%d, %d)\n", i, x, y, x + x_pos, y + y_pos);
	}
}

/*
 * Insert one target at given index to the target list.
 *
 * @param index	index of the new target
 * @param x	X coordinate of the new target
 * @param y	Y coordinate of the new target
 * @param accessible	flag to ensure the give coodinate must be accessible or not
 *
 * @return
 */
void path_targets_insert_one(int16_t index, int16_t x, int16_t y, uint8_t accessible)
{
	int16_t i;

	/* Skip if given index is out of range. */
	if (index >= TARGET_TOTAL_COUNT)
		return;

	/* Skip if it is impossible to add the given target. */
	if (path_targets_try_add_one(x, y, accessible) == 0)
		return;

	target_list_cnt++;

	/* If the index is the last one, just add it. */
	if (index == target_list_cnt - 1) {
		targetList[index].x = x;
		targetList[index].y = y;
		targetList[index].x_pos = 0;
		targetList[index].y_pos = 0;
		targetList[index].state = 1;
		targetList[index].try_cnt = 0;
		return;
	}

	/*
	 * Loop for migrating the current targets in the list in order to free the location
	 * of the given index
	 */
	for (i = target_list_cnt - 2; i >= 0; i--) {
		if (i != index && targetList[i].state == 0)
			continue;

		targetList[i + 1] = targetList[i];

		if (i != index)
			continue;

		/* The given index is free now, add the target and stop. */
		targetList[i].x = x;
		targetList[i].y = y;
		targetList[i].x_pos = 0;
		targetList[i].y_pos = 0;
		targetList[i].state = 1;
		targetList[i].try_cnt = 0;
		break;
	}
}

/*
 * Add the given target to the end of the target list.
 *
 * @param target	The target to be added
 *
 * @return
 */
void path_targets_add_target(TargetType target)
{
	/* Skip if the target list is full. */
	if (target_list_cnt + 1 >= TARGET_TOTAL_COUNT) {
		USPRINTF("Target List is too long\n");
		path_targets_dump();
		return;
	}

	target_list_cnt++;

	/* Add the target to the end. */
	targetList[target_list_cnt - 1] = target;
}

/*
 * Check whether a give target is in the target list or not.
 *
 * @param x	X coordinate of the new target
 * @param y	Y coordinate of the new target
 *
 * @return	0 if the give target exists in the target list
 * 		1 if the give target is no in the target list
 */
uint8_t path_targest_exists(int16_t x, int16_t y) {
	uint8_t	can_add = 1;
	int16_t	j, min, max;

	min = SHRT_MAX;
	max = SHRT_MIN;
	for (j = target_list_cnt - 1; j >= 0; j--) {
		if (targetList[j].state == 0)
			continue;

		/* Find the min & max from the target that have the same Y coordinate. */
		if (targetList[j].y == y) {
			if (min > targetList[j].x)
				min = targetList[j].x;
			if (max < targetList[j].x)
				max = targetList[j].x;
		}
		if (min != max && min != SHRT_MAX && max != SHRT_MIN) {
			/* If the target is included in the range of min & max, no need to add it. */
			if (min <= x && x <= max) {
				can_add = 0;
				break;
			} else {
				/* Reset the range. */
				min = SHRT_MAX;
				max = SHRT_MIN;
			}
		}
	}

	return can_add;
}

/*
 * Base of the robot movement, add the targets to the target list.
 * When the robot is moving towards NORTH or SOUTH, add the targets
 * from (x1, y1 - 2) to (x2, y2 - 2) and from (x1, y + 2) to (x2, y2 + 2)
 * to the target list, where (x1, y1) is the robot starting coordinate
 * and (x2, y2) is the robot stopping coordinate. From the range of x1 to
 * x2, only add the targets which X coordinate are x1, x2 and those X
 * coordinate is odd number (x % 2 == 1), this is for using less memory.
 *
 * When the robot is moving towards EAST, only add (x2, y2 + 2),
 * (x2 + 2, y2 + 1) and (x2 - 2, y2 + 1).
 *
 * When the robot is moving towards EAST, only add (x2, y2 - 2),
 * (x2 - 2, y2 - 1) and (x2 + 2, y2 - 1).
 *
 * The targets add for EAST & WEST is to deal with the case that the robot
 * can still move when it place to be cleaned on fits the robot to get in.
 *
 * @param
 *
 * @return
 */
void path_targets_add() {
	uint8_t	can_add;
	int16_t	i, j, x, y, coff, last_added;
	uint8_t RightFirst;

	USPRINTF("Add targets: %d(%d), Gyro: %d\n", path_get_robot_direction(), Robot_GetDirection(), Gyro_GetAngle(0));

	/* Skip, if the robot is not moving. */
	if (last_dir == path_get_robot_direction() && last_x_pos == positions[0].x && last_y_pos == positions[0].y) {
		return;
	}

	/* Add targets when robot moving towards NORTH or SOUTH. */
	if (path_get_robot_direction() == SOUTH || path_get_robot_direction() == NORTH) {
		coff = path_get_robot_direction() == NORTH ? -1 : 1;
#if 1
		/*
		 * If the robot is moving towards NORTH, add the target in the front left.
		 * If the robot is moving towards SOUTH, add the target in the front right.
		 */
		x = nextXID(path_get_robot_direction(), coff, 2);
		y = nextYID(path_get_robot_direction(), coff, 2);
		if (Map_GetCell(MAP, x, y) == UNCLEAN && is_a_block(x, y) == 0) {
			//USPRINTF("added %s %d %d %d\n", __FUNCTION__, __LINE__, x, y);
			path_targets_add_one(x, y, 0);
		}

		/*
		 * If the robot is moving towards NORTH, add the target in the front right.
		 * If the robot is moving towards SOUTH, add the target in the front left.
		 */
		x = nextXID(path_get_robot_direction(), -1 * coff, 2);
		y = nextYID(path_get_robot_direction(), -1 * coff, 2);
		if (Map_GetCell(MAP, x, y) == UNCLEAN && is_a_block(x, y) == 0) {
			//USPRINTF("added %s %d %d %d\n", __FUNCTION__, __LINE__, x, y);
			path_targets_add_one(x, y, 0 );
		}
#endif

#if 1
		/*------------------------ Edit By ZZ --------------------------*/
		if(Map_GetYPos() >= Zone_GetCurrentZoneExit().Y)
		{
			RightFirst = 1;
		}
		else
		{
			RightFirst = 0;
		}
		for(j = 0;j<2;j++)
		{
			if(RightFirst)
			{
				RightFirst = 0;
				/*
				* If the robot is moving towards NORTH, add the targets in the left hand side.
				*
				* If the robot is moving towards SOUTH, add the targets in the right hand side.
				*/
				x = nextXID(path_get_robot_direction(), -2 * coff, 0);

				y = nextYID(path_get_robot_direction(), -2 * coff, 0);

				last_added = SHRT_MIN;
				for(i = last_x_pos; (coff == 1 && i >= x) || (coff == -1 && i <= x); i -= coff) {
					//USPRINTF("adding: x: %d\ty: %d\n", i, y, Map_GetCell(MAP, i, y + offset), is_a_block(i, y + coff), path_targets_try_add_one(i, y, 0));

					/* Add the first one the and last one. */
					if (last_added == SHRT_MIN || i == x) {
						if (Map_GetCell(MAP, i, y) == UNCLEAN && is_a_block(i, y - coff) == 0) {
							//if ((can_add = path_targest_exists(i, y)) == 1 ) {
								can_add = path_targets_try_add_one(i, y, 0);
								if (can_add == 1) {
									//USPRINTF("added %d %d, %d\n", __LINE__, i, y);
									path_targets_add_one(i, y, 0);
									last_added = i;
								} else if (i == x) {
									/* Handle the last target. */
									if (last_added != SHRT_MIN) {
										//USPRINTF("added %d %d, %d\n", __LINE__, last_added, y);
										path_targets_add_one(last_added, y, 0);
									}
								}
							//}
						}
					}
					else {
						if (Map_GetCell(MAP, i, y) == UNCLEAN && is_a_block(i, y - coff) == 0 && path_targets_try_add_one(i, y, 0) == 1) {
							last_added = i;
							//USPRINTF("%d last_added %d abs %d\n", __LINE__, last_added, abs(last_added) % 2);

							/* Only add the targets which X coordindate is odd, x % 2 == 1 */

							if (abs(last_added) % 2 == 1) {

								//USPRINTF("added %d %d, %d\n", __LINE__, i, y);
								path_targets_add_one(i, y, 0);
							}
						} else {
							//USPRINTF("added %s %d %d %d\n", __FUNCTION__, __LINE__, i+1, x);
							if (last_added != SHRT_MIN) {
								//USPRINTF("added %d %d, %d\n", __LINE__, last_added, y);
								path_targets_add_one(last_added, y, 0);
								last_added = SHRT_MIN;
							}
						}
					}
					//USPRINTF("\n");
				}
				continue;
			}
			else
			{
				RightFirst = 1;
				/*
				 * If the robot is moving towards NORTH, add the target in the right hand side first.
				 *
				 * If the robot is moving towards SOUTH, add the target in the left hand side first.
				 */
				x = nextXID(path_get_robot_direction(), 2 * coff, 0);

				y = nextYID(path_get_robot_direction(), 2 * coff, 0);

				USPRINTF("x: %d\ty: %d\tlast_x_pos: %d\tlast_y_pos:%d\tcoff: %d\t\n", x, y, last_x_pos, last_y_pos, coff);
				last_added = SHRT_MIN;

				for(i = last_x_pos; (coff == 1 && i >= x) || (coff == -1 && i <= x); i -= coff) {
					//USPRINTF("adding: x: %d\ty: %d\n", i, y, Map_GetCell(MAP, i, y), is_a_block(i, y + coff), path_targets_try_add_one(i, y, 0));

					/* Add the first one and the last one. */
					if (last_added == SHRT_MIN || i == x) {
						if (Map_GetCell(MAP, i, y) == UNCLEAN && is_a_block(i, y + coff) == 0) {
							//if ((can_add = path_targest_exists(i, y)) == 1 ) {
								can_add = path_targets_try_add_one(i, y, 0);
								if (can_add == 1) {
									//USPRINTF("added %d %d, %d\n", __LINE__, i, y);
									path_targets_add_one(i, y, 0);
									last_added = i;
								} else if (i == x) {
									/* Handle the last target. */
									if (last_added != SHRT_MIN) {
										//USPRINTF("added %d %d, %d\n", __LINE__, last_added, y);
										path_targets_add_one(last_added, y, 0);
									}
								}
							//}
						}
					}
					else {
						if (Map_GetCell(MAP, i, y) == UNCLEAN && is_a_block(i, y + coff) == 0 && path_targets_try_add_one(i, y, 0) == 1) {
							last_added = i;
							//USPRINTF("%d last_added %d abs %d\n", __LINE__, last_added, abs(last_added) % 2);

							/* Only add the targets which X coordindate is odd, x % 2 == 1 */
							if (abs(last_added) % 2 == 1) {

								//USPRINTF("added %d %d, %d\n", __LINE__, i, y);
								path_targets_add_one(i, y, 0);
							}
						}
						else {
							if (last_added != SHRT_MIN) {
								//USPRINTF("added %d %d, %d\n", __LINE__, last_added, y);
								path_targets_add_one(last_added, y, 0);
								last_added = SHRT_MIN;
							}
						}
					}
					//USPRINTF("\n");
				}
				continue;
			}
		
			//USPRINTF("\n");
		}
	}
#endif
	else {
#if 1
		/*
		 * When the robot is moving towards EAST, only add (x2, y2 + 2).
		 *
		 * When the robot is moving towards EAST, only add (x2, y2 - 2).
		 */
		x = nextXID(path_get_robot_direction(), 0, 2);
		y = nextYID(path_get_robot_direction(), 0, 2);

		can_add = path_targest_exists(x, y);
		if (can_add == 1 && Map_GetCell(MAP, x, y) == UNCLEAN && is_a_block(x, y + 1) == 0) {
			//USPRINTF("added %d %d, %d\n", __LINE__, x, y);
			path_targets_add_one(x, y, 0);
		}

		/*
		 * When the robot is moving towards EAST, only add (x2 + 2, y2 + 1).
		 *
		 * When the robot is moving towards EAST, only add (x2 - 2, y2 - 1).
		 */
		x = nextXID(path_get_robot_direction(), -2 , 1);
		y = nextYID(path_get_robot_direction(), -2, 1);
		if (Map_GetCell(MAP, x, y) == UNCLEAN && is_a_block(x, y - 1) == 0) {
			//USPRINTF("added %s %d %d %d\n", __FUNCTION__, __LINE__, x, y);
			if (path_targets_try_add_one(x, y, 0) == 1) {
				path_targets_add_one(x, y, 0);
			}
		}

		/*
		 * When the robot is moving towards EAST, only add (x2 - 2, y2 + 1).
		 *
		 * When the robot is moving towards EAST, only add (x2 + 2, y2 - 1).
		 */
		x = nextXID(path_get_robot_direction(), 2 , 1);
		y = nextYID(path_get_robot_direction(), 2, 1);

		can_add = path_targest_exists(x, y);
		if (can_add == 1 && Map_GetCell(MAP, x, y) == UNCLEAN && is_a_block(x, y - 1) == 0) {
			//USPRINTF("added %s %d %d %d\n", __FUNCTION__, __LINE__, x, y);
			path_targets_add_one(x, y, 0);
		}
#endif
		/*
		x = nextXID(path_get_robot_direction(), 0, -2);
		y = nextYID(path_get_robot_direction(), 0, -2);
		if (Map_GetCell(MAP, x, y) == UNCLEAN && is_a_block(x, y - 1) == 0)
			path_targets_add_one(x, y, 0);
		*/
	}

	/* Update last robot position and direction. */
	last_x_pos = positions[0].x;
	last_y_pos = positions[0].y;
	last_dir = path_get_robot_direction();

#if 1
	//path_targets_dump();
#endif
}

/*
 * Given the index of a target, update the coordinate for cleaning it.
 *
 * @param index	Index of the given target.
 *
 * @return
 */
void path_targets_update_one(uint16_t index)
{
	int8_t	x_pos, y, y_pos;
	int16_t	x;

	x = targetList[index].x;
	y = targetList[index].y;
	x_pos = y_pos = 0;
	if (is_block_accessible(x, y) == 0) {
		/* If the target is no longer cleanable, remove it. */
		if (is_block_cleanable(x, y) == 0) {
			USPRINTF("remove\t%d\n", __LINE__);
			path_targets_remove(index);
			return;
		} else {
			/* Check the 3 cells on the left hand side of the target is a block or not. */
			if (is_a_block(x - 2, y - 1) == 1 || is_a_block(x - 2, y) == 1 || is_a_block(x - 2, y + 1) == 1)
				x_pos += 1;

			/* Check the 3 cells on the right hand side of the target is a block or not. */
			if (is_a_block(x + 2, y - 1) == 1 || is_a_block(x + 2, y) == 1 || is_a_block(x + 2, y + 1) == 1)
				x_pos -= 1;

			/* Check the 3 cells in the back of the target is a block or not. */
			if (is_a_block(x - 1, y - 2) == 1 || is_a_block(x, y - 2) == 1 || is_a_block(x + 1, y - 2) == 1)
				y_pos += 1;

			/* Check the 3 cells in the front of the target is a block or not. */
			if (is_a_block(x - 1, y + 2) == 1 || is_a_block(x, y + 2) == 1 || is_a_block(x + 1, y + 2) == 1)
				y_pos -= 1;
		}
	}
	//USPRINTF("checking\tx: %d\ty: %d\tx_pos: %d\ty_pos: %d\n", x, y, x_pos, y_pos);

	/* If the adjust coordiante is not accessible, remove the target. */
	if (is_block_accessible(x + x_pos, y + y_pos) == 0) {
		USPRINTF("remove\t%d\n", __LINE__);
		path_targets_remove(index);
	} else {
		/* Update the coordinate for cleaning the target and it's state. */
		targetList[index].x_pos = x_pos;
		targetList[index].y_pos = y_pos;
		if (is_narrow_block_horizontal(x + x_pos, y + y_pos) || is_narrow_block_vertical(x + x_pos, y + y_pos))
			targetList[index].state = 2;
	}
}

/*
 * Update the weight for each target in the target list.
 *
 * @param
 *
 * @return
 */
int8_t path_targets_weight(int16_t x, int16_t y)
{
	int8_t		j, k, weight;
	CellState	cs;

	/* Find the weight for a single target. */
	weight = 0;
	for (k = ROBOT_RIGHT_OFFSET; k <= ROBOT_LEFT_OFFSET; k++) {
		for (j = ROBOT_RIGHT_OFFSET; j <= ROBOT_LEFT_OFFSET; j++) {
			cs = Map_GetCell(MAP, x + k, y + j);
			if (cs == UNCLEAN) {
				weight++;
			}
		}
	}

	//USPRINTF("%s %d %d\n", __FUNCTION__, __LINE__, weight);
	/*
	 * If the weight is greater than weight threshold, set it to 0,
	 * which indicate that the target needed to be clean.
	 */
	if (weight >= weight_cnt_threshold)
		weight = 0;

	return weight;
}

/*
 * Swap the last 2 target in the target list.
 * Swap is taken place when the robot tries to the same target more than 1 once.
 * This is to avoid the robot non-stopply trying the same target.
 *
 * @param
 *
 * @return
 */
void path_targets_swap()
{
	int8_t	found = 0;
	int16_t x, y, index;
	TargetType	t;

	/* Make sure the target list is not empty. Otherwise, stop. */
	found = path_targets_get_last(&x, &y);
	if (found == 0)
		return;

	USPRINTF("%s %d: %d %d %d %d %d\t%d %d %d %d\n", __FUNCTION__, __LINE__, positions[1].x, positions[1].y, positions[1].x_target, positions[1].y_target, positions[1].dir,
				positions[0].x, positions[0].y, x, y, path_get_robot_direction());

	/* If the robot is in the same position and same direction as before. */
	if ((positions[1].x_target == x && positions[1].y_target == y) &&
		(positions[0].x == positions[1].x && positions[0].y == positions[1].y && positions[1].dir == path_get_robot_direction())) {

		/* Make sure there are more than 2 targets in the target list. */
		index = path_targets_get_index(x, y);
		if (index < 1)
			return;

		USPRINTF("%s %d: should swap %d\n", __FUNCTION__, __LINE__, index);
		t = targetList[index];
		targetList[index] = targetList[index - 1];
		targetList[index - 1] = t;
		target_swapped = 1;
	}
}

/*
 * Function to move the targets have a higher state value to the end of the target list,
 * when the try_entrance flag is enabled.
 *
 * @param
 *
 * @return
 */
void path_targets_rearrange()
{
	uint8_t		stop;
	int16_t		i;
	TargetType	target;

	if (try_entrance == 0)
		return;

	stop = 0;
	while (!stop) {
		for (i = target_list_cnt - 1; i > 0; i--) {
			if (targetList[i].state == 0)
				continue;

			/* A target has higher state value is found. */
			if (targetList[i].state < targetList[i - 1].state) {
				target = targetList[i - 1];
				target.state = 1;
				USPRINTF("remove\t%d\n", __LINE__);
				path_targets_remove(i - 1);
				path_targets_add_target(target);
			}
		}

		if (i == 0)
			stop = 1;
	}
	path_targets_dump();
}

/*------------------------- Edit By ZZ ------------------------*/
//|ì?à???????¨???a?¤???¨o?à????|ì???D??????¨|?§|ì????à¨o|ì?¨?|??¨???¨¨??¨|?§,?¨1D?|ì??????¨¤??
void path_targets_update_oldtargets()
{
	int16_t		i;
	TargetType	target;
	uint16_t target_list_cnt_tmp = target_list_cnt;

	for (i = 0; i < target_list_cnt_tmp; i++) 
	{
		if (((targetList[i].y <= (Map_GetYPos()-1)) && (targetList[i].y >= (Map_GetYPos()-3)) && (Map_GetYPos() >= Zone_GetCurrentZoneExit().Y)) || 
				((targetList[i].y >= (Map_GetYPos()+1)) && (targetList[i].y <= (Map_GetYPos()+3)) && (Map_GetYPos() <= Zone_GetCurrentZoneExit().Y)))
		//?¨?2a|ì?à???DD|ì????¨°?DD¨?D??¨?D?????¨|?§|ì????à¨o|ì??ê?¨???¨¨??¨|?§
		{
			target = targetList[i];
			target.state = 1;
			path_targets_remove(i);
			path_targets_add_target(target);
			i--;
			target_list_cnt_tmp --;						//?¨1D?|ì??¨a2??¨?2a¨￠?
			USPRINTF("%s %d: update the target:(%d: %d,%d)\n",__FUNCTION__,__LINE__,i,target.x,target.y);
		}
	}
}
/**************************************************************/

/*
 * Update the targets in the target list. The following actions will be done:
 *
 * 	1. If the target is not unclean, remove it.
 * 	2. If the target has a try count greater than the maximum allowed try
 * 	count, remove it.
 * 	3. If the target is not longer cleanable, remove it.
 *	4. Depends on the robot movement, add the new targets.
 *	5. Swap the last 2 targets if it is needed to do so.
 *	6. If the try_entrance flag is set, rearrange the targets that have a
 *	greate state value to the end of the list
 *	7. If the weight_enabled flag is set, update the weight for each target.
 *
 * @param
 *
 * @return
 */
void path_targets_update() {
	int16_t	i;
	int16_t j,k;
	/* Add new targets. */
	path_targets_add();
	
	i = 0;
	while (i < target_list_cnt) {
		if (targetList[i].state == 0)
			break;

		/* Remove the targets if they are not unlcean. */
		/*---------------------------- Edit By ZZ ---------------------*/			//发生过目标点在边界外面的情况
		if ((Map_GetCell(MAP, targetList[i].x, targetList[i].y) != UNCLEAN) || 
			 (targetList[i].x > (Zone_GetCurrentZone().X + ZONE_SIZE_HALF)) || 
			 (targetList[i].y > (Zone_GetCurrentZone().Y + ZONE_SIZE_HALF)) ||
			 (targetList[i].x < (Zone_GetCurrentZone().X - ZONE_SIZE_HALF)) ||
			 (targetList[i].y < (Zone_GetCurrentZone().Y - ZONE_SIZE_HALF)) )
		{
			USPRINTF("remove\t%d %d (%d, %d)\n", __LINE__, i, targetList[i].x, targetList[i].y);
			path_targets_remove(i);
		} else {
			/* Remove the targets that has a try count greater than the allowed try count. */
			if (targetList[i].try_cnt > max_try_cnt) {
				USPRINTF("remove\t%d (%d, %d)\n", __LINE__, targetList[i].x, targetList[i].y);
				path_targets_remove(i);
				clear_block = 1;
			}

			/* Remove the targets that not longer cleanable. */
			if (is_block_cleanable(targetList[i].x, targetList[i].y) == 0) {
				USPRINTF("remove\t%d (%d, %d)\n", __LINE__, targetList[i].x, targetList[i].y);
				path_targets_remove(i);
			} else {
				/* Adjust the coordinate for cleaning the target. */
				path_targets_update_one(i);
				i++;
			}
		}
	}
	
	#ifdef VIRTUAL_WALL
	if(vw_pos_cnt!=0)
	{
		for(j=0;j<vw_pos_cnt;j++)
		{
			k = 0;
			while (k < target_list_cnt)
			{
				if((TwoPointsDistance( vw_pos[j].pos.X, vw_pos[j].pos.Y, targetList[k].x, targetList[k].y))<6)
				{
					path_targets_remove(k);
					Map_SetCell(MAP, cellToCount(targetList[k].x), cellToCount(targetList[k].y), CLEANED);
				}
				k++;
			}
		}
	}
	#endif
//	path_targets_swap();

	path_targets_rearrange();

	path_targets_dump();
	
	#ifdef DEBUG_PC_MAP
	for (i = 0; i < target_list_cnt; i++) {
		if (targetList[i].state > 0) {
			PC_NavDebug(targetList[i].x + targetList[i].x_pos,targetList[i].y + targetList[i].y_pos,0,9);
		}
	}			
	PC_NavDebug(Zone_GetCurrentZoneEntrance().X,Zone_GetCurrentZoneEntrance().Y,0,5);
	PC_NavDebug(Zone_GetCurrentZoneExit().X,Zone_GetCurrentZoneExit().Y,0,7);
	PC_NavDebug(Zone_GetCurrentZone().X,Zone_GetCurrentZone().Y,0,8);			
	#endif	
}

/*
 * Check both ends of a lane are cleaned or not.
 *
 * @param *x	Pointer to the X coordinate that the robot should move to clean
 * @param *y	Pointer to the Y coordinate that the robot should move to clean
 *
 * @return	0 if both ends are cleaned
 * 		1 if either one end is not cleaned
 * 		2 if both ends are not cleaned
 */
uint8_t path_lane_is_cleaned(int16_t *x, int16_t *y) {
	int16_t	i, found, min, max, min_stop, max_stop, x_tmp, y_tmp;

	min_stop = max_stop = found = 0;
	min = max = SHRT_MAX;
	x_tmp = *x;
	y_tmp = *y;

	for (i = 0; (min_stop == 0 || max_stop == 0); i++) {
		/* Find the unclean area in the SOUTH direction of the lane. */
		if (min_stop == 0) 
		{
			/* Stop if the cells is blocked, or reach the boundary. */
//			/*-------------------- Edit By ZZ --------------------*/
//			if (is_block_blocked(x_tmp - i, y_tmp) == 1 || is_block_boundary(x_tmp - i, y_tmp) == 1 || (Map_GetCell(MAP,x_tmp - (i + 2), y_tmp) == BLOCKED_BOUNDARY) || (Map_GetCell(MAP,x_tmp - (i + 3), y_tmp) == BLOCKED_BOUNDARY)) 
//			{//不加后面的条件可能会一直停在边界处（出现过）
//			/********************************************************/	
//				min_stop = 1;
//			}
//			else 
//			{
//				/*-------------------------- Edit By ZZ --------------------*/
//				if ((is_brush_block_unclean(x_tmp - i, y_tmp) == 1) && (is_brush_block_unclean(x_tmp - i - 1, y_tmp) == 1)) 
//				{
//					if (is_block_blocked(x_tmp - (i + 1), y_tmp) == 0)
//				/************************************************************/
//						min = i;
//					min_stop = 1;
//				}
//			}
			if(is_block_accessible(x_tmp - i,y_tmp) == 0)
				min_stop = 1;
			else if(Map_GetCell(MAP,x_tmp - i, y_tmp) == UNCLEAN)
			{
				if(Map_GetCell(MAP,x_tmp - (i + 2), y_tmp) == BLOCKED_BOUNDARY)
				{
					min_stop = 1;
					min = i - 1;
				}
				else
				{
					min = i;
					if((Map_GetCell(MAP,x_tmp - (i + 1), y_tmp) == CLEANED) && (Map_GetCell(MAP,x_tmp - (i + 3), y_tmp) != BLOCKED_BOUNDARY))
						min = i + 1;
				}
			}
		}

		/* Find the unclean area in the NORTH direction of the lane. */
		if (max_stop == 0) {
			/* Stop if the cells is blocked, or reach the boundary. */
			/*-------------------- Edit By ZZ --------------------*/
//			if (is_block_blocked(x_tmp + i, y_tmp) == 1 || is_block_boundary(x_tmp + i, y_tmp) == 1 || (Map_GetCell(MAP,x_tmp + (i + 2), y_tmp) == BLOCKED_BOUNDARY) || (Map_GetCell(MAP,x_tmp + (i + 3), y_tmp) == BLOCKED_BOUNDARY)) 
//			{//不加后面的条件可能会一直停在边界处（出现过）
//			/********************************************************/	
//				max_stop = 1;
//			}
//			else
//			{
//				/*---------------------------- Edit By ZZ -------------------*/
//				if ((is_brush_block_unclean(x_tmp + i, y_tmp) == 1) && (is_brush_block_unclean(x_tmp + i + 1, y_tmp) == 1)) 
//				{
//					if (is_block_blocked(x_tmp + i + 1, y_tmp) == 0)
//				/***************************************************************/
//						max = i;
//					max_stop = 1;
//				}
//			}
			if(is_block_accessible(x_tmp + i,y_tmp) == 0)
				max_stop = 1;
			else if(Map_GetCell(MAP,x_tmp + i, y_tmp) == UNCLEAN)
			{
				if(Map_GetCell(MAP,x_tmp + i + 2, y_tmp) == BLOCKED_BOUNDARY)
				{
					max_stop = 1;
					max = i - 1;
				}
				else
				{
					max = i;
					if((Map_GetCell(MAP,x_tmp + i + 1, y_tmp) == CLEANED) && (Map_GetCell(MAP,x_tmp + i + 3, y_tmp) != BLOCKED_BOUNDARY))
						max = i + 1;
				}
			}
		}
	}

	USPRINTF("%s %d %d %d\n", __FUNCTION__, __LINE__, min, max);
	/* Both ends are not cleaned. */
	if (min != SHRT_MAX && max != SHRT_MAX) {
		/*
		 * If the number of cells to clean are the same of both ends, choose either one base the
		 * previous robot positions. Otherwise, move to the end that have more unclean cells.
		 */
		if (min == max) {
			if (positions[2].y == positions[1].y) {
				if (positions[2].x == positions[1].x) {
					if (positions[0].x == positions[1].x) {
						*x += max;
					} else if (positions[0].x > positions[1].x) {
						*x -= min;
					} else {
						*x += max;
					}
				} else if ( positions[2].x > positions[1].x) {
					*x -= min;
				} else {
					*x += max;
				}
			} else if (positions[0].y == positions[1].y) {
				if (positions[0].x == positions[1].x) {
					*x += max;
				} else if (positions[0].x > positions[1].x) {
					*x += max;
				} else {
					*x -= min;
				}
			} else {
				*x += max;
			}
		}
		else if (min > max) {
			*x += max;
		}
		else {
			*x -= min;
		}

		found = 2;
	} else if (min != SHRT_MAX) {
		/* Only the SOUTH end is not cleaned. */
		*x -= min;
		found = 1;
	} else if (max != SHRT_MAX ) {
		/* Only the NORTH end is not cleaned. */
		*x += max;
		found = 1;
	}

	return found;
}

/*
 * Function to find the next cleaning target.
 * When a target on the left hand side of the map is found, and it has a greater
 * Y coordinate than the last target in the target list, clean such target first.
 * Otherwise, clean the last target in the target list.
 *
 * @param *x	Pointer to the X coordinate that the robot should move to clean
 * @param *y	Pointer to the Y coordinate that the robot should move to clean
 *
 * @return	0 if no more target to be cleaned
 * 		1 if found a target to be cleaned
 */
int8_t path_next_approaching(int16_t *x, int16_t *y)
{
	int8_t	found_left, found_last, found, weight;
	int16_t	i, j, k, x_last, y_last, x_left, y_left, index, index_left, index_last, cost, y_max;

	CellState	cs;

#ifndef SHORTEST_PATH_V2
	int16_t	cost1;
#endif

	/* Find the most left target, which has the greatest Y coodinate, in the target list. */
	found_left = found_last = found  = 0;
	while (!found_left) {
		y_max = SHRT_MIN;
		index_left = -1;
		for (i = target_list_cnt - 1; i >= 0; i--) {
			if (targetList[i].state == 0) {
				continue;
			}

			if (targetList[i].y > y_max) {
				/* FIXME: -1 or -2, -2 should be used when clean with 195 per lane. */
				if (Map_GetCell(MAP, targetList[i].x, targetList[i].y - 1) == CLEANED) {
					y_max = targetList[i].y;
					x_left = targetList[i].x + targetList[i].x_pos;
					y_left = targetList[i].y + targetList[i].y_pos;
					index_left = i;
				}
			}
		}

		if (i == -1 && index_left == -1) {
			break;
		}

		/* Check the weight of the target, if it is less than weight threshold, remove it and try again. */
		if (weight_enabled == 1) {
			weight = path_targets_weight(targetList[index_left].x, targetList[index_left].y);
			if (weight > 0 && weight < weight_cnt_threshold) {
				USPRINTF("%s %d: remove target: (%d, %d), weight: %d\n",
					__FUNCTION__, __LINE__, targetList[index_left].x, targetList[index_left].y, weight);
				path_targets_remove(index_left);
			} else {
				found_left = 1;
			}
		} else {
			found_left = 1;
		}
	}

	/* Get the last target in the target list. */
	found_last = 0;
	while (!found_last) {
		found = path_targets_get_last(&x_last, &y_last);
		if (found == 0)
			return found;

		index_last = path_targets_get_index(x_last, y_last);
		if (index_last < 0) {
			USPRINTF("%s %d: target index: %d (%d, %d)\n", __FUNCTION__, __LINE__, index_last, x_last, y_last);
			return 0;
		}

		/* Check the weight of the target, if it is less than weight threshold, remove it and try again. */
		if (weight_enabled == 1) {
			weight = path_targets_weight(targetList[index_last].x, targetList[index_last].y);
			if (weight > 0 && weight < weight_cnt_threshold) {
				USPRINTF("%s %d: remove target: (%d, %d), weight: %d\n",
					__FUNCTION__, __LINE__, targetList[index_last].x, targetList[index_last].y, weight);
				path_targets_remove(index_last);
			} else {
				found_last = 1;
			}
		} else {
			found_last = 1;
		}
	}

	/*
	 * If the most left target and the last target are found, compare their Y
	 * coordinate, the one has a greater Y coordinate will be clean first.
	 */
	if (0 && found_left == 1 && y_left >= y_last) {
		index = index_left;
		*x = x_left;
		*y = y_left;
		found = 1;
	} else {
		/* Only last one if found. */
		index = index_last;
		*x = x_last;
		*y = y_last;
		found = 1;
	}

	if (*x == positions[0].x && abs(*y - positions[0].y) == 2) {
		k = 1;
		i = *y;
		while (k == 1) {
			if (is_block_accessible(*x, i) == 0) {
				if (*y > positions[0].y && i > *y) {
					i--;
				} else if (*y < positions[0].y && *y < i) {
					i++;
				}
				USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
				break;
			}
			if (Map_GetCell(MAP, *x - 2, i) == UNCLEAN && Map_GetCell(MAP, *x - 3, i) != BLOCKED_BOUNDARY) {
				USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
				break;
			}
			if (Map_GetCell(MAP, *x + 2, i) == UNCLEAN && Map_GetCell(MAP, *x + 3, i) != BLOCKED_BOUNDARY) {
				USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
				break;
			}
			weight = 0;
			for (j = -1; j <= 1; j++) {
				cs = Map_GetCell(MAP, *x + j, i + (*y > positions[0].y ? 2 : - 2));
				if (cs != CLEANED && cs!= UNCLEAN) {
					if (*y > positions[0].y && i > *y) {
						USPRINTF("%s %d: %d\n", __FUNCTION__, __LINE__, i);
						i--;
					} else if (*y < positions[0].y && *y < i) {
						USPRINTF("%s %d: %d\n", __FUNCTION__, __LINE__, i);
						i++;
					}
					k = 0;
					USPRINTF("%s %d: %d\n", __FUNCTION__, __LINE__, i);
					break;
				}
				if (cs == CLEANED) {
					weight++;
				}
				if (cs == BLOCKED_BOUNDARY) {
					break;
				}
			}
			if (weight == 3) {
				break;
			}
			if (k == 1) {
				i += (*y > positions[0].y ? 1 : -1);
			}
		}
		
		if(!(((*y>positions[0].y)&&(i<positions[0].y))||((*y<positions[0].y)&&(i>positions[0].y))))
		{		
			if (*y != i) {
				USPRINTF("%s %d: reset target from (%d, %d) to (%d, %d)\n\n", __FUNCTION__, __LINE__, *x, *y, *x, i);
				*y = i;
			}
		}
	}

#if 0
	/*
	 * Check the target is reachable or not. If it is reachable, increase it try count.
	 * If the try count is greater than the maximum allowed try count, remove it from the list.
	 */
	if ((cost = path_find_shortest_path(positions[0].x, positions[0].y, *x, *y, 0, last_dir)) <= 0) {
		targetList[index].try_cnt++;
		USPRINTF("no path to: %d (%d, %d) (%d, %d) %d\n", __LINE__, positions[0].x, positions[0].y, *x, *y, targetList[index].try_cnt);
		if (targetList[index].try_cnt > max_try_cnt) {
			USPRINTF("remove\t%d (%d, %d) (%d, %d)\n", __LINE__, positions[0].x, positions[0].y, *x, *y);
			path_targets_remove(index);
		}
	}
#endif

	if (found) {
		USPRINTF("approaching found: %d\tx: %d\ty: %d\t(%d, %d)\n",  found, targetList[index].x, targetList[index].y, *x, *y);

#ifndef SHORTEST_PATH_V2
		/*
		 * The following code is to handle the case that when the last 2 targets have
		 * the same lane ID but in different row:
		 *
		 *	 		Lane:
		 * 		y	y+n
		 *
		 * x:		R	0
		 * x+1:			T2
		 * x+2:			0
		 * x+3:			0
		 * x+4:			0
		 * x+5:			T1
		 *
		 * In above, if the next target is T1, and T2 is the target after T1. R is the
		 * robot position. The robot maybe first move to (x + 5, y), after that it will
		 * move to (x + 5, y + n), finally clean the whole lan of (y + n) and stop at
		 * (x, y + n). With the following code, it will either move to (x + 1, y) or
		 * (x, y + n), then (x + 1, y + n), finally clean the whole lan of (y + n) and
		 * stop at (x + 5, y + n).
		 *
		 * In shortest path version 2 uses the up-side-down tree for searching the path,
		 * between a tree node and its higher level node(or parent), the lane ID
		 * difference is 1. When searching the path to target, the horizontal movement
		 * always has a higher priority, so that it doesn't need the following to
		 * optimize the path.
		 */

		/*
		 * If the previous target refer to the current target has a lower cost,
		 * replace it as the current target.
		 */
		if (target_swapped == 0 && index > 0 && targetList[index - 1].y == targetList[index].y && cost > 0) {
			cost1 = path_find_shortest_path(positions[0].x, positions[0].y, targetList[index - 1].x + targetList[index - 1].x_pos, *y, 0, last_dir);
			USPRINTF("%s %d %d %d %d %d\n", __FUNCTION__, __LINE__, cost1, cost, targetList[index - 1].x + targetList[index - 1].x_pos, *y);
			if (cost1 > 0 && cost1 < cost) {
				*x = (targetList[index - 1].x + targetList[index - 1].x_pos);
			}
		} else {
			target_swapped = 0;
		}
#endif

		USPRINTF("%s %d: %d %d %d %d %d %d\n", __FUNCTION__, __LINE__,  positions[0].x, positions[1].x, positions[0].y, positions[1].y,last_dir, path_get_robot_direction());
#if 0
		/* Increase the target try count if the robot movement is only 1 cell. */
		if ((abs(positions[0].x - positions[1].x) <= 1) && (abs(positions[0].y - positions[1].y) <= 1) && last_dir == path_get_robot_direction()) {
			targetList[index].try_cnt++;
		} else {
			if (cost >= 0) {
				targetList[index].try_cnt = 0;
			}
		}
#endif
	} else {
		USPRINTF("approaching not found.\r\n");
	}
	return found;
}

/*
 * Find how many cells ahead to clean with a given target.
 *
 * @param x	X coordinate of robot before moving to the target
 * @param y	Y coordinate of robot before moving to the target
 * @param x_next	X coordinate of target
 *
 * @return
 */
int16_t path_ahead_to_clean(int16_t x, int16_t y, int16_t x_next)
{
	int16_t offset;

	/*
	 * Stop when the target's X coordinate is not to the boundary of the map.
	 * Since this kind of target is must be the target point for the cleaning
	 * destination.
	 */
	if (!(x == SHRT_MIN || x == SHRT_MAX))
		return 0;

	/*
	 * Find the offset about how many cells to clean ahead with
	 * reference to the given point x_next.
	 */
	offset = (x == SHRT_MIN ? -2 : 2);
	while ((x == SHRT_MIN && offset > x) || (x == SHRT_MAX && offset < x)) {
		/* Reach the boundary, stop. */
		if ((x == SHRT_MIN && x_next + offset <= xMin) || (x == SHRT_MAX && x_next + offset >= xMax) ) {
			offset = 0;
			break;
		} else if ((is_brush_block_unclean(x_next + offset, y) == 1) && 
		/*-------------------------------- Edit By ZZ ---------------------------*/
							 (Map_GetCell(MAP,x_next + offset + (x == SHRT_MIN ? -1 : 1) , y) != BLOCKED_BOUNDARY) && 
							 (Map_GetCell(MAP,x_next + offset + (x == SHRT_MIN ? -2 : 2) , y) != BLOCKED_BOUNDARY) && 
							 (Map_GetCell(MAP,x_next + offset + (x == SHRT_MIN ? -3 : 3) , y) != BLOCKED_BOUNDARY)) {
		/*************************************************************************************/
			offset += (x == SHRT_MIN ? -1 : 1);
		} else {
			break;
		}
	}
	/*--------------------------- Edit By ZZ ---------------------------*/
	offset -= (x == SHRT_MIN ? -2 : 2);
	/********************************************************************/
		
	//USPRINTF("%s %d %d %d %d\n", __FUNCTION__, __LINE__, x, y, offset);
	return offset;
}

/*
 * Update the Map cells for when robot move towards NORTH or SOUTH.
 * This is for avoiding the cells around the robot position is/are marked
 * as not accessible, since it shouldn't happen, the robot occupies 5x5
 * cells, if the robot is pass through, it means that the obstcal marked
 * before should be cleared.
 *
 * @param
 *
 * @return
 */
void path_update_cells()
{
	int16_t 	i, start, end;
	int32_t		y;
//	CellState	cs;

	/* Skip, if robot is not moving towards NORTH or SOUTH. */
	if ((path_get_robot_direction() % 1800) != 0)
		return;

	start = positions[1].x > positions[0].x ? positions[0].x : positions[1].x;
	end = positions[1].x > positions[0].x ? positions[1].x : positions[0].x;
	USPRINTF("%s %d: start: %d\tend: %d\n", __FUNCTION__, __LINE__, start, end);
	for (i = start; i <= end; i++) {
		/* Check the Map cells which Y coordinate equal (y - 2). */
		y = positions[0].y - 1;

		if (Map_GetCell(MAP, i, y) == BLOCKED_OBS || Map_GetCell(MAP, i, positions[0].y - 2) == BLOCKED_BUMPER) {
			if ( i == start) {
				if (Map_GetCell(MAP, i + 1, y) == CLEANED) {
					USPRINTF("%s %d: reset (%d, %d) to cleaned.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					USPRINTF("%s %d: reset (%d, %d) to unclean.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			} else if (i == end) {
				if (Map_GetCell(MAP, i - 1, y) == CLEANED) {
					USPRINTF("%s %d: reset (%d, %d) to cleaned.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					USPRINTF("%s %d: reset (%d, %d) to unclean.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			} else {
				if (Map_GetCell(MAP, i - 1, y) == CLEANED || Map_GetCell(MAP, i + 1, y) == CLEANED) {
					USPRINTF("%s %d: reset (%d, %d) to cleaned.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					USPRINTF("%s %d: reset (%d, %d) to unclean.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			}
		}

		/* Check the Map cells which Y coordinate equal (y + 2). */

		y = positions[0].y + 1;


		if (Map_GetCell(MAP, i, y) == BLOCKED_OBS || Map_GetCell(MAP, i, y) == BLOCKED_BUMPER) {
			if ( i == start) {
				if (Map_GetCell(MAP, i + 1, y) == CLEANED) {
					USPRINTF("%s %d: reset (%d, %d) to cleaned.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					USPRINTF("%s %d: reset (%d, %d) to unclean.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			} else if (i == end) {
				if (Map_GetCell(MAP, i - 1, y) == CLEANED) {
					USPRINTF("%s %d: reset (%d, %d) to cleaned.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					USPRINTF("%s %d: reset (%d, %d) to unclean.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			} else {
				if (Map_GetCell(MAP, i - 1, y) == CLEANED || Map_GetCell(MAP, i + 1, y) == CLEANED) {
					USPRINTF("%s %d: reset (%d, %d) to cleaned.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					USPRINTF("%s %d: reset (%d, %d) to unclean.\n", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			}
		}
	}

	/*
	 * The following is try to handle the cases:
	 *
	 * 1. Robot moves towards NORTH
	 *
	 * 		1110	0111
	 * 		1x1e	e1x1
	 *		1110	0111
	 *		0uuu	uuu0
	 *
	 * 2. Robot moves towards SOUTH
	 *
	 *		0uuu	uuu0
	 * 		1110	0111
	 * 		1x1e	e1x1
	 *		1110	0111
	 *
	 * Where u is a value either 2 or 3, x is the current robot position,
	 * e is the target position. With the above changes, the movement of the
	 * robot will be looks nicer.
	 */
	 
//	if (path_get_robot_direction() == NORTH || path_get_robot_direction() == SOUTH) {
//		if (path_get_robot_direction() == NORTH && positions[0].x > positions[1].x) {
//			if (positions[0].y >= 0 && Map_GetCell(MAP, positions[0].x, positions[0].y + 2) == UNCLEAN) {
//				for (i = 0; i < 3; i++) {
//					cs = Map_GetCell(MAP, positions[0].x + 2, positions[0].y + i);
//					if (cs != CLEANED && cs != UNCLEAN) {
//						USPRINTF("%s %d: reset (%d, %d) to %d.\n", __FUNCTION__, __LINE__, positions[0].x + 3, positions[0].y + i, cs);
//						Map_SetCell(MAP, cellToCount(positions[0].x + 3), cellToCount(positions[0].y + i), cs);

//						USPRINTF("%s %d: reset (%d, %d) to unclean.\n", __FUNCTION__, __LINE__, positions[0].x + 2, positions[0].y + i);
//						Map_SetCell(MAP, cellToCount(positions[0].x + 2), cellToCount(positions[0].y + i), UNCLEAN);
//					}
//				}
//			} else if (positions[0].y < 0 && Map_GetCell(MAP, positions[0].x, positions[0].y - 2) == UNCLEAN) {
//				for (i = 0; i < 3; i++) {
//					cs = Map_GetCell(MAP, positions[0].x + 2, positions[0].y - i);
//					if (cs != CLEANED && cs!= UNCLEAN) {
//						USPRINTF("%s %d: reset (%d, %d) to %d.\n", __FUNCTION__, __LINE__, positions[0].x + 3, positions[0].y - i, cs);
//						Map_SetCell(MAP, cellToCount(positions[0].x + 3), cellToCount(positions[0].y - i), cs);

//						USPRINTF("%s %d: reset (%d, %d) to unclean.\n", __FUNCTION__, __LINE__, positions[0].x + 2, positions[0].y - i);
//						Map_SetCell(MAP, cellToCount(positions[0].x + 2), cellToCount(positions[0].y - i), UNCLEAN);
//					}
//				}
//			}
//		} else if (path_get_robot_direction() == SOUTH && positions[0].x < positions[1].x) {
//			if (positions[0].y >= 0 && Map_GetCell(MAP, positions[0].x, positions[0].y + 2) == UNCLEAN) {
//				for (i = 0; i < 3; i++) {
//					cs = Map_GetCell(MAP, positions[0].x - 2, positions[0].y + i);
//					if (cs != CLEANED && cs!= UNCLEAN) {
//						USPRINTF("%s %d: reset (%d, %d) to %d.\n", __FUNCTION__, __LINE__, positions[0].x - 3, positions[0].y + i, cs);
//						Map_SetCell(MAP, cellToCount(positions[0].x - 3), cellToCount(positions[0].y + i), cs);

//						USPRINTF("%s %d: reset (%d, %d) to unclean.\n", __FUNCTION__, __LINE__, positions[0].x - 2, positions[0].y + i);
//						Map_SetCell(MAP, cellToCount(positions[0].x - 2), cellToCount(positions[0].y + i), UNCLEAN);
//					}
//				}
//			} else if (positions[0].y < 0 && Map_GetCell(MAP, positions[0].x, positions[0].y - 2) == UNCLEAN) {
//				for (i = 0; i < 3; i++) {
//					cs = Map_GetCell(MAP, positions[0].x - 2, positions[0].y - i);
//					if (cs != CLEANED && cs!= UNCLEAN) {
//						USPRINTF("%s %d: reset (%d, %d) to %d.\n", __FUNCTION__, __LINE__, positions[0].x - 3, positions[0].y - i, cs);
//						Map_SetCell(MAP, cellToCount(positions[0].x - 3), cellToCount(positions[0].y - i), cs);

//						USPRINTF("%s %d: reset (%d, %d) to unclean.\n", __FUNCTION__, __LINE__, positions[0].x - 2, positions[0].y - i);
//						Map_SetCell(MAP, cellToCount(positions[0].x - 2), cellToCount(positions[0].y - i), UNCLEAN);
//					}
//				}
//			}
//		}
//	}
}

/*
 * Check whether the robot is trapped or not. The robot is trapped if there
 * is no path to (0, 0) or home.
 *
 * @param
 *
 * @return	0 if the robot is trapped
 * 		1 if the robot is not trapped.
 */
int8_t path_escape_trapped() {

	int8_t	val=1;
	path_set_current_pos();
	uint8_t i = 0;
		for ( i = 0; i < trappedCellSize; ++i ) {
			USPRINTF("%s %d Check %d trapped reference cell: x: %d, y: %d\n", __FUNCTION__, __LINE__,
			         i, trappedCell[i].X, trappedCell[i].Y);
			if (is_block_accessible(trappedCell[i].X, trappedCell[i].Y) == 0) {
				Map_Set_Cells(ROBOT_SIZE, trappedCell[i].X, trappedCell[i].Y, CLEANED);
			}

			val = path_find_shortest_path( positions[0].x, positions[0].y, trappedCell[i].X, trappedCell[i].Y, 0, last_dir );
			USPRINTF("%s %d: val %d\n", __FUNCTION__, __LINE__, val);
			if (val < 0 || val == SCHAR_MAX) {
				/* No path to home, which is set when path planning is initialized. */
				val = 0;
				break;
			} else {
				val = 1;
				
			}
		}
	return val;
}

/*
 * Function to find the next target to clean. When the robot goes to a new
 * lane, it will try to make sure the new lane is cleaned on both ends, then
 * try to find a target to clean from the target list. The logic below will let
 * the robot move in ZigZag. and robot always try to clean the most left hand
 * side(which target has a greater Y coordiante) with reference from the grid Map.
 *
 * @param *target_x	Pointer to target position's X coordinate
 * @param *target_y	Pointer to target position's Y coordinate
 *
 * @return	0 if no more target is found
 * 		1 if a target is found
 * 		2 if robot is trapped
 * 		-1 if target is blocked
 */
int8_t path_next(int32_t *target_x, int32_t *target_y) {
	int8_t	val;
	uint8_t status;
	int16_t	x, y, /*cnt,*/ x_next_area, y_next_area, offset;
	int8_t i, j;
	Point16_t	pos;
	static uint8_t MapOptimizationFlag = 0;
	static Point16_t MapOptimizationCell;
	

	/*------------------------------- Edit By ZZ ------------------------*/
	if(MapOptimizationFlag)
	{
		if(CM_ABS(Map_GetCurrentCell().X,MapOptimizationCell.X)>5 || (CM_ABS(Map_GetCurrentCell().Y,MapOptimizationCell.Y)>5))
		{
			MapOptimizationFlag = 0;
		}
	}
	if(PP_RoundingMovingFlag == 0)
		Map_Set_Cells_IgnoreCells(ROBOT_SIZE,Map_GetXPos(),Map_GetYPos(),CLEANED,BLOCKED_BOUNDARY);
	/**********************************************************************/
	
	/* Update the robot position history. */
	path_set_current_pos();

	val = 1;
	/* Update the Map cells to avoid cells passed by the robot are marked as obstcals. */
//	path_update_cells();

	USPRINTF("\r\npath_next\tx: %d\ty: %d\tlx: %d\tly: %d\n", positions[0].x, positions[0].y, last_x_pos, last_y_pos);

	/*
	 * Check the current lane is clean or not and make sure the robot
	 * not non-stopply hit the wall, especially the wall with the black bricks.
	 */
	x_next_area = positions[0].x;
	y_next_area = positions[0].y;
	status = path_lane_is_cleaned(&x_next_area, &y_next_area);
	if (status == 1 && positions[0].x == positions[1].x && positions[0].y == positions[1].y && positions[0].dir == positions[1].dir) {
		status = 0;
	}

	if ( status == 1 ) {
		uint8_t unCleanedCnt = 0;
		for ( i = -1; i <= 1; ++i ) {
			for ( j = -1; j <= 1; ++j ) {
				if ( Map_GetCell(MAP, x_next_area + i, y_next_area + j) == UNCLEAN ) {
					unCleanedCnt++;
				}
			}
		}

		if ( unCleanedCnt < 2) {
			status = 0;
		}
	}
	/*------------------------ Edit By ZZ -----------------------*/
	if(status != 0)
	{
		should_follow_wall = 0;
	}
	/**************************************************************/
	USPRINTF("status: %d\tx next: %d\ty next: %d\tcell: %d\n",
			status, x_next_area, y_next_area, Map_GetCell(MAP, x_next_area, y_next_area));

	/* Update the target list. */
	path_targets_update();

	/*
	 * Clear the blocks as needed. A block as below
	 *
	 * 	111
	 * 	1u1
	 * 	111
	 *
	 * where u will be either 2 or 3, reset to
	 *
	 * 	111
	 * 	111
	 * 	111
	 *
	 */

	if (clear_block == 1) {
		USPRINTF("Clear block\r\n");
		clear_block = 0;
		//return -1;
	}

	if (status > 0) {
//		y = positions[0].y;
//		USPRINTF("%s %d %d %d\n", __FUNCTION__, __LINE__, positions[1].x, positions[2].x);
//		if (x_next_area > positions[0].x) {
//			x = (is_block_cleaned(x_next_area + 1, y_next_area) == 0) ? SHRT_MAX : x_next_area;

//			if ((offset = path_ahead_to_clean(x, y_next_area, x_next_area)) != 0) {
//				USPRINTF("%s %d %d %d %d\n", __FUNCTION__, __LINE__, x, x_next_area, offset);
//				x = x_next_area + offset;// + (x == SHRT_MAX ? 2 : -2);
//			}
//		} else {
//			x = (is_block_cleaned(x_next_area - 1, y_next_area) == 0) ? SHRT_MIN : x_next_area;

//			if ((offset = path_ahead_to_clean(x, y_next_area, x_next_area)) != 0) {
//				USPRINTF("%s %d %d %d %d\n", __FUNCTION__, __LINE__, x, x_next_area, offset);
//				x = x_next_area + offset;// + (x == SHRT_MIN ? -2 : 2);
//			}
//		}
//		if (first_start == 0)
//			first_start++;

//		positions[0].x_target = x_next_area;
//		positions[0].y_target = y_next_area;
	} else {
		/* Get the next target to clean. */
		val = path_next_approaching(&x_next_area, &y_next_area);
		if (val > 0) {
			if (first_start == 1)
				first_start++;

			positions[0].x_target = x_next_area;
			positions[0].y_target = y_next_area;

			/* Find the path to the next target to clean. */
			pos.X = Map_GetXPos();
			pos.Y = Map_GetYPos();
			val = path_move_to_unclean_area(pos, x_next_area, y_next_area, &x, &y, last_dir);
			USPRINTF("%s %d %d %d %d %d\n", __FUNCTION__, __LINE__, x_next_area, y_next_area, x, y);

			/*
			 * If there is a path to the target, adjust the action in a more continue way, so
			 * that the action sequence like move straight, stop, move straight again won't happen.
			 */
			if (val > 0) {
//				val = 1;
//				if (y == positions[0].y) {
//					if (Map_GetCell(MAP, x, y) == CLEANED) {
//						if (x > positions[0].x && x_next_area > positions[0].x) {
//							x = (is_block_cleaned(x + 1, y) == 0) ? SHRT_MAX : x;

//							if ((offset = path_ahead_to_clean(x, y, x_next_area)) != 0) {
//								x = x_next_area + offset;// + (x == SHRT_MAX ? 2 : -2);
//							}
//						} else if (x < positions[0].x && x_next_area < positions[0].x) {
//							USPRINTF("%s %d %d %d\n", __FUNCTION__, __LINE__, x, is_block_cleaned(x - 1, y));
//							x = (is_block_cleaned(x - 1, y) == 0) ? SHRT_MIN : x;

//							if ((offset = path_ahead_to_clean(x, y, x_next_area)) != 0) {
//								USPRINTF("%s %d %d %d %d\n", __FUNCTION__, __LINE__, x, x_next_area, offset);
//								x = x_next_area + offset;// + (x == SHRT_MIN ? -2 : 2);
//							}
//						}
//					}
//				} else if (abs(y - positions[0].y) == 1) {
//					if ( x != positions[0].x) {
//						cnt = path_targets_same_lane_count(x, y);
//						if (cnt > 0) {
//							if (x > positions[0].x) {
//								x += cnt;
//							} else {
//								x -= cnt;
//							}
//						}
//					}
//				}

//				offset = path_targets_get_index(x_next_area, y_next_area);
//				if ((abs(positions[0].x - positions[1].x) <= 1) && (abs(positions[0].y - positions[1].y) <= 1) && last_dir == path_get_robot_direction()) {
//					targetList[offset].try_cnt++;
//				} else {
//					targetList[offset].try_cnt = 0;;
//				}
			} else {
				offset = path_targets_get_index(x_next_area, y_next_area);
				targetList[offset].try_cnt++;
				USPRINTF("no path to: %d (%d, %d) (%d, %d) %d\n", __LINE__, positions[0].x, positions[0].y, x_next_area, y_next_area, targetList[offset].try_cnt);
				if (targetList[offset].try_cnt > max_try_cnt) {
					USPRINTF("remove\t%d (%d, %d) (%d, %d)\n", __LINE__, positions[0].x, positions[0].y, x_next_area, y_next_area);
					path_targets_remove(offset);
				}
				USPRINTF("%s %d %d %d %d %d %d\n", __FUNCTION__, __LINE__, val, x_next_area, y_next_area, x_next_area, y_next_area);
			}
		} else {
			USPRINTF("%s %d %d %d %d %d %d\n", __FUNCTION__, __LINE__, val, x_next_area, y_next_area, x, y);
		}
		if (val == -2) {
			/* Robot is trapped and no path to starting point or home. */
			if (path_escape_trapped() == 0) {
				return 2;
			} else {
				return -1;
			}

		}
	}

	if (val > 0) {
		/*
		 * If the flag PP_MOVE_TO_CELL_CENTER is set, force the robot to move to
		 * the center of a cell for each action, otherwise, just use the encoder
		 * counter plus the offsets.
		 */
		if(status > 0)
		{
			x = x_next_area;
			y = y_next_area;
		}
#ifdef PP_MOVE_TO_CELL_CENTER
		*target_x = (x == positions[0].x) ? Map_GetXCount() : cellToCount(x);
		*target_y = (y == positions[0].y) ? Map_GetYCount() : cellToCount(y);
#else
		*target_x = Map_GetXCount() + (x - positions[0].x) * CELL_COUNT_MUL;
		*target_y = Map_GetYCount() + (y - positions[0].y) * CELL_COUNT_MUL;
#endif
	}

#if defined(SHORTEST_PATH_V2) && defined (SHORTEST_PATH_V2_RAM)

	/* If dynamic memory is used, print the memory usage. */
	USPRINTF("Memory usage: target: %d(%d * %d)\tline: %d(%d * %d)\n",
			target_list_cnt * sizeof(TargetType), sizeof(TargetType), target_list_cnt,
			path_line_get_count() * sizeof(LineType), sizeof(LineType), path_line_get_count());
#endif

	USPRINTF("path_next\tx: %d(%d)\ty: %d(%d)\t next dest: %d\tx: %d(%d)\ty: %d(%d)\n",
		positions[0].x, Map_GetXCount(), positions[0].y, Map_GetYCount(), val, countToCell(*target_x), *target_x, countToCell(*target_y), *target_y);

	return val;
}

/*
 * Function for the robot to go back to its starting point. If its
 * starting point is blocked or somehow not accessible, it will try
 * to find a nearest point to the starting point and move to it,
 * but such point is limited to (home_x -/+ 20, home_y -/+ 20)
 *
 * @param *target_x	Pointer to target position's X coordinate
 * @param *target_y	Pointer to target position's Y coordinate
 *
 * @return	0 if robot reaches its home point or excess max home try count,
 * 		other wise the cost to next stop point
 */
//uint8_t path_home(int32_t *target_x, int32_t *target_y) {
//	uint8_t		stop;
//	int16_t		x, x_next, y, y_next, i, j, cost, offset, retval;
//	Point16_t	pos;

//	/* Clear the map blocks which path_home is called. */
//	if (home_try_cnt == 0) {
//		Map_ClearBlocks();
//		Map_Set_Cells(ROBOT_SIZE, countToCell(*target_x), countToCell(*target_y), CLEANED);
//		last_dir = path_get_robot_direction();
//	}

//	path_set_current_pos();

//	USPRINTF("path_home: current: (%d, %d) (%d, %d) \thome: (%d, %d)\tdir: %d\n",
//			positions[0].x, positions[0].y, Map_GetXCount(), Map_GetYCount(), countToCell(*target_x), countToCell(*target_y), last_dir);

//#if DEBUG_MAP
//	/* If the flag DEBUG_MAP is set, print the map for debugging. */
//	debug_map(MAP, countToCell(*target_x), countToCell(*target_y));
//#endif
//	cost = -1;
//	offset = retval = 0;
//	x_next = countToCell(*target_x);
//	y_next = countToCell(*target_y);

//	/* Stop only when the robot reaches the starting point. */
//	if (!(x_next == positions[0].x && y_next == positions[0].y)) {
//		stop = 0;
//		while (!stop) {
//			 /* Continuously find the suitable point that closest to the robot's starting point, with the maximum offset 20. */
//			for (i = countToCell(*target_x) - offset; (i <= countToCell(*target_x) + offset) && (i <= map_size); i += 2) {
//				for (j = countToCell(*target_y) - offset; (j <= countToCell(*target_y) + offset) && (j <= map_size); j += 2) {
//					if (!(i == (countToCell(*target_x) - offset) || i == (countToCell(*target_x) + offset) || j == (countToCell(*target_y) - offset) || j == (countToCell(*target_y) + offset)))
//						continue;

//					if (Map_GetCell(MAP, i, j) == UNCLEAN || (is_a_block(i, j) == 1))
//						continue;

//					/* If unreachable from current position, clear the blocks. */
//					if (is_block_accessible(i, j) == 0) {
//						//Map_ClearBlocks();
//						continue;
//					}

//					/* If no path found, clear the blocks. */
//					if (path_find_shortest_path(positions[0].x, positions[0].y, i, j, 0, last_dir) < 0) {
//						//Map_ClearBlocks();
//						continue;
//					}
//					USPRINTF("%s %d: %d %d\n", __FUNCTION__, __LINE__, cost, abs(i - countToCell(*target_x)) + abs(j - countToCell(*target_y)));
//					if (cost == -1 || cost > (abs(i - countToCell(*target_x)) + abs(j - countToCell(*target_y)))) {
//						cost = abs(i- countToCell(*target_x)) + abs(j - countToCell(*target_y));
//						x_next = i;
//						y_next = j;
//					}
//				}
//			}

//			if (cost == -1 || (offset != 0 && offset <= cost)) {
//				if (offset < map_size && (abs(positions[0].x) + abs(positions[0].y) > offset)) {
//					offset += 2;
//					/* If over the offset limit, stop. */
//					if (offset > 10) {
//						USPRINTF("%s %d: stop robot, offset is too large, home (%d, %d)\n", __FUNCTION__, __LINE__, x_next, y_next);
//						retval = 0;
//						*target_x = Map_GetXCount();
//						*target_y = Map_GetYCount();
//					}
//				} else {
//					USPRINTF("%s %d: stop robot, no path to home (%d, %d)\n", __FUNCTION__, __LINE__, x_next, y_next);
//					retval = 0;
//					*target_x = Map_GetXCount();
//					*target_y = Map_GetYCount();
//					stop = 1;
//				}
//			} else {
//				/* If no path find anymore, stop. */
//				pos.X = Map_GetXPos();
//				pos.Y = Map_GetYPos();
//				if ((retval = path_move_to_unclean_area(pos, x_next, y_next, &x, &y, last_dir)) <= 0) {
//					USPRINTF("%s %d: stop robot, no path to home (%d, %d)\n", __FUNCTION__, __LINE__, x_next, y_next);
//					retval = 0;
//					*target_x = Map_GetXCount();
//					*target_y = Map_GetYCount();
//				} else {
//					CM_SetHome(cellToCount(x_next), cellToCount(y_next));
//#ifdef PP_MOVE_TO_CELL_CENTER
//					/*
//					 * If the flag PP_MOVE_TO_CELL_CENTER is set, force the robot to move to
//					 * the center of a cell for each action, otherwise, just use the encoder
//					 * counter plus the offsets.
//					 */
//					*target_x = (x == positions[0].x) ? Map_GetXCount() : cellToCount(x);
//					*target_y = (y == positions[0].y) ? Map_GetYCount() : cellToCount(y);
//#else
//					*target_x = Map_GetXCount() + (x - positions[0].x) * CELL_COUNT_MUL;
//					*target_y = Map_GetYCount() + (y - positions[0].y) * CELL_COUNT_MUL;
//#endif
//				}
//				stop = 1;
//			}
//		}
//	}

//	/* If the robot movement is only 1 cells, increase the try count. */
//	if (((abs(positions[0].x - positions[1].x) <= 1) && (abs(positions[0].y - positions[1].y) <= 1) && last_dir == path_get_robot_direction())) {
//		home_try_cnt++;
//	}

//	/* If the try count over 5, stop. */
//	if (home_try_cnt > 5) {
//		retval = 0;
//		*target_x = Map_GetXCount();
//		*target_y = Map_GetYCount();

//#if defined(SHORTEST_PATH_V2) && defined (SHORTEST_PATH_V2_RAM)

//		/* If dynamic memory is used, free the memory. */
//		if (targetList != NULL) {
//			memset(targetList, 0, sizeof(TargetType) * target_list_cnt);
//			free(targetList);
//			targetList = NULL;
//		}
//#endif
//	}

//	USPRINTF("home next dest:\t%d\tx: %d(%d)\ty: %d(%d)\tcnt: %d\n", retval, countToCell(*target_x), *target_x, countToCell(*target_y), *target_y, home_try_cnt);
//	last_dir = path_get_robot_direction();

//	return retval;
//}

void path_escape_set_trapped_cell( Point16_t *cell, uint8_t size ) {
	uint8_t i = 0;
	trappedCellSize = size;
	for ( i = 0; i < trappedCellSize; ++i ) {
		trappedCell[i] = cell[i];
		USPRINTF("%s %d Set %d trapped reference cell: x: %d\ty:%d\n", __FUNCTION__, __LINE__,
	           i, trappedCell[i].X, trappedCell[i].Y);
	}
}

Point16_t *path_escape_get_trapped_cell() {
	return trappedCell;
}

int16_t path_get_home_x() {
	return home_x;
}

int16_t path_get_home_y() {
	return home_y;
}
