#ifndef __CONFIG_H__
#define __CONFIG_H__

/* ------------------------------------- System Function Setting ------------------------------------- */

/*
 * Chipset setting.
 */
#define CPU_CHIP			(91)

/*
 * Build system, CPU ID checking will be different for building on Linux or Window.
 */
#define BUILD_ON_LINUX			(0)

/*
 * Definition for enable debug or not.
 */
#define ENABLE_DEBUG			   (1)
#define DEBUG_PC_MAP         (1)
#define USART_PRINT_ENABLE   (1)
/*
 * Enable debugging the grid map.
 */
//#define DEBUG_MAP			(1)

/*
 * Definition to enable/disable robot rounding the obstcal or not.
 */
#define PP_ROUNDING_OBSTCAL		(1)

/*
 * Definition to enable/disable align with the when when robot is rounding the obstcal.
 * Default disabled.
 */
//#define PP_ROUNDING_ALIGNMENT		(1)

/*
 * Definition to force robot to move the center of a cell.
 *
 * If not defined, when moving to a cell, it will just use the
 * curent positions of X or Y encoder count only.
 */
#define PP_MOVE_TO_CELL_CENTER		(1)

/* OBS setting */
#define OBS_DYNAMIC			(1)
#define WALL_DYNAMIC    (1)

/* Bumper and Cliff Error */
#define CLIFF_ERROR    (1)
#define BUMPER_ERROR   (1)
#define BUMPER_JAMED   (1)
/*
 * If defined, enable virtual wall.
 */
#define VIRTUAL_WALL			(1)

/*
 * Enable or disable zone cleaning mode.
 */
#define ZONE_WALLFOLLOW		(1)

/* ----------- Alignment ------- */
/*
 * Enable alignment
 */
#define ALIGNMENT_ENABLE			(1)

/*
 * Define that the alignment will use the wall angle to directly alignment gyro angle
 */
#define ALIGNMENT_ANGLE_WALL_ENABLE		(1)

#endif

/*------ WIFI Module -----------*/

//#define WIFI_EMW3081				(1)



/*----- Path Planning Gyro ----- */

/*
 * Define which Gyro will be use.
 * If defined GYRO_XV7011, it will use XV70XX from Espon.
 * If not defined, it will use RN13XX from MicroInfinity
 */
#define GYRO_XV7011			(1)

/*
 * Define for enabling/disabling Gyro realtime calibration.
 * Default is disabled.
 */
#define GYRO_CALIBRATION		(1)

/* ------------------------------------- System Parameter Setting ------------------------------------- */

/* Zig-Zag cleanning time 120 minutes */
#define CLEANNING_TIME			(14400)

/* With water tank, Zig-Zag cleanning time 90 minutes */
#define WET_CLEANNING_TIME		(10800)

/* Low battery go home voltage value */
#define LOW_BATTERY_GO_HOME_VOLTAGE		(1360)

/*
 * Defines for maximum distance between 2 virtual wall points,
 * If less than this value, points between these two points will be
 * marked as unaccessible.
 */
#define VIRTUAL_WALL_MAX_SEGMENT_DISTANCE	(6)

/*
 * Definition relates to the robot.
 */

/* 
 * robot is defined as occupying 9(3x3) cells
 */
#define ROBOT_SIZE			(3)

#define ROBOT_HEAD_OFFSET		(2)
#define ROBOT_LEFT_OFFSET		(1)
#define ROBOT_RIGHT_OFFSET		(-1)

#define ROBOT_BRUSH_LENGTH		(1)
#define ROBOT_BRUSH_LEFT_OFFSET		(0)
#define ROBOT_BRUSH_RIGHT_OFFSET	(0)

#define SLOW_DOWN_DISTANCE		(2*CELL_COUNT_MUL)

/*
 * Total number of targets that can be allowed..
 */
#define TARGET_TOTAL_COUNT		(MAP_SIZE * 5 / 2)

/* ------------ Shortest Path -------------------- */
/*
 * Total number of lines that the shorestpath will search for.
 */
#define POS_LINE_CNT			(500)

/* -------------- Path Planning Wall Follow ------ */

/* Escape time set to 9 minutes */
#define ESCAPE_TRAPPED_TIME		(1080)
#define ESCAPE_TRAPPED_TIME_ZZ (480)
/* 5 meters for finding wall, align the starting angle for Gyro */
#define MAP_FIND_WALL_DISTANCE		(67 * 4)

/* Escape time set to 9 minutes */
#define ESCAPE_TRAPPED_TIME		(1080)

/* Set trapped reference target size for robot to check that if it is trapped */
#define ESCAPE_TRAPPED_REF_CELL_SIZE		(3)

/* Set maximum bumper count of complicated area */
#define COMPLICATED_AREA_BUMPER_MAX_COUNT		(10)

/* ------------ Core Move --------------- */

/*
 * Distance between left & right wheel in mm.
 */
#define WHEEL_BASE			    (201)

/*count:1600 C:18cm*/
#define ANGLE_MUL            8/5
//#define SPEED_MUL            3/2
#define SPEED_1M_MIN         31/10  //1600/24/18 
#define DISTANCE_1CM         76
#define TURN_AROUND_CNT      7220
#define PWM_OUT              1
/*
 * Range of BASE_SPEED should be better within 12 to 15.
 * When it is too small, it will move like shaking when robot startup.
 * When it is too large, it will fall down when reach the cliff.
 */
#define Max_Speed             (15*SPEED_1M_MIN)
#define BASE_SPEED						(6*SPEED_1M_MIN)
#define Turn_Speed            (7*SPEED_1M_MIN)
#define TURN_SPEED	          (7*SPEED_1M_MIN)
#define ROTATE_TOP_SPEED_10		(7*SPEED_1M_MIN)

#define RUN_SPEED_17	        (15*SPEED_1M_MIN)
#define RUN_SPEED_16	        (15*SPEED_1M_MIN)
#define RUN_SPEED_15	        (15*SPEED_1M_MIN)
#define RUN_SPEED_14	        (14*SPEED_1M_MIN)
#define RUN_SPEED_13	        (13*SPEED_1M_MIN)
#define RUN_SPEED_12	        (12*SPEED_1M_MIN)
#define RUN_SPEED_11	        (11*SPEED_1M_MIN)
#define RUN_SPEED_10	        (10*SPEED_1M_MIN)
#define RUN_SPEED_9	          (9*SPEED_1M_MIN)
#define RUN_SPEED_8	          (8*SPEED_1M_MIN)
#define RUN_SPEED_7	          (7*SPEED_1M_MIN)
#define RUN_SPEED_6	          (6*SPEED_1M_MIN)
#define RUN_SPEED_5			      (5*SPEED_1M_MIN)
#define RUN_SPEED_4	          (4*SPEED_1M_MIN)
#define RUN_SPEED_3	          (3*SPEED_1M_MIN)
#define RUN_SPEED_2	          (2*SPEED_1M_MIN)
#define RUN_SPEED_1	          (1*SPEED_1M_MIN)
/*
 * Go home using CM_MoveToCell function
 */
#define GO_HOME_METHOD_2		(1)

/* ---------- Path Planning Map ---------- */

/*
 * Definition of the grid map.
 */
#define MAP_DIMENSION			CELL_SIZE * 185// 180//132 //9225//7925  // 14250
#define MAP_SIZE					(MAP_DIMENSION / CELL_SIZE)
#define MAP_SINGLE_SIZE		52
/*
 * Definition relates to a grid cell.
 */

#define CELL_SIZE						(103)
#define CELL_COUNT_MUL			(520)
#define CELL_COUNT_MUL_1_2	(260)


#define CELL_SIZE_2			(2 * CELL_SIZE)
#define CELL_SIZE_3			(3 * CELL_SIZE)

/* --------- Zone Wall Follow ------ */

/*
 * Zone definitions.
 */

#define ZONE_DIMENSION			(CELL_SIZE * ZONE_SIZE)//3090  //1430  // 910  // 1950
#define ZONE_SIZE						(30)
#define ZONE_SIZE_HALF			(ZONE_SIZE / 2)

/*
 * Zone size of keeping obstacals
 */
#define ZONE_KEEP_OBSTACALS_SIZE	(2)

/*
 * Number of maximum zone allowed.
 */
#define ZONE_MAX			((MAP_DIMENSION * MAP_DIMENSION / ZONE_DIMENSION / ZONE_DIMENSION) * 4)

/*
 * Maximum number of zone allowed, if greater than this count, force go home.
 */
#define FROCE_GO_HOME_ZONE_CNT		(25)

/*
 * Boundary increment, extend the boundary with this value
 */
#define BOUNDARY_INCREMENT			(3)

/* --------- Defines about how to stop when cleaning zone mode. ---- */

/*
 * Maximum distance for method 1 which will cause the robot to stop cleaning.
 * The value is the distance between start point of current zone and start point of
 * zone 1, 2, or 3. Also is for the end point to end point of the zones for matching.
 *
 * This value is calculated in cell size.
 */
#define STOP_WALL_FOLLOW_M1_MAX_DISTANCE	(7)

/*
 * Check zone size of method 1
 */
#define STOP_WALL_FOLLOW_M1_CHECK_ZONE_SIZE		(3)
#define STOP_WALL_FOLLOW_M3_CHECK_ZONE_SIZE		(3)


