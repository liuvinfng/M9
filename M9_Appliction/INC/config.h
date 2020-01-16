#ifndef __CONFIG_H__
#define __CONFIG_H__

/* ------------------------------------- System Setting ------------------------------------- */
//#define SPK_CHINA
//#define SPK_ENGLISH
//#define SPK_NORMAL
/*
 * Chipset setting.
 */
//#define CPU_CHIP			(91)

/*
 * Build system, CPU ID checking will be different for building on Linux or Window.
 */
//#define BUILD_ON_LINUX			(0)


//#define STR_TURN      (1)//edit by vin

//#define NEW_CLIFF     (1) //did not use 
/* ------------------------------------- Path Planning ------------------------------------- */

/*
 * Definition for enable debug or not.
 */
//



#define ENABLE_DEBUG					(1)
#define USART_PRINT_ENABLE    (1)
#define DEBUG_MAP			    		(1)


/*
 * Enable debugging the grid map.
// */

/*
*Disable debugging the boundary of the map,avoiding system crash when debug the boundary 
*/
//#define NOT_DEBUG_BOUNDARY			(1)
/*
 * Enable debugging the shortest path map when finding the
 * shortest path is using the A* like method.
 */
//#define DEBUG_SM_MAP			(1)

/*
 * Enable target debug.
 */
//#define DEBUG_TARGETS			(1)

/*
 * Definition to enable/disable robot rounding the obstcal or not.
 */
//#define PP_ROUNDING_OBSTCAL		(1)

/*
 * Definition to force robot to move the center of a cell.
 *
 * If not defined, when moving to a cell, it will just use the
 * curent positions of X or Y encoder count only.
 */
#define PP_MOVE_TO_CELL_CENTER		(1)

/* Zig-Zag cleanning time 120 minutes */
#define CLEANNING_TIME			(14400)

/* With water tank, Zig-Zag cleanning time 90 minutes */
#define WET_CLEANNING_TIME		(10800)

/* Low battery go home voltage value */
#define LOW_BATTERY_GO_HOME_VOLTAGE		(1300)

/* OBS setting */
#define OBS_DYNAMIC			(1)
#define WALL_DYNAMIC    (1)

/*
 * If defined, enable virtual wall.
 */
#define VIRTUAL_WALL			(1)

/*
 * Defines for maximum distance between 2 virtual wall points,
 * If less than this value, points between these two points will be
 * marked as unaccessible.
 */
#define VIRTUAL_WALL_MAX_SEGMENT_DISTANCE	(6)

#define OBS_DYNAMIC_MOVETOTARGET (1)
/* Bumper and Cliff Error */
#define CLIFF_ERROR    (1)
#define BUMPER_ERROR   (1)

/*
 * Definition relates to the robot.
 */

/*
 * If ROBOT_SIZE equals to 5, robot is defined as occupying 25(5x5) cells.
 * If it equals to 3, robot is defined as occupying 9(3x3) cells
 */
#define ROBOT_SIZE			          (3)

#define ROBOT_HEAD_OFFSET		      (2)
#define ROBOT_LEFT_OFFSET		      (1)
#define ROBOT_RIGHT_OFFSET		    (-1)

#define ROBOT_BRUSH_LENGTH		    (1)
#define ROBOT_BRUSH_LEFT_OFFSET		(0)
#define ROBOT_BRUSH_RIGHT_OFFSET	(0)

#define SLOW_DOWN_DISTANCE		(CELL_COUNT_MUL)



/*
 * Total number of targets that can be allowed..
 */
#define TARGET_TOTAL_COUNT		(MAP_SIZE * 5 / 2)

/* ------------------------------------- Shortest Path ------------------------------------- */

/*
 * Enable debugging the shortest path when using line-segments
 * strategy.
 */
#define DEBUG_POS			(1)

/*
 * Defines for when strategy to be used for shortest path. If
 * defines SHORTEST_PATH_V2, line-segment strategy will be use,
 * otherwise, it will use A* like strategy.
 */
#define SHORTEST_PATH_V2		(1)

/*
 * When defines SHORTEST_PATH_V2_RAM, line-segment & targets will
 * be saved by using dymanic memory allocation, but that is not
 * statble currently.
 */
//#define SHORTEST_PATH_V2_RAM		(1)

/*
 * Total number of lines that the shorestpath will search for.
 */
#define POS_LINE_CNT			(300)

#define POS_POINT_CNT			(300)

#define Path_POINT_CNT		(300)

/* ------------------------------------- Path Planning Wall Follow ------------------------------------- */

/*
 * If defined DISABLE_WALL_ALIGN, robot will disable wall
 * alignment when it starts, otherwise, it when it starts,
 * it will move for a short distance and align the Gyro angle
 * with the wall.
 */
#define DISABLE_WALL_ALIGN		(1)

/*
 * If defined DISABLE_WALL_ALIGN_SPOT_MODE, remote spot key even won't be handled.
 */
#define DISABLE_WALL_ALIGN_SPOT_MODE	(1)

/*
 * Define find wall method
 */
//#define FIND_WALL_ANGLE_METHOD_1		(1)
#define FIND_WALL_ANGLE_METHOD_2		(1)

/* 5 meters for finding wall, align the starting angle for Gyro */
#define MAP_FIND_WALL_DISTANCE		(67 * 4)

/* Escape time set to 9 minutes */
#define ESCAPE_TRAPPED_TIME		(1080)

/* Set trapped reference target size for robot to check that if it is trapped */
#define ESCAPE_TRAPPED_REF_CELL_SIZE		(3)

/* Set maximum bumper count of complicated area */
#define COMPLICATED_AREA_BUMPER_MAX_COUNT		(10)

/* ------------------------------------- Path Planning Gyro ------------------------------------- */

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
#define GYRO_CALIBRATION             (1)

/*
 * Defines for enabling/disabling tilted detect or not.
 * When detecting the robot is tilted, it will check the all the 3 cliff
 * sensors value, which if all are less then 1500, and at the same time,
 * if the gyro X-axis & Y-axis angle are greater than 5 degree, we will
 * confirmed that the robot is tilted.
 */
//#define ENABLE_TILTED_DETECT		(1)

/*
 * Value for maximum allowed angle while the robot is tilted. If it is greater
 * than this angle, the robot is tilted.
 */
#define TILTED_ANGLE_LIMIT		(75)

/*
 * Value for lower limit of all the 3 cliff sensors, if all those 3 sensors have
 * values lower than this value, the robot is tilted.
 */
#define TILTED_CLIFF_LIMIT		(1500)

/* ------------------------------------- Core Move ------------------------------------- */

/*
 * Distance between left & right wheel in mm.
 */
#define WHEEL_BASE			(201)



/*
 * Go home using CM_MoveToCell function
 */
#define GO_HOME_METHOD_2		(1)

/*
 * How to add targets in zone, if defined, will add targets by using wall
 * follow path, otherwise, will just add radomly.
 */
#define ADD_TARGET_BY_PATH		    (1)
#define ADD_TARGET_BY_PATH_ONLY		(1)

/* ------------------------------------- Path Planning Map ------------------------------------- */

/*
 * Definition of the grid map.
 */
#define MAP_DIMENSION			250//14420//(13596)//(14420)//CELL_SIZE * 190 //9225//7925  // 14250
#define MAP_SIZE			    (MAP_DIMENSION / CELL_SIZE)
#define MAP_LIMIT_LOW	    (-MAP_SIZE/2+1)
#define MAP_LIMIT_HIGH	  (MAP_SIZE/2-1)

#define TARGET_LIST_AMOUNT 100

/*
 * Definition relates to a grid cell.
 */

/*robot size  =3 */
/*
	robot size = 300mm
	robot step/mm = 1602(counts) / 204(mm)
	robot occupy 3x3 cells
	cells size = 786 (counts)
*/
#define CELL_SIZE			  (1)
#define CELL_SIZE_2			(2 * CELL_SIZE) 
#define CELL_SIZE_3			(3 * CELL_SIZE) 
#define CELL_COUNT_MUL				760 
#define CELL_COUNT_MUL_1_2		CELL_COUNT_MUL/2//(415)  
#define CELL_COUNT_MUL_3_2		CELL_COUNT_MUL_1_2*3//(1245) 
#define CELL_COUNT_MUL_2			CELL_COUNT_MUL*2//(1660) 
#define CELL_COUNT_MUL_1_4		CELL_COUNT_MUL/4//(207)
#define CELL_COUNT_MUL_7_4		CELL_COUNT_MUL/4*7

/* ------------------------------------- speed ------------------------------------- */
#define MUL_PWM			          1
#define MUL			              3/2
#define SPEED_DIV			        (int8_t)8*MUL
#define ROTATE_TOP_SPEED			(int8_t)26*MUL
#define BASE_SPEED				    (int8_t)15*MUL
#define RUN_SLOW_SPEED				(int8_t)15*MUL
#define RUN_SLOW_OBS				  (int8_t)10*MUL
#define RUN_TOP_SPEED					(int8_t)35*MUL
#define RUN_TOP_SPEED_U				(int8_t)30*MUL
#define RUN_ADJUST_TOP_SPEED	(int8_t)40*MUL
#define RUN_SIDE_BACK_SPEED		(int8_t) 5*MUL

#define MAX_SPEED 						(uint8_t)35*MUL
#define BACK_SPEED						(uint8_t)20*MUL
#define ZERO_SPEED						(uint8_t)0
#define WALL_WALK_SPEED				(uint8_t)25*MUL
#define WALL_ADJUST_TO_SPEED	(uint8_t)30*MUL
#define TURN_SPEED            (uint8_t)25*MUL

#define MAX_DISTANCE 					(uint32_t)0x1ffffffe

#define SPEED_1M_MIN         31/10  //1600/24/18 
#define DISTANCE_1CM         76

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
/* ------------------------------------- MOUSE ------------------------------------- */


//#define FR5801   (1)

#endif
