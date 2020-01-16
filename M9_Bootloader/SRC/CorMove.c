#include <math.h>
#include "CorMove.h"
#include "Display.h"
#include "Home.h"
#include "Movement.h"
#include "Rcon.h"
#include "ShortestPath.h"
#include "Speaker.h"
#include "Spot.h"
#include "TouchPad.h"
#include "USART.h"
#include "Gyro.h"
#include "Map.h"
#include "MyMath.h"
#include "PathPlanning.h"
#include "WallFollowMulti.h"
#include "Zone.h"
#include "PP_rounding.h"

#define MOVE_TO_CELL_SEARCH_INCREMENT 2
#define MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH 10
#define MOVE_TO_CELL_SEARCH_ARRAY_LENGTH (2 * MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH / MOVE_TO_CELL_SEARCH_INCREMENT + 1)
#define MOVE_TO_CELL_SEARCH_ARRAY_LENGTH_MID_IDX ((MOVE_TO_CELL_SEARCH_ARRAY_LENGTH * MOVE_TO_CELL_SEARCH_ARRAY_LENGTH - 1) / 2)

typedef enum {
	ACTION_NONE	= 0x01,
	ACTION_GO	  = 0x02,
	ACTION_BACK	= 0x04,
	ACTION_LT	  = 0x08,
	ACTION_RT	  = 0x10,
} ActionType;


extern volatile int16_t Temp_Right_Wheel_PWM, Temp_Left_Wheel_PWM;
extern uint32_t wall_follow_to_zone_cell_timer;

extern uint16_t map_size;
extern PositionType positions[];
extern uint32_t wall_follow_to_zone_exit_timer;
extern volatile uint32_t Work_Timer;
extern int16_t xMin, xMax, yMin, yMax;

extern uint16_t moveToPointTimeCount;
extern volatile uint8_t cleaning_mode;
extern uint32_t escape_trapped_timer_ZZ;

Point32_t	g_home_point, g_charge_point;

uint8_t g_map_touring_cancel = 0,wheelspd=0;
uint8_t BlockAroundChargerFlag;

uint8_t LED_Blink = 0, LED_Blink_State = 0;
uint8_t	go_home = 0;
uint8_t	remote_go_home = 0;
uint8_t	from_station = 0;
int16_t station_zone = -1;
int16_t WheelCount_Left = 0, WheelCount_Right = 0;
uint8_t lowBattery = 0;
int16_t map_gyro_offset = 0;
uint8_t tiledUpCount = 0;
uint8_t isSingleRoom = 0;

uint8_t	should_follow_wall = 0;



Point16_t path[ZONE_SIZE * 8] = {{0, 0}};
Point16_t path_blocks[ZONE_SIZE * 8] = {{0, 0}};
volatile int16_t path_blocks_angle[ZONE_SIZE * 8]={0};
volatile uint16_t path_blocks_size=0;

Point16_t relativePos[MOVE_TO_CELL_SEARCH_ARRAY_LENGTH * MOVE_TO_CELL_SEARCH_ARRAY_LENGTH] = {{0, 0}};

int16_t xMinSearch, xMaxSearch, yMinSearch, yMaxSearch;

#ifdef ZONE_WALLFOLLOW
extern Point32_t positionAlignment[];
extern uint16_t angleAlignment[];
extern uint16_t alignmentPtr;
#endif

#ifdef VIRTUAL_WALL
uint8_t vw_pos_cnt = 0;
VWType vw_pos[50] = {{{0, 0}}};
Point32_t pos_cur[5] = {{0, 0}};
extern volatile uint8_t WallFollowMulti_End_VirtualWall;
extern volatile uint8_t WallFollowMulti_Pass_VirtualWall;
extern  Point32_t Start_WallFollowMulti_VirtualWall_Point,End_WallFollowMulti_VirtualWall_Point;
volatile uint8_t VirtualWall_Path_Error=0;
#endif

volatile uint8_t station_position=0;

extern volatile uint8_t CorMove_Rcon;

volatile uint8_t CM_MoveForward_Ignore_Rcon=0;
volatile uint8_t WallFollow_Point_Exit=0;

extern volatile Point16_t ShortestPath_Last_Cell;
extern volatile uint8_t ShortestPath_Time_Over;

extern uint8_t WallFollowARoundFlag;


extern volatile uint16_t Zone_Run_Time;

Point32_t MapOffset = {0,0};
uint8_t IsMovingFlag = 0;
uint8_t GoToTempTargetFlag = 0;							//0->Not go;	1->Arriving		2->Arrived	3->interrupt
uint8_t SpecialPIDFlag = 0;
int32_t Base_Speed = BASE_SPEED ;
Point32_t StartPointToTempTarget;
uint16_t StartAngleToTempTarget;
uint32_t StartTimeToTempTarget;
uint8_t BumperTemp;
uint8_t CliffTemp;
uint32_t RconTemp;
Point32_t TempTarget;
uint8_t PP_RoundingMovingFlag = 0;
Point16_t g_LastPosition = {0,0};
uint16_t g_LastAngle = 0;
uint8_t g_SamePositionCnt = 0;

void CM_count_normalize(uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y)
{
	*x = cellToCount(countToCell(Map_GetRelativeX(heading, offset_lat, offset_long)));
	*y = cellToCount(countToCell(Map_GetRelativeY(heading, offset_lat, offset_long)));
}

/*--------------------------------- Edit By ZZ ----------------------------*/
void CM_count_normalize_ByXYCount(int32_t XCount, int32_t YCount, uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y)
{
	*x = cellToCount(countToCell(Map_GetRelativeX_ByXCount(XCount,heading, offset_lat, offset_long)));
	*y = cellToCount(countToCell(Map_GetRelativeY_ByYCount(YCount,heading, offset_lat, offset_long)));
}

void ResetMapOffset(void)
{
	if((MapOffset.X != 0) || (MapOffset.Y != 0))
	{
		Map_MoveTo(-MapOffset.X,-MapOffset.Y);
		MapOffset.X = MapOffset.Y = 0;
	}
}

/***************************************************************************/
int32_t CM_ABS(int32_t A, int32_t B)
{
	return ((A > B) ? (A - B) : (B - A));
}

void CM_update_position(uint16_t heading_0, int16_t heading_1, int32_t left, int32_t right) 
{
	double	dd,x_tmp,y_tmp;
	int16_t c, d, x, y, path_heading;
	int32_t i, j;

	if (left == 0 && right == 0) 
	{
		return;
	}

	if (heading_0 > heading_1 && heading_0 - heading_1 > 1800) {
		path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
	} else if (heading_1 > heading_0 && heading_1 - heading_0 > 1800) {
		path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
	} else {
		path_heading = (uint16_t)(heading_0 + heading_1) >> 1;
	}

	WheelCount_Left -= left;
	WheelCount_Right -= right;
		/*********************************************/

	dd = left + right;
	dd /= 2;

	x = countToCell(Map_GetXCount() - MapOffset.X);
	y = countToCell(Map_GetYCount() - MapOffset.Y);
	
	x_tmp = dd * cos(deg2rad(path_heading, 10));
	y_tmp = dd * sin(deg2rad(path_heading, 10));
	
	if((MapOffset.X != 0) || (MapOffset.Y != 0))
	{
		MapOffset.X -= (int32_t)x_tmp;
		MapOffset.Y -= (int32_t)y_tmp;
	}
	else
	Map_MoveTo(x_tmp, y_tmp);
	
	if (x != countToCell(Map_GetXCount() - MapOffset.X) || y != countToCell(Map_GetYCount() - MapOffset.Y))
	{
		for (c = 1; c >= -1; --c) {
			for (d = 1; d >= -1; --d) {
				CM_count_normalize_ByXYCount((Map_GetXCount() - MapOffset.X),(Map_GetYCount() - MapOffset.Y),path_heading, CELL_SIZE * c, CELL_SIZE * d, &i, &j);
				if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BOUNDARY) {
					Map_SetCell(MAP, i, j, CLEANED);
				}
			}
		}
	}

#if 0
		if (Get_LWall_ADC() > 380) {
			CM_count_normalize_ByXYCount((Map_GetXCount() - MapOffset.X),(Map_GetYCount() - MapOffset.Y),heading_0, CELL_SIZE_2, CELL_SIZE, &i, &j);
			if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, i, j, BLOCKED);
			}
		}
		if (Get_RWall_ADC() > 380) {
			CM_count_normalize_ByXYCount((Map_GetXCount() - MapOffset.X),(Map_GetYCount() - MapOffset.Y),heading_0, -CELL_SIZE_2, CELL_SIZE, &i, &j);
			if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, i, j, BLOCKED);
			}
		}
#endif
		
	/*-------------------- Edit By ZZ ------------------------*/
#if 0
	for (c = -1; c <= 1; c++)
	{
		for (d = -1; d <= 1; d++) 
		{
			if(abs(c)!=abs(d))
			{
				if(Map_GetCell(MAP,x+c,y+d) == BLOCKED)
				{
					if(c == 0)
					{
						Map_SetCell(MAP,cellToCount(x-1),cellToCount(y+d*2),Map_GetCell(MAP,x+c,y+d));
						Map_SetCell(MAP,cellToCount(x),cellToCount(y+d*2),Map_GetCell(MAP,x+c,y+d));
						Map_SetCell(MAP,cellToCount(x+1),cellToCount(y+d*2),Map_GetCell(MAP,x+c,y+d));
					}
					else
					{
						Map_SetCell(MAP,cellToCount(x+c*2),cellToCount(y-1),Map_GetCell(MAP,x+c,y+d));
						Map_SetCell(MAP,cellToCount(x+c*2),cellToCount(y),Map_GetCell(MAP,x+c,y+d));
						Map_SetCell(MAP,cellToCount(x+c*2),cellToCount(y+1),Map_GetCell(MAP,x+c,y+d));
					}
				}
			}
			else if((abs(c)==abs(d)) && (c!=0))
			{
				if(Map_GetCell(MAP,x+c,y+d) == BLOCKED)
				{
					Map_SetCell(MAP,cellToCount(x+c*2),cellToCount(y+d),Map_GetCell(MAP,x+c,y+d));
					Map_SetCell(MAP,cellToCount(x+c),cellToCount(y+d*2),Map_GetCell(MAP,x+c,y+d));
				}
			}
			if(Map_GetCell(MAP,x+c,y+d) != BLOCKED_BOUNDARY)
				Map_SetCell(MAP, cellToCount(x+c), cellToCount(y+d), CLEANED);
		}
	}
#endif
}

void CM_update_map_bumper(ActionType action, uint8_t bumper)
{
	int16_t	c;
	int32_t	x_tmp, y_tmp;
	
	OBS_ON();

	if ((bumper & RightBumperTrig) && (bumper & LeftBumperTrig)) {
		for (c = -1; c <= 1; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	} else if (bumper & LeftBumperTrig) {
		CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE_2, CELL_SIZE_2, &x_tmp, &y_tmp);
		USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

		if (action == ACTION_LT) {

		} else {
			CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE_2, CELL_SIZE, &x_tmp, &y_tmp);
			USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			if ((positions[0].x == positions[1].x) && (positions[0].y == positions[1].y) && (positions[0].dir == positions[1].dir) &&
			    (positions[0].x == positions[2].x) && (positions[0].y == positions[2].y) && (positions[0].dir == positions[2].dir)) {
				CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_2, &x_tmp, &y_tmp);
				USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
			}
		}
	} else if (bumper & RightBumperTrig) {
		CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE_2, CELL_SIZE_2, &x_tmp, &y_tmp);
		USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		if (action == ACTION_RT) {

		} else {
			CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE_2, CELL_SIZE, &x_tmp, &y_tmp);
			USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			if ((positions[0].x == positions[1].x) && (positions[0].y == positions[1].y) && (positions[0].dir == positions[1].dir) &&
			    (positions[0].x == positions[2].x) && (positions[0].y == positions[2].y) && (positions[0].dir == positions[2].dir)) {
				CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_2, &x_tmp, &y_tmp);
				USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
			}
		}
	}
}

void CM_update_map(ActionType action, uint8_t bumper) {
	int16_t	c;
	uint16_t i;
	int32_t	x_tmp, y_tmp;
	uint8_t temp_cliff=0;
	
	for (c = 0; c < 3; ++c) {
		i = SHRT_MAX;
		switch (c) {
			case 0:
				i = Cormove_Get_OBSStatus() & Status_Right_OBS;
				break;
			case 1:
				i = Cormove_Get_OBSStatus() & Status_Front_OBS;
				break;
			case 2:
				i = Cormove_Get_OBSStatus() & Status_Left_OBS;
				break;
		}

		CM_count_normalize(Gyro_GetAngle(0), (c - 1) * CELL_SIZE, CELL_SIZE_3, &x_tmp, &y_tmp);
		USPRINTF("%s %d: %d\n", __FUNCTION__, __LINE__, i);
		if (i) {
			CM_count_normalize(Gyro_GetAngle(0), (c - 1) * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			if (Map_GetCell(MAP, countToCell(x_tmp), countToCell(y_tmp)) != BLOCKED_BUMPER) {
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_OBS);
			}
		}
	}

	CM_update_map_bumper(action, bumper);
	temp_cliff = Get_Cliff_Trig();
	
	if (temp_cliff& Status_Cliff_Front) {
		for (c = -1; c <= 1; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
	if (temp_cliff & Status_Cliff_Left) {
		for (c = 1; c <= 2; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
	if (temp_cliff & Status_Cliff_Right) {
		for (c = -2; c <= -1; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
}
void CM_update_map_Cliff(uint8_t cliff)
{
	int8_t c=0;
	int32_t	x_tmp, y_tmp;
	
	if (cliff & Status_Cliff_Front) {
		for (c = -1; c <= 1; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
	if (cliff & Status_Cliff_Left) {
		for (c = 1; c <= 2; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
	if (cliff & Status_Cliff_Right) {
		for (c = -2; c <= -1; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
}

void CM_signed_obstacle(uint8_t obs_status_tmp,uint8_t bumper_status_tmp,uint8_t cliff_status_tmp) 
{
	int32_t	x_tmp, y_tmp;
	
	if ((obs_status_tmp & Status_Front_OBS) || ((bumper_status_tmp & LeftBumperTrig) && (bumper_status_tmp & RightBumperTrig)) || (cliff_status_tmp & Status_Cliff_Front)) 
	{
		CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_2, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED);
	}
	if ((obs_status_tmp & Status_Left_OBS) || (bumper_status_tmp & LeftBumperTrig) || (cliff_status_tmp & Status_Cliff_Left)) 
	{
		CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED);
	}
	if ((obs_status_tmp & Status_Right_OBS) || (bumper_status_tmp & RightBumperTrig) || (cliff_status_tmp & Status_Cliff_Right)) 
	{
		CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED);
	}
}
/*--------------------------Head Angle--------------------------------*/
void CM_HeadToCourse(uint8_t Speed, int16_t Angle)
{
	int16_t Diff = 0;
	uint32_t SpeedUp, Tick, turnning_time;
	ActionType action = ACTION_NONE;
	uint8_t isBumperTriggered;
	uint8_t Motor_Check_Code = 0;
	static int16_t	angle_turned = 0;
	
	Deceleration();
	
	SpeedUp = Tick = 0;

	turnning_time = Work_Timer;

	Diff = Angle - Gyro_GetAngle(0);

	USPRINTF("\n%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)\tBias: %d\tTemp: %d\tScale: %d\n",
	         __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(0), Diff, (Angle - Gyro_GetAngle(0)), Gyro_GetXAcc(), Gyro_GetYAcc(), Gyro_GetZAcc());

	while (Diff >= 1800) {
		Diff = Diff - 3600;
	}

	while (Diff <= (-1800)) {
		Diff = Diff + 3600;
	}

	if ((Diff < 10) && (Diff > (-10)))
	{
		return;
	}

	USPRINTF("\n%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)\tangle_turned: %d\tBias: %d\tTemp: %d\tScale: %d\n",
	         __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(0), Diff, (Angle - Gyro_GetAngle(0)), angle_turned, Gyro_GetXAcc(), Gyro_GetYAcc(), Gyro_GetZAcc());

	if ((Diff <= 1800 && Diff >= 1700) || (Diff >= -1800 && Diff <= -1700)) {
		if (Diff <= 1800 && Diff >= 1700) {
			if (angle_turned < 0) {
				USPRINTF("%s %d: Turn Left\n", __FUNCTION__, __LINE__);

				Set_Dir_Left();
				action = ACTION_LT;
				angle_turned += Diff;
			} else {
				USPRINTF("%s %d: Turn Right\n", __FUNCTION__, __LINE__);

				Set_Dir_Right();
				action = ACTION_RT;
				angle_turned += (Diff - 3600);
			}
		} else {
			if (angle_turned > 0) {
				USPRINTF("%s %d: Turn Right\n", __FUNCTION__, __LINE__);

				Set_Dir_Right();
				action = ACTION_RT;
				angle_turned += Diff;
			} else {
				USPRINTF("%s %d: Turn Left\n", __FUNCTION__, __LINE__);

				Set_Dir_Left();
				action = ACTION_LT;
				angle_turned += (3600 + Diff);
			}
		}
	} else {
		if ((Diff >= 0) && (Diff <= 1800)) {
			USPRINTF("%s %d: Turn Left\n", __FUNCTION__, __LINE__);

			Set_Dir_Left();
			action = ACTION_LT;
		} else if ((Diff <= 0) && (Diff >= (-1800))) {
			USPRINTF("%s %d: Turn Right\n", __FUNCTION__, __LINE__);

			Set_Dir_Right();
			action = ACTION_RT;
		}
		angle_turned += Diff;
	}

	SpeedUp = RUN_SPEED_3;
	while (1) {

		if (Work_Timer - turnning_time > 120) {
			Stop_Brifly();
			CM_TouringCancel();
			Set_Touch();
			cleaning_mode = Clean_Mode_Navigation;
			USPRINTF("%s %d: work timeout break!\n", __FUNCTION__, __LINE__);
			return;
		}

		Motor_Check_Code = Check_Motor_Current();
		if (Motor_Check_Code) {
			if (Self_Check(Motor_Check_Code)) {
				Stop_Brifly();
				CM_TouringCancel();
				Set_Touch();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				USPRINTF("%s %d: motor(s) error break!\n", __FUNCTION__, __LINE__);
				return;
			}
			Initialize_Motor();
			Stop_Brifly();
			return;
		}

		if (Touch_Detect()) {
			Set_Touch();
			CM_TouringCancel();
			Set_Clean_Mode(Clean_Mode_Userinterface);
			USPRINTF("%s %d: touch detect break!\n", __FUNCTION__, __LINE__);
			return;
		}
		#ifdef BLDC_INSTALL
		if (Remote_Key(Remote_Max)) {
			if (lowBattery == 0) {
				Switch_VacMode();
			}
		}
		#endif
		if (Remote_Key(Remote_Home) && go_home == 0) {
			Deceleration();
			Stop_Brifly();
			Set_BLDC_Speed(Vac_Speed_NormalL);
			Check_Bat_SetMotors(Home_Vac_Power, Home_SideBrush_Power, Home_MainBrush_Power);
			Set_Clean_Mode(Clean_Mode_GoHome);
			USPRINTF("%s %d: remote home is pressed.\n", __FUNCTION__, __LINE__);
			CM_SetGoHome(1);
			return;
		}

		if (Get_Cliff_Trig() == Status_Cliff_All) {
			USPRINTF("%s %d: robot is taken up break! \n", __FUNCTION__, __LINE__);
			Stop_Brifly();
			return;
		}

		isBumperTriggered = Get_Bumper_Status();
		if (isBumperTriggered) {
			Stop_Brifly();
			CM_update_map(action, isBumperTriggered);

			USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
			CM_CorBack(COR_BACK_50MM);
			Stop_Brifly();
			CM_update_map(action, isBumperTriggered);

			Reset_TempPWM();
			Set_Wheel_Speed(0, 0);

#ifdef BUMPER_ERROR
			if (Get_Bumper_Status()) {
				USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
				CM_CorBack(COR_BACK_50MM);
				if (Get_Bumper_Status()) {
					CM_CorBack(COR_BACK_50MM);
					if (Get_Bumper_Status()) {
						USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
						CM_CorBack(COR_BACK_50MM);
						Set_Error_Code(Error_Code_Bumper);
						Stop_Brifly();
						return;
					}
				}
			}
#endif
			
			Diff = Angle - Gyro_GetAngle(0);
			while (Diff >= 1800) {
				Diff = Diff - 3600;
			}

			while (Diff <= (-1800)) {
				Diff = Diff + 3600;
			}

			USPRINTF("\n%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)\n", __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(0), Diff, (Angle - Gyro_GetAngle(0)));
			if ((Diff >= 0) && (Diff <= 1800)) {
				USPRINTF("Turn Left\n");

				Set_Dir_Left();
				action = ACTION_LT;
			} else if ((Diff <= 0) && (Diff >= (-1800))) {
				USPRINTF("Turn Right\n");

				Set_Dir_Right();
				action = ACTION_RT;
			}
		}

		Diff = CM_ABS(Angle, Gyro_GetAngle(0));
		Diff = Diff > 1800 ? 3600 - Diff : Diff;
		if ((Diff < 10) && (Diff > (-10))) {
			Stop_Brifly();
			USPRINTF("%s %d: Angle: %d\tGyro: %d\tDiff: %d\n", __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(0), Diff);
			return;
		}

		Tick++;
		if (Tick > 20) {
			Tick = 0;
			if (Diff > 100) {
				SpeedUp += 1;
				SpeedUp = SpeedUp > Speed ? Speed : SpeedUp;
			} else {
				SpeedUp -= 1;
				SpeedUp = SpeedUp < RUN_SPEED_3 ? RUN_SPEED_3 : SpeedUp;
			}
		}
		Set_Wheel_Speed(SpeedUp, SpeedUp);

		delay(1);
	}
}

#ifdef VIRTUAL_WALL
void CM_VM_insert(uint32_t dir)
{
	dir = dir;
	if (vw_pos_cnt >= 50) {
		USPRINTF("%s %d: too many virtual wall points detected\n", __FUNCTION__, __LINE__);
		return;
	}

	if (vw_pos_cnt != 0 && vw_pos[vw_pos_cnt - 1].pos.X == Map_GetXPos() && vw_pos[vw_pos_cnt - 1].pos.Y == Map_GetYPos()) {
		USPRINTF("%s %d: same virtual wall points as added last (%d, %d)\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
		return;
	}

	vw_pos[vw_pos_cnt].pos.X = Map_GetXPos();
	vw_pos[vw_pos_cnt].pos.Y = Map_GetYPos();
	vw_pos_cnt++;
			
	USPRINTF("%s %d: vw_pos_cnt: %d\n", __FUNCTION__, __LINE__, vw_pos_cnt);
}

void CM_ProceVW(void)
{
	uint8_t i, j;
	int16_t x, y, start, end;
	float	slop, intercept;

	USPRINTF("%s %d: vw_pos_cnt: %d\n", __FUNCTION__, __LINE__, vw_pos_cnt);
	for (i = 0; i < vw_pos_cnt; i++) {
		USPRINTF("%s %d: (%d, %d)\n", __FUNCTION__, __LINE__, vw_pos[i].pos.X, vw_pos[i].pos.Y);
	}

	if (vw_pos_cnt == 1) {
		return;
	}

	for (i = 0; i < vw_pos_cnt - 1; i++) {
		for (j = i + 1; j < vw_pos_cnt; j++) {
			if (vw_pos[i].pos.Y == vw_pos[j].pos.Y) {
				start = vw_pos[i].pos.X > vw_pos[j].pos.X ? vw_pos[j].pos.X : vw_pos[i].pos.X;
				end = vw_pos[i].pos.X > vw_pos[j].pos.X ? vw_pos[i].pos.X : vw_pos[j].pos.X;

				if (end - start > VIRTUAL_WALL_MAX_SEGMENT_DISTANCE) {
					continue;
				}

				USPRINTF("%s %d: i: %d\tstart: %d\tend: %d\n", __FUNCTION__, __LINE__, i, start, end);
				for (x = start; x <= end; x++) {
					USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, x, vw_pos[i].pos.Y);
					Map_SetCell(MAP, cellToCount(x), cellToCount(vw_pos[i].pos.Y), BLOCKED_BUMPER);
				}
			} else if (vw_pos[i].pos.X == vw_pos[j].pos.X) {
				start = vw_pos[i].pos.Y > vw_pos[j].pos.Y ? vw_pos[j].pos.Y : vw_pos[i].pos.Y;
				end = vw_pos[i].pos.Y > vw_pos[j].pos.Y ? vw_pos[i].pos.Y:  vw_pos[j].pos.Y;

				if (end - start > VIRTUAL_WALL_MAX_SEGMENT_DISTANCE) {
					continue;
				}

				USPRINTF("%s %d: i: %d\tstart: %d\tend: %d\n", __FUNCTION__, __LINE__, i, start, end);
				for (y = start; y <= end; y++) {
					USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, vw_pos[i].pos.X, y);
					Map_SetCell(MAP, cellToCount(vw_pos[i].pos.X), cellToCount(y), BLOCKED_BUMPER);
				}
			} else {
				slop = (((float)vw_pos[i].pos.Y) - ((float)vw_pos[j].pos.Y)) / (((float)vw_pos[i].pos.X) - ((float)vw_pos[j].pos.X));
				intercept = ((float)(vw_pos[i].pos.Y)) - slop *  ((float)(vw_pos[i].pos.X));

				start = vw_pos[i].pos.X > vw_pos[j].pos.X ? vw_pos[j].pos.X : vw_pos[i].pos.X;
				end = vw_pos[i].pos.X > vw_pos[j].pos.X ? vw_pos[i].pos.X : vw_pos[j].pos.X;

				if (TwoPointsDistance(vw_pos[i].pos.X, vw_pos[i].pos.Y, vw_pos[j].pos.X, vw_pos[j].pos.Y) > VIRTUAL_WALL_MAX_SEGMENT_DISTANCE) {
					continue;
				}

				USPRINTF("%s %d: i: %d\tstart: %d\tend: %d\n", __FUNCTION__, __LINE__, i, start, end);
				for (x = start; x <= end; x++) {
					y = (int16_t) (slop * (x) + intercept);

					USPRINTF("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, x, y);
					Map_SetCell(MAP, cellToCount(x), cellToCount(y), BLOCKED_BUMPER);					
				}
				USPRINTF("%s %d: x: %d\n", __FUNCTION__, __LINE__, x);
			}
		}
	}
}

void CM_UpdateVW(int32_t x, int32_t y)
{
	uint8_t should_remove, accept = 0;
	int16_t i, j;
	int32_t distance, tmp;
	Point16_t	point = {0, 0};

#ifdef ZONE_WALLFOLLOW
	accept = 1;
	point = Zone_GetCurrentZoneExit();
	for (i = 0; accept == 1 && i < vw_pos_cnt; i++) {
		if (TwoPointsDistance(point.X, point.Y, vw_pos[i].pos.X, vw_pos[i].pos.Y) < 4) {
			USPRINTF("%s %d: failed to use zone exit (%d, %d) as virtual wall reference point.\n", __FUNCTION__, __LINE__, point.X, point.Y);
			accept = 0;
			break;
		}
	}
#endif

	if (accept == 0) {
		USPRINTF("%s %d: fallback to use (0, 0).\n", __FUNCTION__, __LINE__);
		point.X = Map_GetXPos();	
		point.Y = Map_GetYPos();	
	}

	USPRINTF("%s %d: virtual wall reference point (%d, %d)\n", __FUNCTION__, __LINE__, point.X, point.Y);
	for (i = 4; i >= 0; i--) {
		USPRINTF("%s %d: checking point %d (%d, %d)\n", __FUNCTION__, __LINE__, i, pos_cur[i].X, pos_cur[i].Y);
		should_remove = 0;
		if (x == pos_cur[i].X && y == pos_cur[i].Y) {
			should_remove = 1;
		}

		for (j = 0; should_remove == 0 && j < vw_pos_cnt; j++) {
			distance = TwoPointsDistance(pos_cur[i].X, pos_cur[i].Y, vw_pos[j].pos.X, vw_pos[j].pos.Y);
			if (distance <= 4) {
				should_remove = 1;
				USPRINTF("%s %d: should remove: i: %d (%d, %d) distance: %d (%d, %d) (%d, %d)\n",
					__FUNCTION__, __LINE__, i, pos_cur[i].X, pos_cur[i].Y, distance,
					pos_cur[i].X, pos_cur[i].Y, vw_pos[j].pos.X, vw_pos[j].pos.Y);
			}
		}

		USPRINTF("%s %d: should remove %d\n", __FUNCTION__, __LINE__, should_remove);
		if (should_remove == 1) {
			for (j = i; j < 4; j++) {
				pos_cur[j] = pos_cur[j + 1];
			}

			pos_cur[4].X = point.X;
			pos_cur[4].Y = point.Y;
		}
	}

	distance = SHRT_MAX;
	for (i = 0; i < vw_pos_cnt; i++) {
		tmp = TwoPointsDistance(x, y, vw_pos[i].pos.X, vw_pos[i].pos.Y);
		if (tmp < distance) {
			distance = tmp;
		}
	}
	USPRINTF("%s %d: distance: %d\tvw_pos_cnt: %d\n", __FUNCTION__, __LINE__, distance, vw_pos_cnt);
	if ((distance == SHRT_MAX && vw_pos_cnt == 0 ) || (distance != SHRT_MAX && distance > 4)) {
		for (j = 4; j > 0; j--) {
			pos_cur[j] = pos_cur[j - 1];
		}
		pos_cur[0].X = x;
		pos_cur[0].Y = y;
	}
	for (j = 0; j < 5; j++) {
		USPRINTF("%s %d: vw pos (%d, %d)\n", __FUNCTION__, __LINE__, pos_cur[j].X, pos_cur[j].Y);
	}
}
#endif

uint8_t CM_MoveToTarget_Interrupt(Point32_t Target)
{
	int16_t Adjust_Speed;
	int16_t LeftSpeed,RightSpeed;
	uint16_t Direction;
	int8_t i;
	int32_t x,y;
	uint8_t Proportion = 7;
	Base_Speed = wheelspd;
	if(Base_Speed<BASE_SPEED)Base_Speed = BASE_SPEED;
	if(SpecialPIDFlag == 1)
	{
		Base_Speed = BASE_SPEED;
	}
	if((CM_ABS((Map_GetXCount() - MapOffset.X),Target.X) <= ((IsMovingFlag)?CELL_COUNT_MUL+100:100)) && 
		 (CM_ABS((Map_GetYCount() - MapOffset.Y),Target.Y) <= ((IsMovingFlag)?CELL_COUNT_MUL+100:100)))	//相差一个cell就算到了
	{
		return 1;
	}
			
	Direction = course2dest(Map_GetXCount() - MapOffset.X,Map_GetYCount() - MapOffset.Y,Target.X,Target.Y);
	
	Adjust_Speed = (Gyro_GetAngle(0) - Direction);//一度就是一个最小单位速度
	if (Adjust_Speed >= 1800) {
			Adjust_Speed -= 3600;
		} else if (Adjust_Speed <= -1800) {
			Adjust_Speed += 3600;
	}
		
	if (OBS_SLOW||Is_OBS_Near()) {
		Base_Speed = BASE_SPEED;
		Base_Speed = Base_Speed < BASE_SPEED ? BASE_SPEED : Base_Speed;
	}

	LeftSpeed  = Base_Speed + Adjust_Speed / Proportion;
	RightSpeed = Base_Speed - Adjust_Speed / Proportion;
	
	if (LeftSpeed < 0) {
		LeftSpeed = 0;
	} else if (LeftSpeed > RUN_SPEED_12) {//RUN_SPEED_11
		LeftSpeed = RUN_SPEED_12;//RUN_SPEED_11
	}

	if (RightSpeed < 0) {
		RightSpeed = 0;
	} else if (RightSpeed > RUN_SPEED_12) {//RUN_SPEED_11
		RightSpeed = RUN_SPEED_12;//RUN_SPEED_11
	}
	
	if(SpecialPIDFlag == 1)
	{
		Base_Speed = BASE_SPEED; 
		if(Adjust_Speed > 100)
		{
			RightSpeed = 0;
		}
		else if(Adjust_Speed < -100)
		{
			LeftSpeed = 0;
		}
	}

	Move_Forward(LeftSpeed, RightSpeed);
	/*----------------------------- Check Event ---------------------------*/
	if((Work_Timer - StartTimeToTempTarget) > 10)//超过5s
	{
		Wheel_Stop();
		IsMovingFlag = 0;
		GoToTempTargetFlag = 7;
	}
	
	for (i = -1;(GoToTempTargetFlag == 1) && (i <= 1); i++) 
	{			
		CM_count_normalize_ByXYCount((Map_GetXCount() - MapOffset.X),(Map_GetYCount() - MapOffset.Y), Gyro_GetAngle(0), i * CELL_SIZE, CELL_SIZE_2, &x, &y);
		if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY) {
			Wheel_Stop();
			IsMovingFlag = 0;
			GoToTempTargetFlag = 4;
		}
	}
	
	if (Touch_Detect())
	{
		Wheel_Stop();
		IsMovingFlag = 0;
		Set_Touch();
		GoToTempTargetFlag = 4;
	}
	
	if (Get_Rcon_Remote() == Remote_Home && (go_home == 0)) 
	{
		Wheel_Stop();
		IsMovingFlag = 0;
		GoToTempTargetFlag = 4;
	}
	
	CliffTemp = Get_CliffStatus();
	if(CliffTemp)
	{
		Wheel_Stop();
		IsMovingFlag = 0;
		should_follow_wall = 1;
		GoToTempTargetFlag = 5;
	}

	BumperTemp = Get_Bumper_Status();
	if(BumperTemp)
	{
		Wheel_Stop();
		IsMovingFlag = 0;
		should_follow_wall = 1;
		GoToTempTargetFlag = 3;
	}
	
	if (go_home == 0 && !(from_station == 1 && Zone_GetZoneSize() == 1)) 
	{
		RconTemp = Get_Rcon_Status();
		if(RconTemp & (RconFL_HomeT|RconFR_HomeT))
		{
			Wheel_Stop();
			IsMovingFlag = 0;
			Reset_Rcon_Status();
			GoToTempTargetFlag = 6;
		}
	}
	return 0;
}

MapTouringType CM_MoveToPoint(Point32_t Target, uint8_t vw_ignore)
{
	int32_t Target_Course, Rotate_Angle, Integrated, Left_Speed, Right_Speed, Base_Speed, distance;
	uint8_t Integration_Cycle, boundary_reach;

	ActionType action = ACTION_NONE;

	uint8_t Motor_Check_Code = 0;

	uint8_t isBumperTriggered;

	int8_t	HomeT, HomeL, HomeR, home_hit;
	uint32_t Temp_Rcon_Status = 0;

	uint32_t move_timer;
	uint8_t temp_cliff=0;
	int8_t	slow_down;
	int16_t	i;
	int32_t x, y;
	
	static int16_t lat_cell_y=0;
	static uint8_t same_y_line_counter=0;
	static int16_t lat_cell_x=0;
	static uint8_t same_x_line_counter=0;
	
	static Point16_t vw_piont[3]={0};

	MapTouringType	retval = MT_None;

	should_follow_wall = 0;
	#ifdef MOBILITY
	uint32_t Temp_Mobility_Distance = 0;
  #endif
#ifdef VIRTUAL_WALL
	uint8_t vw_update = 0;
	int32_t vw_x, vw_y;
	Point32_t	next_point;

	vw_x = Map_GetXPos();
	vw_y = Map_GetYPos();
#endif

	if(Map_GetYPos()==lat_cell_y)
	{
		same_y_line_counter++;
	}
	else
	{
		lat_cell_y = Map_GetYPos();
		same_y_line_counter = 0;
	}
	
	if(Map_GetXPos()==lat_cell_x)
	{
		same_x_line_counter++;
	}
	else
	{
		lat_cell_x = Map_GetXPos();
		same_x_line_counter = 0;
	}
	
	OBS_ON();
	if(CorMove_Rcon==0)
	{
		delay(2000);
		CorMove_Rcon = 1;
	}

	Reset_Charge_Rcon();

	USPRINTF("MoveToPoint Time: %d\tGyro Calibration: %d\n", moveToPointTimeCount / 2, Gyro_GetCalibration());
	
	HomeT = HomeL = HomeR = home_hit = slow_down = 0;
	Integration_Cycle = 0;
	Target_Course = Rotate_Angle = Integrated = Left_Speed = Right_Speed = 0;
	
	Base_Speed = (Get_LeftWheel_Speed()+Get_RightWheel_Speed())/2;
	/*--------------------------- Edit By ZZ --------------------------------*/
	if(IsMovingFlag == 0)
	{
		Target_Course = course2dest(Map_GetXCount(), Map_GetYCount(), Target.X, Target.Y);
		CM_HeadToCourse(ROTATE_TOP_SPEED_10, Target_Course);	//turn to target position		
	}	
	IsMovingFlag = 0;
	GoToTempTargetFlag = 0;
	/***************************************************************************/
	
	Gyro_Calibration_Cmd(DISABLE);

	if (Touch_Detect()) {
		USPRINTF("%s %d: Gyro Calibration: %d\n", __FUNCTION__, __LINE__, Gyro_GetCalibration());
		Gyro_Calibration_Cmd(ENABLE);
		return MT_Key_Clean;
	}
	if (Get_Clean_Mode() == Clean_Mode_GoHome) {
		USPRINTF("%s %d: Gyro Calibration: %d\n", __FUNCTION__, __LINE__, Gyro_GetCalibration());
		Gyro_Calibration_Cmd(ENABLE);

		Set_Clean_Mode(Clean_Mode_Navigation);
		return MT_Remote_Home;
	}

	Reset_Charge_Rcon();
	
	move_timer = Work_Timer;
	Set_Left_Brush(ENABLE);
	Set_Right_Brush(ENABLE);
	if (Get_LeftBrush_Stall())Set_LeftBrush_Stall(0);
	if (Get_RightBrush_Stall())Set_RightBrush_Stall(0);

	if (go_home == 1) {
		Set_VacMode(Vac_Normal);
		Set_BLDC_Speed(Vac_Speed_NormalL);
	}
	else {
		Set_Vac_Speed();
	}

	if(vw_ignore)
	{
		if((vw_piont[0].X==Map_GetXPos())&&(vw_piont[0].Y==Map_GetYPos()))
		{
			if((vw_piont[1].X==Map_GetXPos())&&(vw_piont[1].Y==Map_GetYPos()))
			{
				if((vw_piont[2].X==Map_GetXPos())&&(vw_piont[2].Y==Map_GetYPos()))
				{
					path_targets_clear_list();
					vw_piont[2] = vw_piont[1];
					vw_piont[1] = vw_piont[0];
					vw_piont[0].X = Map_GetXPos();
					vw_piont[0].Y = Map_GetYPos();
					retval = MT_None;
					Stop_Brifly();

					VirtualWall_Path_Error = 1;
					USPRINTF("%s %d: move to point: %d\tGyro Calibration: %d error:%d \n", __FUNCTION__, __LINE__, retval, Gyro_GetCalibration(),VirtualWall_Path_Error);
					Gyro_Calibration_Cmd(ENABLE);

					return retval;
				}
				else
				{
					vw_piont[2] = vw_piont[1];
					vw_piont[1] = vw_piont[0];
					vw_piont[0].X = Map_GetXPos();
					vw_piont[0].Y = Map_GetYPos();
				}
			}
			else
			{
				vw_piont[2] = vw_piont[1];
				vw_piont[1] = vw_piont[0];
				vw_piont[0].X = Map_GetXPos();
				vw_piont[0].Y = Map_GetYPos();
			}
		}
		else
		{
			vw_piont[2] = vw_piont[1];
			vw_piont[1] = vw_piont[0];
			vw_piont[0].X = Map_GetXPos();
			vw_piont[0].Y = Map_GetYPos();
		}
	}
	
		/*-------------------------- Edit By ZZ ----------------------*/
	Rotate_Angle = g_LastAngle - Gyro_GetAngle(0);
	if (Rotate_Angle >= 1800) {
			Rotate_Angle -= 3600;
		} else if (Rotate_Angle <= -1800) {
			Rotate_Angle += 3600;
		}
	if((g_LastPosition.X == Map_GetXPos()) && (g_LastPosition.Y == Map_GetYPos()))//同一点且同一方向
	{
		g_SamePositionCnt ++;
		if(g_SamePositionCnt >= 3 && (abs(Rotate_Angle) < 100))		
		{
			USPRINTF("%s %d: Not move to target point! Current cell:(%d,%d),Target cell:(%d,%d)\n",__FUNCTION__,__LINE__,Map_GetXPos(),Map_GetYPos(),countToCell(Target.X),countToCell(Target.Y));
			USPRINTF("&&&&&&&robot Set boundary&&&&&&&!\n");
			switch(get_robot_direction_NSWE(Gyro_GetAngle(0)))
			{
				case NORTH_ZZ: Map_SetCell(MAP,cellToCount(Map_GetXPos()+2),Map_GetYCount(),BLOCKED);break;
				case WEST_ZZ: Map_SetCell(MAP,Map_GetXCount(),cellToCount(Map_GetYPos()+2),BLOCKED);break;
				case SOUTH_ZZ: Map_SetCell(MAP,cellToCount(Map_GetXPos()-2),Map_GetYCount(),BLOCKED);break;
				case EAST_ZZ: Map_SetCell(MAP,Map_GetXCount(),cellToCount(Map_GetYPos()-2),BLOCKED);break;
			}
			return MT_None;
		}
	}
	else
	{
		g_SamePositionCnt = 0;	
	}

	g_LastAngle = Gyro_GetAngle(0);
	g_LastPosition = Map_GetCurrentCell();
	
	while (1) {	
	
		if (CM_ABS(Map_GetXCount(), Target.X) < (3 * CELL_COUNT_MUL+150) && CM_ABS(Map_GetYCount(), Target.Y) < (3 * CELL_COUNT_MUL+150)) 
		{
			IsMovingFlag = 1;
			StartPointToTempTarget = Map_GetCurrentPoint();
			StartAngleToTempTarget = Gyro_GetAngle(0);
			StartTimeToTempTarget = Work_Timer;
			USPRINTF("StartTimeToTempTarget:%d\n",StartTimeToTempTarget);
			USPRINTF("Eixt at:(%d,%d),Angle:%d\n",Map_GetXPos(),Map_GetYPos(),Gyro_GetAngle(0));
			MapOffset.X = Target.X-Map_GetXCount();
			MapOffset.Y = Target.Y-Map_GetYCount();
			Map_MoveTo(MapOffset.X,MapOffset.Y);
			TempTarget = Target;
			GoToTempTargetFlag = 1;
			retval = MT_None;
			wheelspd=(Get_LeftWheel_Speed()+Get_RightWheel_Speed())/2;
			break;
		}
		
		#ifdef MOBILITY
		/*-------------------------------------Mobility----------------------------------------------*/
    if(Get_LeftWheel_Step()<500)
    {
      Temp_Mobility_Distance = Get_Move_Distance();
    }
    else
    {
      if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
      {
        Temp_Mobility_Distance = Get_Move_Distance();
        Check_Mobility();
        Reset_Mobility_Step();
      }
    }
		#endif
		/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code = Check_Motor_Current();
		if (Motor_Check_Code) {
			if (Self_Check(Motor_Check_Code)) {
				Stop_Brifly();
				CM_TouringCancel();
				Set_Clean_Mode(Clean_Mode_Userinterface);				
				retval = MT_Key_Clean;
				break;
			}
			else
			{
				Base_Speed = BASE_SPEED;
			}
			Initialize_Motor();
		}

		/* Timer for CM_MoveToPoint, 40 seconds, to prevent non-stop turn, due to bumper hitted but not triggered. */
		if (Work_Timer > move_timer + 80) {
			Stop_Brifly();
			retval = MT_None;
			break;
		}

		/* Check low battery event, if battery is low, stop. */
		if (go_home == 1) {
			if (Check_Bat_SetMotors(Home_Vac_Power, Home_SideBrush_Power, Home_MainBrush_Power) && go_home != 1 ) {
				Stop_Brifly();
				USPRINTF("%s %d: low battery, battery < 1200 is detected.\n", __FUNCTION__, __LINE__);
				retval = MT_Battery;
				break;
			}
		} else {
			if (Check_Bat_SetMotors(Clean_Vac_Power, Clean_SideBrush_Power, Clean_MainBrush_Power) && go_home != 1 ) {
				Stop_Brifly();
				USPRINTF("%s %d: low battery, battery < 1200 is detected.\n", __FUNCTION__, __LINE__);
				retval = MT_Battery;
				break;
			}
		}

		if ((retval = CM_handleExtEvent()) != MT_None) {
			break;
		}

	  temp_cliff = Get_Cliff_Trig();
		if (temp_cliff) {
			Set_Wheel_Speed(0, 0);
			Set_Dir_Backward();
			delay(300);
			CM_update_map_Cliff(temp_cliff);
			temp_cliff = Get_Cliff_Trig();
			if (temp_cliff) {
				USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
				CM_CorBack(COR_BACK_50MM);
				CM_update_map_Cliff(temp_cliff);
#ifdef CLIFF_ERROR
				
				temp_cliff = Get_Cliff_Trig();
				if (temp_cliff) {
					if (temp_cliff == Status_Cliff_All) {
						Stop_Brifly();
						retval = MT_Key_Clean;
						break;
					}
					USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
					CM_CorBack(COR_BACK_20MM);
					CM_update_map_Cliff(temp_cliff);
					
					if (Get_Cliff_Trig()) {
						if (Get_Cliff_Trig() == Status_Cliff_All) {
							Stop_Brifly();
							retval = MT_Key_Clean;
							break;
						}
						USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
						CM_CorBack(COR_BACK_20MM);
						if (Get_Cliff_Trig()) {
								Set_Error_Code(Error_Code_Cliff);
								Stop_Brifly();
								retval = MT_Key_Clean;
								break;
						}
					}
				}
#endif
				Stop_Brifly();
				CheckGyroCalibrationTime(120);
				should_follow_wall = 1;
				retval = MT_None;
				USPRINTF("%s %d: cliff break!\n", __FUNCTION__, __LINE__);
				break;
			}
		}

		Temp_Rcon_Status = Get_Rcon_Status() & Rcon_Signal_All_T;
#ifdef ZONE_WALLFOLLOW
		if (go_home == 0 && Temp_Rcon_Status)
#else
		if (go_home == 0 && Temp_Rcon_Status) 
#endif
			{
			Reset_Charge_Rcon();
			if (Temp_Rcon_Status & (RconFR_HomeT | RconFL_HomeT)) {
				HomeT++;
			}  
			if (Temp_Rcon_Status & RconL_HomeT) {
				HomeL++;
			} 
			if (Temp_Rcon_Status & RconR_HomeT) {
				HomeR++;
			}
			USPRINTF("%s %d: home detected %x(%d %d %d)\n", __FUNCTION__, __LINE__, Temp_Rcon_Status, HomeL, HomeT, HomeR);

			if (HomeR + HomeL + HomeT > 4) {
				home_hit = HomeR > HomeL ? HomeR : HomeL;
				home_hit = home_hit > HomeT ? home_hit : HomeT;

				Stop_Brifly();
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);
				if (home_hit == HomeR) {

					CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_2, &x, &y);

					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				} else if (home_hit == HomeL) {

					CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_2, &x, &y);

					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				} else {

					CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_2, &x, &y);

					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				}
#ifndef ZONE_WALLFOLLOW
				CM_SetStationHome();
#endif
				Stop_Brifly();
				CheckGyroCalibrationTime(120);
				retval = MT_None;
				break;
			} else if (HomeT == 0 && (HomeR > 2 || HomeL > 2)) {
				Stop_Brifly();
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);

				if (HomeR > 2) {

					CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_2, &x, &y);

					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				} else if (HomeL > 2) {

					CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_2, &x, &y);

					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				}
#ifndef ZONE_WALLFOLLOW
				CM_SetStationHome();
#endif
				Stop_Brifly();
				retval = MT_None;
				break;
			} else if (HomeR == 0 && HomeL == 0 && HomeT > 2) {
				Stop_Brifly();
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);

				CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_2, &x, &y);

				Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
#ifndef ZONE_WALLFOLLOW
				CM_SetStationHome();
#endif

				Stop_Brifly();
				retval = MT_None;
				break;
			}
		}else if (go_home == 1 && Is_Station() == 1 ) {
#ifndef ZONE_WALLFOLLOW
				CM_SetStationHome();
#endif
			Stop_Brifly();
			retval = MT_None;
			break;
		}

#ifdef VIRTUAL_WALL
		if (vw_ignore == 0) {			
			if(go_home == 0)
			{
				vw_update = 1;
				if (Is_VirtualWall()) {
					vw_update = 0;
					Stop_Brifly();
					CheckGyroCalibrationTime(120);
					Temp_Rcon_Status = Get_Rcon_Status();					

					if (Temp_Rcon_Status & (RconFR_Wall | RconFL_Wall|RconFR_Wall_T | RconFL_Wall_T)) {
					USPRINTF("%s %d front left/right\n", __FUNCTION__, __LINE__);
						CM_VM_insert(RconFR_Wall | RconFL_Wall);
					}
					if (Temp_Rcon_Status & (RconR_Wall|RconR_Wall_T)) {
						USPRINTF("%s %d right\n",__FUNCTION__, __LINE__);
						CM_VM_insert(RconR_Wall);
					}
					if (Temp_Rcon_Status & (RconBR_Wall|RconBR_Wall_T)) {
						USPRINTF("%s %d back right\n", __FUNCTION__, __LINE__);
						CM_VM_insert(RconBR_Wall);						
					}
					if (Temp_Rcon_Status & (RconL_Wall|RconL_Wall_T)) {
						USPRINTF("%s %d left\n", __FUNCTION__, __LINE__);
						CM_VM_insert(RconL_Wall);
					}
					if (Temp_Rcon_Status & (RconBL_Wall|RconBL_Wall_T)) {
						USPRINTF("%s %d back left\n", __FUNCTION__, __LINE__);

						CM_VM_insert(RconBL_Wall);
					}
					CM_UpdateVW(vw_x, vw_y);
					USPRINTF("%s %d: (%d, %d) (%d, %d)\n",
						__FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos(), pos_cur[0].X, pos_cur[0].Y);
					
					x = Map_GetXCount();
					y = Map_GetYCount();

					next_point.X = cellToCount(pos_cur[0].X);
					next_point.Y = cellToCount(pos_cur[0].Y);

					retval = CM_MoveToPoint(next_point, 1);
					CM_ProceVW();

					i = vw_x;
					
					if(VirtualWall_Path_Error==1)
					{
						break;
					}
					else
					{				
						while (1) {
							if (vw_x > countToCell(x)) {
								if (i <= countToCell(x) + 2) {
									break;
								}
							} else {
								if (i >= countToCell(x) - 2) {
									break;
								}
							}
							if ( path_targets_try_add_one(i, vw_y - 2, 0) == 1 && Map_GetCell(MAP, i, vw_y - 2) == UNCLEAN ) {
								USPRINTF("%s %d: adding target (%d, %d)\n", __FUNCTION__, __LINE__, i, vw_y - 2);
								path_targets_add_one(i, vw_y - 2, 0);
							}
							if ( path_targets_try_add_one(i, vw_y + 2, 0) == 1 && Map_GetCell(MAP, i, vw_y + 2) == UNCLEAN ) {
								USPRINTF("%s %d: adding target (%d, %d)\n", __FUNCTION__, __LINE__, i, vw_y + 2);
								path_targets_add_one(i, vw_y + 2, 0);
							}
							i = (vw_x > countToCell(x)) ? i - 1 : i + 1;
						}				
						Reset_Rcon_Status();
						break;
					}
				}
			}
			else if((go_home==1)&&(Get_Rcon_Status()&(RconR_Wall_T | RconL_Wall_T|RconFR_Wall_T | RconFL_Wall_T)))
			{
				vw_update = 1;
				if (Is_VirtualWall()) {
					vw_update = 0;
					Stop_Brifly();

					Temp_Rcon_Status = Get_Rcon_Status();
					if (Temp_Rcon_Status & (RconFR_Wall | RconFL_Wall|RconFR_Wall_T | RconFL_Wall_T)) {
					USPRINTF("%s %d front left/right\n", __FUNCTION__, __LINE__);

						CM_VM_insert(RconFR_Wall | RconFL_Wall);
					}
					if (Temp_Rcon_Status & (RconR_Wall|RconR_Wall_T)) {
						USPRINTF("%s %d right\n",__FUNCTION__, __LINE__);

						CM_VM_insert(RconR_Wall);
					}
					if (Temp_Rcon_Status & (RconBR_Wall|RconBR_Wall_T)) {
						USPRINTF("%s %d back right\n", __FUNCTION__, __LINE__);

						CM_VM_insert(RconBR_Wall);
					}
					if (Temp_Rcon_Status & (RconL_Wall|RconL_Wall_T)) {
						USPRINTF("%s %d left\n", __FUNCTION__, __LINE__);

						CM_VM_insert(RconL_Wall);
					}
					if (Temp_Rcon_Status & (RconBL_Wall|RconBL_Wall_T)) {
						USPRINTF("%s %d back left\n", __FUNCTION__, __LINE__);

						CM_VM_insert(RconBL_Wall);
					}
					CM_UpdateVW(vw_x, vw_y);
					USPRINTF("%s %d: (%d, %d) (%d, %d)\n",
						__FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos(), pos_cur[0].X, pos_cur[0].Y);

					x = Map_GetXCount();
					y = Map_GetYCount();

					next_point.X = cellToCount(pos_cur[0].X);
					next_point.Y = cellToCount(pos_cur[0].Y);
					
					retval = CM_MoveToPoint(next_point, 1);
					CM_ProceVW();

					if(VirtualWall_Path_Error==1)
					{
						VirtualWall_Path_Error = 0;	
						break;
					}
					else
					{			
						i = vw_x;
						while (1) {
							if (vw_x > countToCell(x)) {
								if (i <= countToCell(x) + 2) {
									break;
								}
							} else {
								if (i >= countToCell(x) - 2) {
									break;
								}
							}
							if ( path_targets_try_add_one(i, vw_y - 2, 0) == 1 && Map_GetCell(MAP, i, vw_y - 2) == UNCLEAN ) {
								USPRINTF("%s %d: adding target (%d, %d)\n", __FUNCTION__, __LINE__, i, vw_y - 2);
								path_targets_add_one(i, vw_y - 2, 0);
							}
							if ( path_targets_try_add_one(i, vw_y + 2, 0) == 1 && Map_GetCell(MAP, i, vw_y + 2) == UNCLEAN ) {
								USPRINTF("%s %d: adding target (%d, %d)\n", __FUNCTION__, __LINE__, i, vw_y + 2);
								path_targets_add_one(i, vw_y + 2, 0);
							}
							i = (vw_x > countToCell(x)) ? i - 1 : i + 1;
						}
						Reset_Rcon_Status();
						break;
					}
				}
			}
		}
#endif
				
#ifdef OBS_DYNAMIC		
		OBS_Dynamic_Base(25);
		if (Cormove_Get_OBSStatus()){
			USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
			Stop_Brifly();
			CheckGyroCalibrationTime(120);
			
			CM_update_map_bumper(action, AllBumperT);
			CM_CorBack(COR_BACK_50MM);
			Stop_Brifly();			
			CM_update_map(action, AllBumperT);
			
			
			isBumperTriggered = Get_Bumper_Status();
			CM_update_map(action, isBumperTriggered);			
			retval = MT_None;
			should_follow_wall = 1;
			USPRINTF("%s %d: OBS break!\n", __FUNCTION__, __LINE__);
			break;
		}
#else
		if (Get_FrontOBS() > 5 * Get_OBST_Value()) {
			if (front_obs_val == 0) {
				front_obs_val = Get_OBST_Value();
			} else {
				if (Get_OBST_Value() - front_obs_val > 50) {
					front_obs_val = Get_OBST_Value();
				} else {
					Stop_Brifly();
					CheckGyroCalibrationTime(120);
					isBumperTriggered = Get_Bumper_Status();
					CM_update_map(action, isBumperTriggered);
					retval = MT_None;
					should_follow_wall = 1;
					USPRINTF("%s %d: OBS break!\n", __FUNCTION__, __LINE__);
					break;
				}
			}
		}
#endif

		isBumperTriggered = Get_Bumper_Status();
		if (isBumperTriggered) {
			Stop_Brifly();
			CM_update_map_bumper(action, isBumperTriggered);

			USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
			CM_CorBack(COR_BACK_50MM);

#ifdef BUMPER_ERROR
			if (Get_Bumper_Status()) {
				USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
				CM_CorBack(COR_BACK_50MM);
				if (Get_Bumper_Status()) {
					CM_CorBack(COR_BACK_50MM);
					if (Get_Bumper_Status()) {
						USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
						CM_CorBack(COR_BACK_50MM);
						Set_Error_Code(Error_Code_Bumper);
						Stop_Brifly();
						retval = MT_Key_Clean;
						USPRINTF("%s %d: bumper jam break!\n", __FUNCTION__, __LINE__);
						break;
					}
				}
			}

#endif
			Stop_Brifly();
			CheckGyroCalibrationTime(120);
			CM_update_map(action, isBumperTriggered);
			retval = MT_None;
			should_follow_wall = 1;
			USPRINTF("%s %d: bumper break!\n", __FUNCTION__, __LINE__);
			break;
		}
		
		/* Check map boundary. */
		boundary_reach = 0;
		for (i = -1; boundary_reach == 0 && i <= 1; i++) {						
			CM_count_normalize(Gyro_GetAngle(0), i * CELL_SIZE, CELL_SIZE_2, &x, &y);
			if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY) {
				Wheel_Stop();			
				CheckGyroCalibrationTime(120);
				boundary_reach = 1;
				break;
			}
		}

		if (boundary_reach == 1) {
			USPRINTF("%s %d: boundary break!\n", __FUNCTION__, __LINE__);
			retval = MT_None;
			break;
		}

		/*--------------------------Adjust Move ------------------------------------*/
		Rotate_Angle = course2dest(Map_GetXCount(), Map_GetYCount(), Target.X, Target.Y) - Gyro_GetAngle(0);

		if (Rotate_Angle >= 1800) {
			Rotate_Angle -= 3600;
		} else if (Rotate_Angle <= -1800) {
			Rotate_Angle += 3600;
		}	

		Integration_Cycle++;
		if (Integration_Cycle > 10) {
			Integration_Cycle = 0;
			Integrated += Rotate_Angle;
			if (Integrated > 150) {
				Integrated = 150;
			} else if (Integrated < -150) {
				Integrated = -150;
			}
		}

		distance = TwoPointsDistance(Map_GetXCount(), Map_GetYCount(), Target.X, Target.Y);

		if (Cormove_Get_OBSStatus()) {
			Integrated = 0;
			Base_Speed -= 5;
			Base_Speed = Base_Speed < BASE_SPEED ? BASE_SPEED : Base_Speed;
		}
		else if (OBS_SLOW||Is_OBS_Near()) {
			Integrated = 0;
			Base_Speed -= 5;
			Base_Speed = Base_Speed < BASE_SPEED ? BASE_SPEED : Base_Speed;
		}
		else if ((distance < SLOW_DOWN_DISTANCE) || slow_down) {
			Integrated = 0;
			Base_Speed -= 1;
			Base_Speed = Base_Speed < RUN_SPEED_12 ? RUN_SPEED_12 : Base_Speed;
		}
		else if (Base_Speed <  RUN_SPEED_15) {//RUN_SPEED_15
			Base_Speed += 1;
			Integrated = 0;
//			if(Rotate_Angle>20||Rotate_Angle<-20)
//			{
//				if(Base_Speed>RUN_SPEED_5)Base_Speed=RUN_SPEED_5;
//			}
		}
		Left_Speed  = Base_Speed - Rotate_Angle / 10 - Integrated / 150; //10-150
		Right_Speed = Base_Speed + Rotate_Angle / 10 + Integrated / 150; //10-150

		if (Left_Speed < 0) {
			Left_Speed = 0;
		} else if (Left_Speed > RUN_SPEED_15) {//RUN_SPEED_15
			Left_Speed = RUN_SPEED_15;//RUN_SPEED_15
		}

		if (Right_Speed < 0) {
			Right_Speed = 0;
		} else if (Right_Speed > RUN_SPEED_15) {//RUN_SPEED_15
			Right_Speed = RUN_SPEED_15;//RUN_SPEED_15
		}

//		if(abs(Rotate_Angle)>=600)
		if(distance<CELL_COUNT_MUL*2)	
		{
			Base_Speed=RUN_SPEED_6;
			if(Left_Speed>(RUN_SPEED_6))Left_Speed=RUN_SPEED_6;
			if(Right_Speed>(RUN_SPEED_6))Right_Speed=RUN_SPEED_6;
		}		
																		
		
		Move_Forward(Left_Speed, Right_Speed);
		Base_Speed = (Left_Speed + Right_Speed) / 2;
		
		delay(10);
	}
		
#ifdef VIRTUAL_WALL
	if (vw_update == 1) {
		CM_UpdateVW(vw_x, vw_y);
	}
#endif

	if(same_x_line_counter>4)
	{
		same_x_line_counter = 0;
		CM_update_map_bumper(action, AllBumperT);
	}
	
	if(same_y_line_counter>4)
	{
		same_y_line_counter = 0;
		CM_update_map_bumper(action, AllBumperT);
	}
	
	USPRINTF("%s %d: move to point: %d\tGyro Calibration: %d\n", __FUNCTION__, __LINE__, retval, Gyro_GetCalibration());
	Gyro_Calibration_Cmd(ENABLE);
	return retval;
}


//return: 1: return to user interface;
//        0: Normal return
uint8_t CM_MoveForward(void) {
	uint8_t retval = 0, boundary_reach = 0;;
	int8_t i = 0, isBumperTriggered;
	uint16_t Temp_Speed = 0;
	uint32_t Temp_Status = 0;
	uint8_t Motor_Check_Code = 0;
	int32_t	x, y;
	Point16_t start_point;
	#ifdef MOBILITY
	uint32_t Temp_Mobility_Distance = 0;
	#endif
	uint8_t temp_cliff=0;
	
	i = 0;
	start_point.X = Map_GetXPos();
	start_point.Y = Map_GetYPos();
	
	while (1) {
		
		temp_cliff = Get_Cliff_Trig();
		if (temp_cliff) {
			Set_Wheel_Speed(0, 0);
			Set_Dir_Backward();
			delay(300);
		
			CM_update_map_Cliff(temp_cliff);
			
			if (Get_Cliff_Trig()) {
				USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
				CM_CorBack(COR_BACK_20MM);
				CM_update_map_Cliff(Get_Cliff_Trig());
				
#ifdef CLIFF_ERROR
				if (Get_Cliff_Trig()) {
					if (Get_Cliff_Trig() == Status_Cliff_All) {
						Stop_Brifly();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						retval = 1;
						break;
					}
					USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
					CM_CorBack(COR_BACK_20MM);
					if (Get_Cliff_Trig()) {
						if (Get_Cliff_Trig() == Status_Cliff_All) {
							Stop_Brifly();
							Set_Clean_Mode(Clean_Mode_Userinterface);
							retval = 1;
							break;
						}
						USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
						CM_CorBack(COR_BACK_20MM);
						if (Get_Cliff_Trig()) {
							Set_Error_Code(Error_Code_Cliff);
							Stop_Brifly();
							Set_Clean_Mode(Clean_Mode_Userinterface);
							retval = 1;
							break;
						}
					}
				}
#endif
				Stop_Brifly();
				retval = MT_None;
				USPRINTF("%s %d: cliff break!\n", __FUNCTION__, __LINE__);
				break;
			}
		}

		//8. If hit bumper
		isBumperTriggered = Get_Bumper_Status();
#ifdef OBS_DYNAMIC
		if (isBumperTriggered != 0 || Is_FrontOBS_Trig()) {
#else
		if (isBumperTriggered != 0 || (Get_FrontOBS() > Get_FrontOBST_Value())) {
#endif
			Stop_Brifly();
			USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
			CM_CorBack(COR_BACK_20MM);
			Stop_Brifly();
			CM_update_map(ACTION_NONE, isBumperTriggered);
			break;
		}
		
		if (OBS_SLOW||Is_OBS_Near()) {
			Temp_Speed = BASE_SPEED;
		} else {
			i++;
			if (i > 2) {
				i = 0;
				if (Temp_Speed < RUN_SPEED_15)Temp_Speed++;
			}
		}
		if (Temp_Speed < BASE_SPEED)Temp_Speed = BASE_SPEED;
		Move_Forward(Temp_Speed, Temp_Speed);

		#ifdef MOBILITY
		/*-------------------------------------Mobility----------------------------------------------*/
    if(Get_LeftWheel_Step()<500)
    {
      Temp_Mobility_Distance = Get_Move_Distance();
    }
    else
    {
      if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
      {
        Temp_Mobility_Distance = Get_Move_Distance();
        Check_Mobility();
        Reset_Mobility_Step();
      }
    }		
		#endif
		
#ifdef WALL_DYNAMIC
		Wall_Dynamic_Base(50);
#endif
#ifdef OBS_DYNAMIC
		OBS_Dynamic_Base(10);
#endif

		if (Touch_Detect())
		{
			Set_Clean_Mode(Clean_Mode_Userinterface);
			USPRINTF("%s %d: Check: Touch Clean Mode! return 0\n", __FUNCTION__, __LINE__);
			retval = 1;
			return retval;
		}
		#ifdef BLDC_INSTALL
		if (Remote_Key(Remote_Max)) {
			if (lowBattery == 0) {
				Switch_VacMode();
			}
		}
		#endif

		if (Remote_Key(Remote_Home) && go_home == 0) {
			Deceleration();
			Stop_Brifly();
			Set_BLDC_Speed(Vac_Speed_NormalL);
			Check_Bat_SetMotors(Home_Vac_Power, Home_SideBrush_Power, Home_MainBrush_Power);
			Set_Clean_Mode(Clean_Mode_GoHome);
			USPRINTF("%s %d: remote home is pressed.\n", __FUNCTION__, __LINE__);
			CM_SetGoHome(1);
			break;
		}

		Motor_Check_Code = Check_Motor_Current();
		if (Motor_Check_Code) {
			if (Self_Check(Motor_Check_Code)) {
				CM_TouringCancel();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				USPRINTF("%s %d: Check: motor current fail! return 0\n", __FUNCTION__, __LINE__);
				retval = 1;
				return retval;
			}
			else {
				Temp_Speed = BASE_SPEED;
			}
			Initialize_Motor();
		}

		Temp_Status = Get_Rcon_Status();
			
		if ( Temp_Status & (RconFR_HomeT | RconFL_HomeT | RconL_HomeT | RconR_HomeT) ) {
		#ifndef ZONE_WALLFOLLOW
			CM_SetStationHome();
		#endif
			USPRINTF("%s %d: home detected!\n", __FUNCTION__, __LINE__);
		}

		if (Temp_Status & Rcon_Signal_All_T) {
			Stop_Brifly();
			#ifdef RIGHT_WALL
			x = Map_GetRelativeX(Gyro_GetAngle(0), -CELL_SIZE_2, 0);
			y = Map_GetRelativeY(Gyro_GetAngle(0), -CELL_SIZE_2, 0);
			#endif
			Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
			break;
		}

#ifdef VIRTUAL_WALL
		/*
		 * When checking virtual wall signals, ignore the RconBL_Wall signal.
		 */		
		if(CM_MoveForward_Ignore_Rcon==1)		
		{
			Temp_Status = 0;
			Reset_VirtualWall();
			if(TwoPointsDistance(start_point.X,start_point.Y, Map_GetXPos(), Map_GetYPos())> 2)
			{
				CM_MoveForward_Ignore_Rcon = 0;
			}
		}
		
		if (Temp_Status & (RconL_Wall | RconR_Wall | RconBR_Wall | RconFR_Wall | RconFL_Wall|RconL_Wall_T | RconR_Wall_T  | RconFR_Wall_T | RconFL_Wall_T)) {

			Stop_Brifly();
			USPRINTF("%s %d: virtual wall detected! %d\n", __FUNCTION__, __LINE__, Temp_Status);

			if (Temp_Status & (RconFR_Wall | RconFL_Wall|RconFR_Wall_T | RconFL_Wall_T)) {
			#ifdef RIGHT_WALL
				Turn_Left(Turn_Speed, 450);
			} else if (Temp_Status & (RconR_Wall|RconR_Wall_T)) {
				Turn_Left(Turn_Speed, 300);
			} else if (Temp_Status & (RconBR_Wall)) {
				Turn_Left(Turn_Speed, 300);
			} else if (Temp_Status & (RconL_Wall|RconL_Wall_T)) {
				Turn_Left(Turn_Speed, 600);
			}
			#endif
			Reset_VirtualWall();
			USPRINTF("%s %d: Check: virtual wall! Break!\n", __FUNCTION__, __LINE__);
			break;
		}

#endif
			
		if ( go_home == 1 ) {
			break;
		}

		for (i = -1; boundary_reach == 0 && i <= 1; i++) {
			x = Map_GetRelativeX(Gyro_GetAngle(0), i * CELL_SIZE, CELL_SIZE_2);
			y = Map_GetRelativeY(Gyro_GetAngle(0), i * CELL_SIZE, CELL_SIZE_2);

			if (Map_GetCell(MAP, countToCell(x), countToCell(y)) != BLOCKED_BOUNDARY)
			{
				x = Map_GetRelativeX(Gyro_GetAngle(0), i * CELL_SIZE, CELL_SIZE_3);
				y = Map_GetRelativeY(Gyro_GetAngle(0), i * CELL_SIZE, CELL_SIZE_3);
			}

			if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY) {
				boundary_reach = 1;
				Set_Wheel_Speed(0, 0);
				delay(10);
				CM_CorBack(2 * COR_BACK_20MM);
				#ifdef RIGHT_WALL
				Turn_Left(Turn_Speed, 600);
				#endif
				break;
			}
		}
		if (boundary_reach == 1) {
			break;
		}
	}
	return retval;
}

#ifdef	PP_ROUNDING_OBSTCAL
/*------------------ Edit By ZZ -----------------------*/
uint16_t CM_get_robot_direction(uint16_t angle)
{
	uint16_t	dir;

	dir = (uint16_t) round(((double)angle) / 100);
	if((dir == 0) || (dir == 35) || (dir == 36))
		dir = NORTH;
	else if((dir == 8) || (dir == 9))
		dir = EAST;
	else if((dir == 17) || (dir == 18))
		dir = SOUTH;
	else if((dir == 26) || (dir == 27))
		dir = WEST;
	return dir;
}
#endif

uint8_t CM_Touring(void)
{
	uint8_t random_wallfollow=0;
	int8_t	state;
	uint8_t Blink_LED = 0;
	int16_t	i, k, j;

	static int16_t isolated_angle=450;
	
	Point32_t	Next_Point;
	Point16_t	tmpPnt, pnt16ArTmp[3];

#ifdef ZONE_WALLFOLLOW
	int32_t	Target_Course;

	Point16_t entrCell, exitCell, signCell, tmpPoint16 = {0, 0};
	uint16_t signDistance = 0;
	int8_t arriveTarget = 0;

	Point32_t tmpCDN, crtPoint;
	uint8_t emptyZoneCount = 0, check_point = 0;

	uint8_t isolatedWallFollowCount = 0;
	uint8_t isClearBoundary = 0;
	uint8_t isMove = 0;
	uint8_t stopMatchCnt = 0;
	uint8_t isNeedGoHome = 0;

	int16_t	l, m;
	uint8_t	caseTryTime, caseNo, gap;
	int32_t	zoneXMin, zoneXMax, zoneYMin, zoneYMax;

	uint16_t	entrCellRobotHeadingTmp, entrCellRobotHeadingTmp1, tmpDis;
	Point16_t	crtZone, entrCellTmp, entrCellTmp1, zoneCellTmp1,  trappedZoneExit, trappedStartCell;

	uint8_t virtral_move_to_exit=0;
	
	map_size = (isSingleRoom == 1) ? MAP_SINGLE_SIZE : MAP_SIZE;
#endif

	double dd, de;

#ifdef	PP_ROUNDING_OBSTCAL
	uint16_t dir;
#endif

#ifdef ENABLE_DEBUG
	uint32_t	path_time;
#endif

	uint32_t	work_timer_start;

	uint16_t	home_angle = 0, cleanning_time_allowed;
	MapTouringType	mt_state = MT_None;

#ifdef VIRTUAL_WALL
	vw_pos_cnt = 0;
	for (i = 0; i < 5; i++) {
		pos_cur[i].X = pos_cur[i].Y = 0;
	}
#endif
	Display_Clean_Status(Display_Zizag);
	CM_SetGyroOffset(0);
	Set_Station_Position(0);
	Reset_WorkTimer();
	WheelCount_Left = WheelCount_Right = 0;
	tiledUpCount = 0;

	/*--------------------- Edit By ZZ -----------------------*/
	BlockAroundChargerFlag = 0;
	/********************************************************/
	
	Work_Motor_Configure();
	NVIC_SetPriority(SysTick_IRQn, 13);
	Reset_Touch();

	cleanning_time_allowed = Is_Water_Tank() == 1 ? WET_CLEANNING_TIME : CLEANNING_TIME;

	station_zone = -1;
	from_station = 0;
	map_gyro_offset = g_map_touring_cancel = LED_Blink = LED_Blink_State = go_home = remote_go_home = 0;
	work_timer_start = Work_Timer;

	Reset_Rcon_Status();
	Reset_Touch();
	Reset_MoveWithRemote();

	/*Move back from charge station*/
	if (Is_AtHomeBase()) {
		USPRINTF("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
		Set_SideBrush_PWM(30, 30);
				if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Disable_Motors();
			return 0;
		}
		delay(1000);
				if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Disable_Motors();
			return 0;
		}
		delay(1000);
					if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Disable_Motors();
			return 0;
		}
		delay(1000);
				if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Disable_Motors();
			return 0;
		}
		delay(1000);	
		for (i = 0; i < 5; i++) {
			Quick_Back(20, COR_BACK_100MM);	//move back a short distance
			if (Touch_Detect() || Remote_Key(Remote_Clean) || Is_ChargerOn()) {
				USPRINTF("%s %d: move back 100mm and still detect charger or touch event! return 0\n", __FUNCTION__, __LINE__);
				Set_Clean_Mode(Clean_Mode_Userinterface);
				Disable_Motors();
				return 0;
			}
			Beep(3);
			
			#ifdef BLDC_INSTALL
			if(Remote_Key(Remote_Max))
			{
				Switch_VacMode();
			}
			#endif
		}
		Deceleration();
		Stop_Brifly();
		from_station = 1;
		Set_Station_Position(1);
	}

	/*Enabel Gyro*/
	//Reset value
	Gyro_IsUpdated();
	delay(200);
		if (Touch_Detect()) {
		Set_Clean_Mode(Clean_Mode_Userinterface);
		Disable_Motors();
		return 0;
	}
	Gyro_Cmd(DISABLE);
	delay(300);
			if (Touch_Detect()) {
		Set_Clean_Mode(Clean_Mode_Userinterface);
		Disable_Motors();
		return 0;
	}
	Gyro_Cmd(ENABLE);
	delay(300);
		if (Touch_Detect()) {
		Set_Clean_Mode(Clean_Mode_Userinterface);
		Disable_Motors();
		return 0;
	}
	i = 0;
	while ( Gyro_IsUpdated() == 0 ) {
		USPRINTF("%s %d Gyro Initial again!\n", __FUNCTION__, __LINE__);
		delay(200);
		Gyro_Cmd(DISABLE);
				if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Disable_Motors();
			return 0;
		}
		delay(300);
		Gyro_Cmd(ENABLE);
		i++;
		if(i>50||Touch_Detect())//7.5s
		{
			Set_Clean_Mode(Clean_Mode_Userinterface);
			if(i>50)
			{
				Set_Error_Code(Error_Code_Gyro);
			}
			Disable_Motors();
			return 0;
		}
		delay(1000);
		if(i>50||Touch_Detect())//7.5s
		{
			Set_Clean_Mode(Clean_Mode_Userinterface);
			if(i>50)
			{
				Set_Error_Code(Error_Code_Gyro);
			}
			Disable_Motors();
			return 0;
		}
	}
	USPRINTF("%s %d Gyro Initial Finish!\n", __FUNCTION__, __LINE__);

	Work_Motor_Configure();
	Blink_LED = 8;
	Reset_Touch();
	/*wati for gyro initialize*/
	while (Blink_LED--) {
		if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Disable_Motors();
			return 0;
		}
		Set_LED(0, 0, 0, 100, 0, 0);
		delay(1000);
				if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Disable_Motors();
			return 0;
		}
		delay(1000);
		if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Disable_Motors();
			return 0;
		}
		Set_LED(100,100,100,100,100,100);
		delay(1000);
				if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Disable_Motors();
			return 0;
		}
		delay(1000);
		#ifdef BLDC_INSTALL
		if(Remote_Key(Remote_Max))
		{
			Switch_VacMode();
		}
		#endif
	}
	Set_Gyro_Calibration(0);
	if((Gyro_GetCalibration()&0x20)==0x20)
	{
		Set_Gyro_Calibration(1);
	}
	else
	{
		Set_Gyro_Calibration(0);
	}
	
	Set_Wall_GoHome(0);
	g_home_point.X = g_home_point.Y = 0;
	g_charge_point.X = COR_BACK_400MM; 
	g_charge_point.Y = 0;
	USPRINTF("New Home Point: (%d, %d) (%d, %d)\n", g_charge_point.X, g_charge_point.Y, countToCell(g_charge_point.X), countToCell(g_charge_point.Y));

	Map_Initialize();
	PathPlanning_Initialize(&g_home_point.X, &g_home_point.Y);

#ifdef ZONE_WALLFOLLOW
	Zone_Initialize();
#endif

	Map_Wall_Follow_Initialize();

	Reset_Rcon_Status();

	/* delay for checking whether robot is in the station */
	delay(700);
	if (from_station == 1) {
		USPRINTF("%s %d: Turn 45 degree to the wall\n", __FUNCTION__, __LINE__);

		CM_HeadToCourse(ROTATE_TOP_SPEED_10, Gyro_GetAngle(0) + (isSingleRoom == 0 ? 450 : 1800));
		Reset_Rcon_Status();
		
		if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			USPRINTF("%s %d: Check: Touch Clean Mode! return 0\n", __FUNCTION__, __LINE__);
			return 0;
		}

		from_station = 1;
		station_zone = 0;
	}


	if (from_station == 1 && isSingleRoom == 1) {
		dd = Map_GetXCount();
		de = Map_GetYCount();

		CM_Matrix_Rotate(g_home_point.X - dd, g_home_point.Y - de, &g_home_point.X, &g_home_point.Y, -map_gyro_offset);
		home_angle = (3600 - map_gyro_offset) % 3600;

		Gyro_Reset_With_Offset((3600 + map_gyro_offset) % 3600);

		USPRINTF("Home: (%d, %d) (%d, %d)\tAngle: %d\n", countToCell(g_home_point.X), countToCell(g_home_point.Y), g_home_point.X, g_home_point.Y, home_angle);
		Map_Initialize();
		PathPlanning_Initialize(&g_home_point.X, &g_home_point.Y);
		if (from_station == 1) {
			for (i = 0; i <= countToCell(dd) - 2; i++) {
				for (j = countToCell(de) - 2; j <= countToCell(de) + 2; j++) {
					Map_SetCell(MAP, cellToCount(i), cellToCount(j), CLEANED);
				}
			}

			for (i = countToCell(dd) - 2; i <= countToCell(dd) + 2; i++) {
				for (j = countToCell(de) - 2; j <= countToCell(de) + 2; j++) {
					Map_SetCell(MAP, cellToCount(i), cellToCount(j), BLOCKED_BOUNDARY);
				}
			}
		}
	}



#ifdef ZONE_WALLFOLLOW
	if ( isSingleRoom == 0 ) {
		//Need to check mini room mode
		WFM_SetMiniRoomChecking(1);
	}
#endif

#ifdef VIRTUAL_WALL		
	End_WallFollowMulti_VirtualWall_Point=Start_WallFollowMulti_VirtualWall_Point;			
#endif	
	
	
	
	/*****************************************************Cleaning*****************************************************/
	while (1) {
				
		if (g_map_touring_cancel == 1) {
			return 0;
		}

#ifdef ZONE_WALLFOLLOW
		if ( isSingleRoom == 0 ) {
		/*************************************1 Zone Cleaning Preprocess*************************************/
		if (go_home != 1) {

			/***************************1.1 Common Process***************************/
			USPRINTF("Wall Follow!\n");
			Map_Wall_Follow(Map_Wall_Follow_Left_Zone);	
			Stop_Brifly();
			USPRINTF("Finished Wall Follow!\n");
			OBS_ON();
			CorMove_Rcon = 1;
			
			if( random_wallfollow == 1 )
			{
				if ( Zone_GetZoneWallFollowStatus() == 1 ) 
				{
					Zone_SetZoneWallFollowStatus(2);
				}
				random_wallfollow = 0;
			}
 
			if (Touch_Detect()) {
						return 0;
					}
			
			//Reset move to point count
			moveToPointTimeCount = 0;

			Stop_Brifly();

			if ( WFM_GetWallFoundStatus() == Map_Find_Wall_Found && Zone_GetZoneSize() < 3 ) {
				#ifdef DEBUG_PC_MAP
				PC_NavDebug(0xff,0xff,0,0);	
				#endif
				if(Zone_GetZoneSize()!=1)
				{
					Set_Station_Position(0);
				}
				
				//Get old zone entrance cell
				entrCellTmp = Zone_GetCurrentZoneEntrance();
				entrCellTmp1.X = entrCellTmp1.Y = zoneCellTmp1.X = zoneCellTmp1.Y = 0;

				//Get old zone entrance heading
				entrCellRobotHeadingTmp = Zone_GetCurrentZoneEntranceRobotHeading();
				entrCellRobotHeadingTmp1 = 0;
				
				USPRINTF("Current Zone Idx: %d\n", Zone_GetZoneSize() - 1);
				USPRINTF("Current Zone: x: %d\ty: %d\n", Zone_GetCurrentZone().X,
				                                         Zone_GetCurrentZone().Y);
				USPRINTF("Current Zone Entrance: x: %d\ty: %d\n", Zone_GetCurrentZoneEntrance().X,
				                                                  Zone_GetCurrentZoneEntrance().Y);
				USPRINTF("Current Zone Exit: x: %d\ty: %d\n", Zone_GetCurrentZoneExit().X,
				                                              Zone_GetCurrentZoneExit().Y);

				USPRINTF("Current Zone Entrance Robot Heading: %d\n", Zone_GetCurrentZoneEntranceRobotHeading());
				USPRINTF("Current Zone Exit Robot Heading: %d\n", Zone_GetCurrentZoneExitRobotHeading());

				//Get current point
				crtPoint.X = Map_GetXCount();
				crtPoint.Y = Map_GetYCount();

				//Reserve and calculate new home point
				CM_Matrix_Rotate(g_home_point.X - crtPoint.X, g_home_point.Y - crtPoint.Y, &g_home_point.X, &g_home_point.Y, -map_gyro_offset);
				CM_Matrix_Rotate(g_charge_point.X - crtPoint.X, g_charge_point.Y - crtPoint.Y, &g_charge_point.X, &g_charge_point.Y, -map_gyro_offset);
				USPRINTF("New charger Point: (%d, %d) (%d, %d)\n", g_charge_point.X, g_charge_point.Y, countToCell(g_charge_point.X), countToCell(g_charge_point.Y));
				home_angle = (3600 - map_gyro_offset) % 3600;

				//Reserve and calculate new entrance cell
				CM_Matrix_Rotate(cellToCount(entrCellTmp.X) - crtPoint.X, cellToCount(entrCellTmp.Y) - crtPoint.Y, &tmpCDN.X, &tmpCDN.Y, -map_gyro_offset);
				entrCellTmp1.X = countToCell(tmpCDN.X);
				entrCellTmp1.Y = countToCell(tmpCDN.Y);

				#ifdef VIRTUAL_WALL				
				CM_Matrix_Rotate(Start_WallFollowMulti_VirtualWall_Point.X - crtPoint.X, Start_WallFollowMulti_VirtualWall_Point.Y - crtPoint.Y, &Start_WallFollowMulti_VirtualWall_Point.X, &Start_WallFollowMulti_VirtualWall_Point.Y, -map_gyro_offset);
				CM_Matrix_Rotate(End_WallFollowMulti_VirtualWall_Point.X - crtPoint.X, End_WallFollowMulti_VirtualWall_Point.Y - crtPoint.Y, &End_WallFollowMulti_VirtualWall_Point.X, &End_WallFollowMulti_VirtualWall_Point.Y, -map_gyro_offset);
				#endif
							
				//Reserve and calculate new entrance robot heading
				entrCellRobotHeadingTmp1 = ( entrCellRobotHeadingTmp + 3600 - map_gyro_offset ) % 3600;

				/* Test Gyro for checking drift, if Gyro is drifting, restart it.
				 * Only do this once, since Gyro drifting is only happen after robot started moving.
				 */
				if (Gyro_Test() == 0) {
					USPRINTF("Gyro Error detected, resetting.\n");
					Gyro_Cmd(DISABLE);
					delay(5000);
					Gyro_Cmd(ENABLE);
				}

				//Reserve and calculate new wall follow boundary and clean area
				USPRINTF("Reserve and calculate new wall follow boundary and clean area!\n");
				
				j = k = 0;
				path_blocks_size = 0;
				for (i = 0; i < alignmentPtr; i++) {
					CM_Matrix_Rotate(positionAlignment[i].X - crtPoint.X, positionAlignment[i].Y - crtPoint.Y, &tmpCDN.X, &tmpCDN.Y, -map_gyro_offset);
					if ((j < ZONE_SIZE * 8) && (j == 0 || (path[j].X != countToCell(tmpCDN.X) || path[j].Y != countToCell(tmpCDN.Y)))) {
						path[j].X = countToCell(tmpCDN.X);
						path[j].Y = countToCell(tmpCDN.Y);

						path_blocks_angle[j] = ( angleAlignment[i] + 3600 - map_gyro_offset ) % 3600;
						#ifdef RIGHT_WALL
						CM_count_normalize_ByXYCount(tmpCDN.X,tmpCDN.Y,path_blocks_angle[j] ,-CELL_SIZE_2 ,0 ,&tmpCDN.X ,&tmpCDN.Y);
						#endif
						path_blocks[j].X = countToCell(tmpCDN.X);
						path_blocks[j].Y = countToCell(tmpCDN.Y);					
						j++;
					}
				}

				//Calulate new positionAlignment
				CM_Matrix_Rotate(positionAlignment[0].X - crtPoint.X, positionAlignment[0].Y - crtPoint.Y, &positionAlignment[0].X, &positionAlignment[0].Y, -map_gyro_offset);

				zoneXMin = zoneXMax = positionAlignment[0].X;
				zoneYMin = zoneYMax = positionAlignment[0].Y;

				USPRINTF("Calulate new positionAlignment\n");
				for (i = 1; i < alignmentPtr; i++) {
					CM_Matrix_Rotate(positionAlignment[i].X - crtPoint.X, positionAlignment[i].Y - crtPoint.Y, &tmpCDN.X, &tmpCDN.Y, -map_gyro_offset);

					if ( tmpCDN.X > zoneXMax ) {
						zoneXMax = tmpCDN.X;
					} else if ( tmpCDN.X < zoneXMin ) {
						zoneXMin = tmpCDN.X;
					}

					if ( tmpCDN.Y > zoneYMax ) {
						zoneYMax = tmpCDN.Y;
					} else if ( tmpCDN.Y < zoneYMin ) {
						zoneYMin = tmpCDN.Y;
					}
					positionAlignment[i].X = tmpCDN.X;
					positionAlignment[i].Y = tmpCDN.Y;
				}

				if ( 0 > zoneXMax ) {
					zoneXMax = 0;
				}
				if ( 0 < zoneXMin ) {
					zoneXMin = 0;
				}
				if ( 0 > zoneYMax ) {
					zoneYMax = 0;
				}
				if ( 0 < zoneYMin ) {
					zoneYMin = 0;
				}

				//Reserve and calculate new zone cell
				gap = 0;
				//Case 1: >	-45, < 45
				if ( course2dest( entrCellTmp1.X, entrCellTmp1.Y, 0, 0 ) < 450 ||
				     course2dest( entrCellTmp1.X, entrCellTmp1.Y, 0, 0 ) >= 3150 ) {

					zoneCellTmp1.X = countToCell(zoneXMax) + gap - ZONE_SIZE_HALF;
					zoneCellTmp1.Y = countToCell(zoneYMin) - gap + ZONE_SIZE_HALF;
					USPRINTF("Zone Direction: -45~45\n");
				}
				//Case 2: >	 45, < 135
				else if ( course2dest( entrCellTmp1.X, entrCellTmp1.Y, 0, 0 ) >= 450 &&
				          course2dest( entrCellTmp1.X, entrCellTmp1.Y, 0, 0 ) < 1350 ) {

					zoneCellTmp1.X = countToCell(zoneXMax) + gap - ZONE_SIZE_HALF;
					zoneCellTmp1.Y = countToCell(zoneYMax) + gap - ZONE_SIZE_HALF;
					USPRINTF("Zone Direction: 45~135\n");
				}
				//Case 3: >	135, < -135
				else if ( course2dest( entrCellTmp1.X, entrCellTmp1.Y, 0, 0 ) < 2250 &&
				          course2dest( entrCellTmp1.X, entrCellTmp1.Y, 0, 0 ) >= 1350 ) {

					zoneCellTmp1.X = countToCell(zoneXMin) - gap + ZONE_SIZE_HALF;
					zoneCellTmp1.Y = countToCell(zoneYMax) + gap - ZONE_SIZE_HALF;
					USPRINTF("Zone Direction: 135~225\n");
				}
				//Case 4: > -135, < -45
				else if ( course2dest( entrCellTmp1.X, entrCellTmp1.Y, 0, 0 ) >= 2250 &&
				          course2dest( entrCellTmp1.X, entrCellTmp1.Y, 0, 0 ) < 3150 ) {

					zoneCellTmp1.X = countToCell(zoneXMin) - gap + ZONE_SIZE_HALF;
					zoneCellTmp1.Y = countToCell(zoneYMin) - gap + ZONE_SIZE_HALF;
					USPRINTF("Zone Direction: 225~315\n");
				}

				//Re-initialize
				Gyro_Reset_With_Offset((3600 + map_gyro_offset) % 3600);
				Map_Initialize();
				PathPlanning_Initialize(&g_home_point.X, &g_home_point.Y);
				Zone_Initialize();

				//Set new wall follow cells
				for (i = 0; i < j; i++) {
					Map_Set_Cells(3, path[i].X, path[i].Y, CLEANED);
				}
				path_blocks_size = j;
				for (i = 0; i < j; i++) {
					Map_SetCell(MAP, cellToCount(path_blocks[i].X), cellToCount(path_blocks[i].Y), BLOCKED_OBS);
				}

				//Set current cell
				if (is_block_accessible(Map_GetXPos(), Map_GetYPos()) == 0) {
					USPRINTF("%s %d: robot is blocked, reset current robot blocks.\n", __FUNCTION__, __LINE__);
					Map_Set_Cells(ROBOT_SIZE, Map_GetXPos(), Map_GetYPos(), CLEANED);
				}

				//Add new zone and set new robot heading
				Zone_AddZone(entrCellTmp1.X, entrCellTmp1.Y, Map_GetXPos(), Map_GetYPos(), zoneCellTmp1.X, zoneCellTmp1.Y);

				Zone_SetZoneEntranceRobotHeading(entrCellRobotHeadingTmp1, Zone_GetCurrentZoneIdx());
				Zone_SetZoneExitRobotHeading(Gyro_GetAngle(0), Zone_GetCurrentZoneIdx());

				USPRINTF("New Home Point: (%d, %d) (%d, %d)\n", g_home_point.X, g_home_point.Y, countToCell(g_home_point.X), countToCell(g_home_point.Y))
				USPRINTF("Current Zone Idx: %d\n", Zone_GetZoneSize() - 1);
				USPRINTF("Current Zone: x: %d\ty: %d\n", Zone_GetCurrentZone().X,
				                                         Zone_GetCurrentZone().Y);
				USPRINTF("Current Zone Entrance: x: %d\ty: %d\n", Zone_GetCurrentZoneEntrance().X,
				                                                  Zone_GetCurrentZoneEntrance().Y);
				USPRINTF("Current Zone Exit: x: %d\ty: %d\n", Zone_GetCurrentZoneExit().X,
				                                              Zone_GetCurrentZoneExit().Y);

				USPRINTF("Current Zone Entrance Robot Heading: %d\n", Zone_GetCurrentZoneEntranceRobotHeading());
				USPRINTF("Current Zone Exit Robot Heading: %d\n", Zone_GetCurrentZoneExitRobotHeading());

				WFM_SetWallFoundStatus(Map_Find_Wall_No_Need_To_Find);
			}
			else if (  Zone_GetZoneSize() >= 3 ) {
				WFM_SetWallFoundStatus(Map_Find_Wall_No_Need_To_Find);
			}
			/***************************1.1 Common Process End***************************/

			/***************************1.2.1Finish Whole Cleaning***************************/

			if ( Zone_GetTrappedZoneSize() == 0 && Zone_GetZoneWallFollowStatus() == 1 ) {
				USPRINTF("Finished Zone Cleaning!\n");
				isNeedGoHome = 1;
				Zone_SetZoneWallFollowStatus(2);
			} else if ( Zone_IsOverZoneCount() == 1 ||
			            stopMatchCnt >= 2){
				go_home = 1;
				Zone_SetZoneWallFollowStatus(1);
				Speaker(CLEANING_END_ENTER_RECHARGE_MODE);
			} else {
				Zone_SetZoneWallFollowStatus(2);
			}
			/***************************1.2.1Finish Whole Cleaning End***************************/

			/***************************1.2.2 Normal Zone Wall Follow***************************/
			//1.2.2.1 Check if it is turing around at the same point
			//1.2.2.1-1 For no zone at the beginning
			if ( ( WFM_IsIsolatedWallFollow() == 1 || WFM_IsWallFollowTooFar() == 1 ) &&
			     Zone_GetZoneSize() == 0 ) {
				USPRINTF("Isolated wall follow but no zone! Rotate and move!\n");

				CM_ResetBoundaryBlocks();

				//Rotate
				isolated_angle = isolated_angle + 300;
				if(isolated_angle > 1350)
					isolated_angle = 450;
						 
				CM_HeadToCourse(ROTATE_TOP_SPEED_10, Gyro_GetAngle(0) + isolated_angle);

				if (Touch_Detect()) {
					return 0;
				}
				if (g_map_touring_cancel == 1) {
					return 0;
				}
				if ( CM_MoveForward() == 1 ) {
					return 0;
				}
				if (g_map_touring_cancel == 1) {
					return 0;
				}
				continue;
			}
			//1.2.2.1-1 For no zone at the beginning end

			//1.2.2.1-2 For other zones
			else if ( WFM_IsIsolatedWallFollow() == 1 ) {
				
				ShortestPath_Last_Cell.X = ShortestPath_Last_Cell.Y = 250;				//初始化思考时间计时
				isolatedWallFollowCount++;
				USPRINTF("%s %d: isolatedWallFollowCount:%d\n",__FUNCTION__,__LINE__,isolatedWallFollowCount);
				if(isolatedWallFollowCount >= 3)//在开新分区之前连续遇到三次孤岛就不再回上一分区起点,直接朝着分区起点继续沿墙
				{
					Zone_SetCurrentZoneBoundary(CLEANED, 1);
					Turn_Left(Turn_Speed,900);
					Zone_RemoveCurrentZone();
					if (Touch_Detect()) {
						return 0;
					}
					continue;
				}
				Zone_ResetPathBlocks();

				entrCell = Zone_GetCurrentZoneEntrance();

				//Set exit cell 15 cell cleaned, make sure that it can find a path
				Map_Set_Cells(ROBOT_SIZE, entrCell.X, entrCell.Y, CLEANED);

				//Go to exit of zone
				arriveTarget = CM_MoveToCell( entrCell.X, entrCell.Y, 2, 1, 1 );

				//Arrive exit of wall follow
				if (arriveTarget == -3) {
					mt_state = MT_Battery;
					return 0;
				} else if (arriveTarget == -4) {
					mt_state = MT_Remote_Home;
					go_home = 1;
				} else if (arriveTarget == -5) {
					//TODO:Add response
					mt_state = MT_Remote_Clean;
				} else if (arriveTarget == -6) {
					mt_state = MT_Battery_Home;
					go_home = 1;
				} else if ( arriveTarget == 1 ) {
					USPRINTF("Arrive zone exit!\n");

					USPRINTF("Now: x:%d\ty:%d\n", countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));
					
					Zone_SetCurrentZoneBoundary(CLEANED, 1);
					CM_HeadToCourse(ROTATE_TOP_SPEED_10, Zone_GetCurrentZoneEntranceRobotHeading() - 900);

					if (Touch_Detect()) {
						return 0;
					}
					Zone_RemoveCurrentZone();
					continue;
				} else {//去不到分区起点

					wall_follow_to_zone_cell_timer = Work_Timer;
					USPRINTF("Wall follow to zone's entrance cell!\n");
					CM_HeadToCourse(ROTATE_TOP_SPEED_10,course2dest(Map_GetXCount(),Map_GetYCount(),cellToCount(entrCell.X),cellToCount(entrCell.Y)));
					while((state = Map_Wall_Follow(Map_Wall_Follow_To_Zone_Entrance)) == 100);			//绕孤岛后直行沿墙
					Zone_SetCurrentZoneBoundary(CLEANED, 1);
					if ( g_map_touring_cancel == 1 ) {
						continue;
					}
					
					if ( go_home == 1 ) {
						continue;
					}
					
					/*----------------- Edit By ZZ -----------------*/
					if(Get_Clean_Mode() == Clean_Mode_Userinterface)
					{
						USPRINTF("%s %d: Clean mode:%d\n",__FUNCTION__,__LINE__,Get_Clean_Mode());
						return 0;
					}
					/*************************************************/

					if((state == 0)	&& (WallFollowARoundFlag == 0))//在转一圈之前到达分区起点
					{
						//Rotate to the wall
						CM_HeadToCourse(ROTATE_TOP_SPEED_10, Zone_GetCurrentZoneEntranceRobotHeading() - 900);
						Zone_RemoveCurrentZone();
						continue;
					}
					
					if((state == 0)	&& (WallFollowARoundFlag == 1))//转完一圈还未到达分区起点
					{
						//进入下一步
					}
					//沿墙超时，进入下一步
					isolatedWallFollowCount = 0;
					
					CM_HeadToCourse(ROTATE_TOP_SPEED_10,course2dest(Map_GetXCount(),Map_GetYCount(),cellToCount(entrCell.X),cellToCount(entrCell.Y)));

					if (Touch_Detect()) {
						return 0;
					}

					CM_MoveForward_Ignore_Rcon = 0;
					if ( CM_MoveForward() == 1 ) {
						return 0;
					}
				if (g_map_touring_cancel == 1) {
					return 0;
				}
				
					Zone_RemoveCurrentZone();
					USPRINTF("%s %d: Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
					continue;
				}
			}
			//1.2.2.1-2 For other zones end
			//1.2.2.1 Check if it is turing around at the same point end

			//1.2.2.2 Set start and end cell robot heading
			//Zone_SetZoneEntranceRobotHeading(resEntrCellRobotHeading, Zone_GetCurrentZoneIdx());
			Zone_SetZoneExitRobotHeading(Gyro_GetAngle(0), Zone_GetCurrentZoneIdx());
			Zone_ResetPathBlocks();

			if (Zone_GetZoneSize() == 1 && from_station == 1) {
				BlockAroundChargerFlag = 1;
				for (i = - 3; i <= 3; i++) {			//左右6个格子
					for (j = -3; j <= 3; j++) {			//前方8个格子
						CM_count_normalize_ByXYCount(g_charge_point.X,g_charge_point.Y,(home_angle+1800)%3600,i*CELL_SIZE,j*CELL_SIZE,&tmpCDN.X,&tmpCDN.Y);
						Map_SetCell(MAP, tmpCDN.X, tmpCDN.Y, BLOCKED);
					}
				}
			}

			USPRINTF("Current Zone Idx: %d\n", Zone_GetZoneSize() - 1);
			USPRINTF("Current Zone: x: %d\ty: %d\n", Zone_GetCurrentZone().X,
			         Zone_GetCurrentZone().Y);
			USPRINTF("Current Zone Entrance: x: %d\ty: %d\n", Zone_GetCurrentZoneEntrance().X,
			         Zone_GetCurrentZoneEntrance().Y);
			USPRINTF("Current Zone Exit: x: %d\ty: %d\n", Zone_GetCurrentZoneExit().X,
			         Zone_GetCurrentZoneExit().Y);

			USPRINTF("Current Zone Entrance Robot Heading: %d\n", Zone_GetCurrentZoneEntranceRobotHeading());
			USPRINTF("Current Zone Exit Robot Heading: %d\n", Zone_GetCurrentZoneExitRobotHeading());

			//1.2.2.3 Add target points
			//If it is mini room mode, then add four targets
			if ( WFM_IsMiniRoom() == 1 ) {
				USPRINTF("Mini room mode add targets.\n")
				//Find xmax, xmin, ymax, ymin
				zoneXMin = zoneXMax = positionAlignment[0].X;
				zoneYMin = zoneYMax = positionAlignment[0].Y;

				for (i = 1; i < alignmentPtr; i++) {
					if ( positionAlignment[i].X >= zoneXMax )
						zoneXMax = positionAlignment[i].X;
					else if ( positionAlignment[i].X <= zoneXMin ) {
						zoneXMin = positionAlignment[i].X;
					}
					if ( positionAlignment[i].Y >= zoneYMax )
						zoneYMax = positionAlignment[i].Y;
					else if ( positionAlignment[i].Y <= zoneYMin ) {
						zoneYMin = positionAlignment[i].Y;
					}
				}

				check_point = l = 0;
				k = ZONE_SIZE_HALF + 1;
				for (i = Zone_GetCurrentZone().X - k; i <= Zone_GetCurrentZone().X + k; i += k) {
					for (j = Zone_GetCurrentZone().Y - k; j <= Zone_GetCurrentZone().Y + k; j +=  k) {
						if (i == Zone_GetCurrentZone().X && j == Zone_GetCurrentZone().Y) {
							continue;
						}
						if ( path_targets_try_add_one(i, j, 0) == 1 && Map_GetCell(MAP, i, j) == UNCLEAN ) {
							USPRINTF("%s %d: adding check point target (%d, %d)\n", __FUNCTION__, __LINE__, i, j);
							path_targets_add_one( i, j, 0 );
						}
						if (is_block_accessible(i, j) == 1 && Map_GetCell(MAP, i, j) == UNCLEAN) {
							USPRINTF("%s %d: mini room enable check point: (%d, %d), idx: %d\n", __FUNCTION__, __LINE__, i, j, l);
							check_point |= 0x1 << l;
						}
						l++;
					}
				}
				USPRINTF("%s %d: check point: %d\n", __FUNCTION__, __LINE__, check_point);

				USPRINTF("Range: XMax: %d\tXMin: %d\tYMax: %d\tYMin: %d\n", countToCell(zoneXMax), countToCell(zoneXMin),
				         countToCell(zoneYMax), countToCell(zoneYMin));

				for ( i = countToCell(zoneXMin) + 2; i <= countToCell(zoneXMax) - 2; i = i + 2 ) {
					for ( j = countToCell(zoneYMin) + 2; j <= countToCell(zoneYMax) - 2; j = j + 2 ) {
						if ( path_targets_try_add_one(i, j, 0) == 1 && Map_GetCell(MAP, i, j) == UNCLEAN ) {
							USPRINTF("%s %d: adding target (%d, %d)\n", __FUNCTION__, __LINE__, i, j);
							path_targets_add_one( i, j, 0 );
						}
					}
				}
				USPRINTF("Target size: %d\n", path_targets_get_count());
				pnt16ArTmp[0] = Zone_GetCurrentZoneExit();
				pnt16ArTmp[1] = Zone_GetCurrentZoneEntrance();
				path_escape_set_trapped_cell(pnt16ArTmp, 2);
			}			
			else {//Normal mode
				//Set trapped reference cell
				pnt16ArTmp[0] = Zone_GetCurrentZoneExit();
				pnt16ArTmp[1] = Zone_GetCurrentZoneEntrance();
				path_escape_set_trapped_cell(pnt16ArTmp, 2);
				path_reset_last_position();
				CM_AddTargets(Zone_GetCurrentZone());
			}
			
			#ifdef VIRTUAL_WALL
			if((WallFollowMulti_End_VirtualWall)||(Get_Rcon_Status()&(RconFR_Wall | RconFL_Wall|RconR_Wall|RconBR_Wall|RconL_Wall|RconBL_Wall|RconFR_Wall_T | RconFL_Wall_T|RconR_Wall_T|RconBR_Wall_T|RconL_Wall_T|RconBL_Wall_T)))
			{
				path_targets_update();
				WallFollowMulti_End_VirtualWall = 0;
				if ( path_targets_get_count() != 0 )
				{
					CM_HeadToCourse(ROTATE_TOP_SPEED_10, Gyro_GetAngle(0)+900);
					CM_MoveForward_Ignore_Rcon = 1;	
					#ifdef RIGHT_WALL//not edit
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE), Map_GetRelativeY(Gyro_GetAngle(0), 0, 0), CLEANED);	
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE), Map_GetRelativeY(Gyro_GetAngle(0), 0, CELL_SIZE), CLEANED);	
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE_2), Map_GetRelativeY(Gyro_GetAngle(0), 0, CELL_SIZE_2), CLEANED);
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE_3), Map_GetRelativeY(Gyro_GetAngle(0), 0, CELL_SIZE_3), CLEANED);
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE), Map_GetRelativeY(Gyro_GetAngle(0), CELL_SIZE, 0), CLEANED);
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE), Map_GetRelativeY(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE), CLEANED);	
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE_2), Map_GetRelativeY(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_2), CLEANED);
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE_3), Map_GetRelativeY(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_3), CLEANED);
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE), Map_GetRelativeY(Gyro_GetAngle(0), -CELL_SIZE, 0), CLEANED);
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE), Map_GetRelativeY(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE), CLEANED);	
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE_2), Map_GetRelativeY(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_2), CLEANED);
					Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), 0, CELL_SIZE_3), Map_GetRelativeY(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_3), CLEANED);
					#endif
					CM_MoveForward();
					CM_MoveForward_Ignore_Rcon = 0;
				}
				path_targets_update();
			}
			#endif
			
			//1.2.2.4 If targets list empty, continue wall follow
			if ( path_targets_get_count() == 0 ) {
				if (from_station != 2 &&  isNeedGoHome != 1 && Zone_GetTrappedZoneSize() == 0 && WFM_IsMiniRoom() != 1 ) {
					USPRINTF("Targets list empty, continue wall follow!\n");
					Zone_SetCurrentZoneBoundary(CLEANED, 1);
					
					//No need to move forward in wall follow
					WFM_SetDirectWallFollow(1);

					emptyZoneCount++;
		
					USPRINTF("%s %d: Empty Zone Cnt: %d\n", __FUNCTION__, __LINE__, emptyZoneCount);
					continue;
				} else {
					USPRINTF("Targets list empty, but need go home or go to trapped zone!\n");
				}
			}
			else
			{
				emptyZoneCount = 0;
			}

			//1.2.2.5 Common process
			//is robot move initial
			isMove = 0;
			/***************************1.2.2 Normal Zone Wall Follow End***************************/
		}
		/*************************************1 Zone Cleaning Preprocess End*************************************/
		}
#endif

		/*************************************2 Cleaning Main Loop*************************************/
		state = -1;
		while (1) {

			/***************************2.1 Common Process***************************/
			if (g_map_touring_cancel == 1) {
				return 0;
			}
			/***************************2.1 Common Process End***************************/

			/***************************2.2-1 Go Home***************************/
			if (go_home == 1) {			
				if(BlockAroundChargerFlag)
				{
					BlockAroundChargerFlag = 0;
					for (i = - 6; i <= 6; i++) {//左右6个格子
						for (j = -3; j <= 8; j++) {//前方8个格子
							CM_count_normalize_ByXYCount(g_charge_point.X,g_charge_point.Y,(home_angle+1800)%3600,i*CELL_SIZE,j*CELL_SIZE,&tmpCDN.X,&tmpCDN.Y);
							Map_SetCell(MAP, tmpCDN.X, tmpCDN.Y, CLEANED);
						}
					}
				}
				

				if (from_station >= 1)
				{
						Set_Wall_GoHome(1);
				}
				
				//2.2-1.1 Common process
				tmpPnt.X = countToCell(g_home_point.X);
				tmpPnt.Y = countToCell(g_home_point.Y);
				pnt16ArTmp[0] = tmpPnt;
#ifdef ZONE_WALLFOLLOW
				if ( isSingleRoom == 0 ) {
					Zone_SetZoneWallFollowStatus(1);
				}
#endif
				path_escape_set_trapped_cell(pnt16ArTmp, 1);

				Next_Point.X = g_home_point.X;
				Next_Point.Y = g_home_point.Y;

				k = 0;
				while ((k++ < 10000) && (LED_Blink_State != LED_Blink)) {
					delay(1);
				}
				if (remote_go_home == 1) {
					Set_LED_On_Switch(1,0,0,1,1,0);
					SetHomeRemote();
				}

#ifdef ZONE_WALLFOLLOW
				if ( isSingleRoom == 0 ) {
					//2.2-1.2 Reset Boundary
					if ( isClearBoundary == 0 ) {
						Zone_SetCurrentZoneBoundary(CLEANED, 1);
						CM_ResetBoundaryBlocks();
						isClearBoundary = 1;
					}
				}
#endif

				//2.2-1.3 Path to unclean area
#if 0
				entrCellTmp.X = Map_GetXPos();
				entrCellTmp.Y = Map_GetYPos();
				state = path_move_to_unclean_area(entrCellTmp, countToCell(Next_Point.X), countToCell(Next_Point.Y), &tmpPoint16.X, &tmpPoint16.Y, 0);
				if (!(state == 1 || state == SCHAR_MAX)) {
					state = Map_Wall_Follow(Map_Wall_Follow_Escape_Trapped);
					Stop_Brifly();
					if (state == 2) {
						Disable_Motors();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return 0;
					}
				}
#endif
				k = 3;
				xMinSearch = xMaxSearch = yMinSearch = yMaxSearch = SHRT_MAX;
				for (i = xMin; xMinSearch == SHRT_MAX; i++) {
					for (j = yMin; j <= yMax; j++) {
						if (Map_GetCell(MAP, i, j) != UNCLEAN) {
							xMinSearch = i - k;
							break;
						}
					}
				}
				for (i = xMax; xMaxSearch == SHRT_MAX; i--) {
					for (j = yMin; j <= yMax; j++) {
						if (Map_GetCell(MAP, i, j) != UNCLEAN) {
							xMaxSearch = i + k;
							break;
						}
					}
				}
				for (i = yMin; yMinSearch == SHRT_MAX; i++) {
					for (j = xMin; j <= xMax; j++) {
						if (Map_GetCell(MAP, j, i) != UNCLEAN) {
							yMinSearch = i - k;
							break;
						}
					}
				}
				for (i = yMax; yMaxSearch == SHRT_MAX; i--) {
					for (j = xMin; j <= xMax; j++) {
						if (Map_GetCell(MAP, j, i) != UNCLEAN) {
							yMaxSearch = i + k;
							break;
						}
					}
				}
				USPRINTF("%s %d: x: %d - %d\ty: %d - %d\n", __FUNCTION__, __LINE__, xMinSearch, xMaxSearch, yMinSearch, yMaxSearch);
				for (i = xMinSearch; i <= xMaxSearch; i++) {
					if (i == xMinSearch || i == xMaxSearch) {
						for (j = yMinSearch; j <= yMaxSearch; j++) {
							Map_SetCell(MAP, cellToCount(i), cellToCount(j), BLOCKED_BUMPER);
						}
					} else {
						Map_SetCell(MAP, cellToCount(i), cellToCount(yMinSearch), BLOCKED_BUMPER);
						Map_SetCell(MAP, cellToCount(i), cellToCount(yMaxSearch), BLOCKED_BUMPER);
					}
				}

#ifdef GO_HOME_METHOD_2
				USPRINTF("Go home Target: (%d, %d)\n", tmpPnt.X, tmpPnt.Y);
				state = CM_MoveToCell( tmpPnt.X, tmpPnt.Y, 2, 6, 3 );
				if ( state == -2 ) {
					CM_HeadToCourse(ROTATE_TOP_SPEED_10, home_angle);
					Disable_Motors();
					for (i = 10; i > 0; i--) {
						Beep(i);
					}
					if (from_station >= 1) {
						Set_Wall_GoHome(1);
						Set_Clean_Mode(Clean_Mode_GoHome);
					} else {
						Set_Clean_Mode(Clean_Mode_Userinterface);
					}

					USPRINTF("%s %d: Finish cleanning but not stop near home, cleanning time: %d(s)\n", __FUNCTION__, __LINE__, (Work_Timer - work_timer_start) / 2);
					return 0;
				} else if (state == -3) {
					CM_HeadToCourse(ROTATE_TOP_SPEED_10, home_angle);
					Disable_Motors();
					mt_state = MT_Battery;
					return 0;
				} else if (state == -5) {
					CM_HeadToCourse(ROTATE_TOP_SPEED_10, home_angle);
					Disable_Motors();
					for (i = 10; i > 0; i--) {
						Beep(i);
					}
					Set_Clean_Mode(Clean_Mode_Userinterface);
					USPRINTF("%s %d: Finish cleanning to find station, cleanning time: %d(s)\n", __FUNCTION__, __LINE__, (Work_Timer - work_timer_start) / 2);
					return 0;
				} else if ( state == -7 ) {
					CM_HeadToCourse(ROTATE_TOP_SPEED_10, home_angle);
					Disable_Motors();
					for (i = 10; i > 0; i--) {
						Beep(i);
					}
					if (from_station >= 1) {
						Set_Wall_GoHome(1);
					}
					Set_Clean_Mode(Clean_Mode_GoHome);
					USPRINTF("%s %d: Finish cleanning to find station, cleanning time: %d(s)\n", __FUNCTION__, __LINE__, (Work_Timer - work_timer_start) / 2);
					return 0;
				} else if ( state == 1 ) {	
					
					CM_HeadToCourse(ROTATE_TOP_SPEED_10, home_angle);
					
					if (from_station == 0) {						

						if (Touch_Detect()) {
							return 0;
						}
					}
					
					if (g_map_touring_cancel == 1) {
						return 0;
					}

					Disable_Motors();
					for (i = 10; i > 0; i--) {
						Beep(i);
					}

					if (from_station >= 1) {						
						Set_Wall_GoHome(1);					
						Set_Clean_Mode(Clean_Mode_GoHome);
					} else {
						Set_Clean_Mode(Clean_Mode_Userinterface);
					}

					USPRINTF("%s %d: Finish cleanning, cleanning time: %d(s)\n", __FUNCTION__, __LINE__, (Work_Timer - work_timer_start) / 2);
					return 0;

				}
#else
				//2.2-1.4 Path home
				state = path_home(&Next_Point.X, &Next_Point.Y);
				//2.2-1.5 Check state
				if (state == 0) {					
					CM_HeadToCourse(ROTATE_TOP_SPEED_10, home_angle);
					
					if (from_station == 0) {						

						if (Touch_Detect()) {
							return 0;
						}
					}				
					if (g_map_touring_cancel == 1) {
						return 0;
					}
					Disable_Motors();
					for (i = 10; i > 0; i--) {
						Beep(i);
					}
					if (from_station >= 1) {
						Set_Wall_GoHome(1);
						Set_Clean_Mode(Clean_Mode_GoHome);
					} else {
						Set_Clean_Mode(Clean_Mode_Userinterface);
					}
					USPRINTF("%s %d: finish cleanning, cleanning time: %d(s)\n", __FUNCTION__, __LINE__, (Work_Timer - work_timer_start) / 2);
					return 0;
				} else {
					mt_state = CM_MoveToPoint(Next_Point, 0);
				}
#endif

			}
			/***************************2.2-1 Go Home End***************************/

			/***************************2.2-2 Normal Cleaning***************************/
			else {
				//State -1: Path next
				//State  0: Target list is empty
				//State  2: Robot is trapped

				/***************************2.2-2.1 Common State*************************/
#ifdef ZONE_WALLFOLLOW
				if ( isSingleRoom == 0 ) {
					if (state >= 1) {
						isMove = 1;
						emptyZoneCount = 0;
						isolatedWallFollowCount = 0;
						USPRINTF("%s %d Empty Zone Count, Isolated Wall Follow Count and isMove reset!\n", __FUNCTION__, __LINE__);
					}
				}
#endif
				/***************************2.2-2.1 Common State End*************************/

				/***************************2.2-2.2-1 Start State -1*************************/
				//Find path
				if (state == -1) {
					LED_Blink = 1;
#ifdef ENABLE_DEBUG
					path_time = Work_Timer;
#endif
					if(Zone_Run_Time>1800)//15min
					{
						Zone_Run_Time = 0;
						path_targets_clear_list();
					}
					state = path_next(&Next_Point.X, &Next_Point.Y);
					ResetMapOffset();
					
					USPRINTF("IsMovingFlag(%d),TempTarget(%d,%d),StartPoint(%d,%d),NextPoint(%d,%d),POS(%d,%d)\n",IsMovingFlag,countToCell(TempTarget.X),countToCell(TempTarget.Y),countToCell(StartPointToTempTarget.X),countToCell(StartPointToTempTarget.Y),countToCell(Next_Point.X),countToCell(Next_Point.Y),countToCell(Map_GetXCount()),countToCell(Map_GetYCount()));										
					if(IsMovingFlag == 1)
					{
						USPRINTF("TempTarget(%d,%d),StartPoint(%d,%d),NextPoint(%d,%d)\n",countToCell(TempTarget.X),countToCell(TempTarget.Y),countToCell(StartPointToTempTarget.X),countToCell(StartPointToTempTarget.Y),countToCell(Next_Point.X),countToCell(Next_Point.Y));
						if((countToCell(TempTarget.Y) == countToCell(Next_Point.Y)) && (CM_ABS(countToCell(TempTarget.Y),countToCell(StartPointToTempTarget.Y)) <= 1))
						{
							if(((StartPointToTempTarget.X - TempTarget.X) * (StartPointToTempTarget.X - Next_Point.X)) < 0)				//下一个目标点在反方向需要转向
							{
								IsMovingFlag = 0;
								USPRINTF("Next Target is behind!\n");
							}
						}
					}
					
					if(PP_RoundingMovingFlag)
					{
						PP_RoundingMovingFlag = 0;
						state = 1;
					}
					else
					{
						if(GoToTempTargetFlag >= 3)//思考时前行过程中遇到障碍物或者按键触发,重新计算路径
						{
							ResetMapOffset();
							IsMovingFlag = 0;
							if(GoToTempTargetFlag == 3)//保险杠
							{
								CM_signed_obstacle(0,BumperTemp,0);
								CM_CorBack(COR_BACK_20MM);
								CM_signed_obstacle(0,BumperTemp,0);
							}
							else if(GoToTempTargetFlag == 5)//探地
							{
								CM_signed_obstacle(0,0,CliffTemp);
								CM_CorBack(COR_BACK_20MM);
								CM_signed_obstacle(0,0,CliffTemp);
							}
							else if(GoToTempTargetFlag == 6)//充电座
							{
								USPRINTF("Rcon:%x,head:%d\n",RconTemp,Gyro_GetAngle(0));
							}
							Wheel_Stop();
							CheckGyroCalibrationTime(120);
							GoToTempTargetFlag = 0;
							state = -1;//在这之前将目标点周围置过“1”，可能会造成一点点漏扫
							USPRINTF("Blocked when movethinking!\n");
						}
					}
					
					if(state != 1)
					{
						IsMovingFlag = 0;
						Set_Wheel_Speed(0,0);
						GoToTempTargetFlag = 0;
						USPRINTF("State:%d\n",state);
					}
					
					USPRINTF("path time: %d\n", Work_Timer - path_time);
					USPRINTF("State: %d", state);
					LED_Blink = 0;
					
					if (g_map_touring_cancel == 1) {
						return 0;
					}					
				}
				/***************************2.2-2.2-1 Start State -1 End*************************/

				/***************************2.2-2.2-2 Start State 0*************************/
				//No target point
				else if (state == 0) {										
					ShortestPath_Last_Cell.X = ShortestPath_Last_Cell.Y = 250;//初始化思考时间计时
					
					/***************************2.2-2.2-2.1 Common Process*************************/
#ifdef ZONE_WALLFOLLOW
					if ( isSingleRoom == 0 ) {
						USPRINTF("End Zigzag!\n");

						//Set boundary CLEANED
						Zone_SetCurrentZoneBoundary(CLEANED, 1);

						if (Zone_GetCurrentZoneIdx() > FROCE_GO_HOME_ZONE_CNT || from_station == 2 || isNeedGoHome == 1 ) {
							USPRINTF("Finish all cleaning! Go home now! from station: %d\tneed go home: %d\n", from_station, isNeedGoHome);
							go_home = 1;
							Zone_SetZoneWallFollowStatus(1);
							Speaker(CLEANING_END_ENTER_RECHARGE_MODE);
							break;
						}

						//If mini room mode, then it will end cleaning
						if ( WFM_IsMiniRoom() == 1 ) {							
							WFM_ResetIsMiniRoom();							
							USPRINTF("%s %d: verifying mini room.\n", __FUNCTION__, __LINE__);

							if ( isMove == 1) {
								l = m = 0;
								k = ZONE_SIZE_HALF + 1;
								tmpPoint16.X = Map_GetXPos();
								tmpPoint16.Y = Map_GetYPos();
								USPRINTF("%s %d: check point: %d\n", __FUNCTION__, __LINE__, check_point);
								for (i = Zone_GetCurrentZone().X - k; (l == 0) && (i <= Zone_GetCurrentZone().X + k); i += k) { 
									for (j = Zone_GetCurrentZone().Y - k; (l == 0) && (j <= Zone_GetCurrentZone().Y + k); j += k) {
										if (i == Zone_GetCurrentZone().X && j == Zone_GetCurrentZone().Y) {
											continue;
										}
										if (check_point & (0x1 << m)) {
											USPRINTF("%s %d: mini room checking point: (%d, %d), idx: %d\n", __FUNCTION__, __LINE__, i, j, m);
											if (is_block_accessible(i, j) == 1) { 
												if (Map_GetCell(MAP, i, j) == CLEANED ||
													path_move_to_unclean_area(tmpPoint16, i, j, &tmpPnt.X, &tmpPnt.Y, 0 ) >= 0) {
													/*-------------------- Edit By ZZ --------------------------*/
													arriveTarget = CM_MoveToCell(i,j,2,1,1);
													if(arriveTarget == 1)
													{
														USPRINTF("%s %d: found\n", __FUNCTION__, __LINE__);
														l = 1;
													}
													else if (arriveTarget == -4) {
														mt_state = MT_Remote_Home;
														go_home = 1;
														continue;
													} else if (arriveTarget == -5) {
														mt_state = MT_Remote_Clean;
														Disable_Motors();
														Set_Clean_Mode(Clean_Mode_Userinterface);
														return 0;
													} else if (arriveTarget == -6) {
														mt_state = MT_Battery_Home;
														go_home = 1;
														continue;
													}
													/**************************************************************/
												}
											}
										}
										m++;
									}
								}
								USPRINTF("%s %d: check point index: %d\n", __FUNCTION__, __LINE__, m);

								if ( l == 0) {
									USPRINTF("%s %d: confirmed mini room, go home now!\n", __FUNCTION__, __LINE__);
									go_home = 1;
									break;
								} else {
									USPRINTF("%s %d: Not really a mini room! Continue wall follow!\n", __FUNCTION__, __LINE__);
								}
							} else {
								USPRINTF("%s %d: robot doesn't move!\n", __FUNCTION__, __LINE__);
								go_home = 1;
								break;
							}
						}

						if ( isMove == 0 && Zone_GetTrappedZoneSize() == 0 ) {
							USPRINTF("Not move! Continue wall follow!\n");

							//No need to move forward in wall follow
							WFM_SetDirectWallFollow(1);

							//Continue empty zone count ++
							emptyZoneCount++;

							USPRINTF("%s %d: Empty Zone Cnt: %d\n", __FUNCTION__, __LINE__, emptyZoneCount);
							break;
						}

						//Get entrance and exit cell
						exitCell = Zone_GetCurrentZoneExit();
						entrCell = Zone_GetCurrentZoneEntrance();
						USPRINTF("%s %d: Entrance: (%d, %d)\tExit(%d, %d)\n", __FUNCTION__, __LINE__, entrCell.X, entrCell.Y, exitCell.X, exitCell.Y);
						/***************************2.2-2.2-2.1 Common Process End*************************/

						/***************************2.2-2.2-2.2 Move to Next Zone Cell*************************/
						caseTryTime = 5, caseNo = 0;
						
						WallFollow_Point_Exit = 0;
						
					  random_wallfollow = 1;
						for ( i = 0; i < caseTryTime; ++i ) {
							//2.2-2.2-2.2.1 Check case order
							//1. If can find line in the beginning wall follow, then robot will:
							//First, move to entrance and wall follow to exit;
							//Second, move to exit
							//Third, move to middle point of entrance and exit

							//2. If cannot find line in the beginning wall follow
							//First, move to exit
							//Second, move to entrance and wall follow to exit;
							//Third, move to middle point of entrance and exit
#ifdef ALIGNMENT_ENABLE
						//Wall follow too far
						if ( WFM_IsWallFollowTooFar() == 1 ) {
							if ( caseNo == 0 && Zone_GetTrappedZoneSize() != 0 ) {
								caseNo = 5;
							} else if ( (caseNo == 0 && Zone_GetTrappedZoneSize() == 0) || caseNo == 5 ) {
								caseNo = 4;
							/*------------------ Edit By ZZ ------------------*/
							}	else if ( caseNo == 7) {
								caseNo = 3;
							/*************************************************/
							}	else {
								caseNo = 7;
							}
						}
						//Goto entrance
						else if ( WFM_GetZoneReturnType() == 1 ) {
							if ( caseNo == 0 && Zone_GetTrappedZoneSize() != 0 ) {
								caseNo = 5;
							} else if ( (caseNo == 0 && Zone_GetTrappedZoneSize() == 0) || caseNo == 5 ) {
								caseNo = 2;
							/*------------------- Edit By ZZ -----------------*/
							} else if(caseNo == 2) {
								caseNo = 7;
							} else if(caseNo == 7) {
								caseNo = 3;
							/***************************************************/
							} else {
								caseNo++;
								if ( caseNo == 4 )
									caseNo = 6;
							}
						}
						//Goto exit
						else {
							if ( caseNo == 0 && Zone_GetTrappedZoneSize() != 0 ) {
								caseNo = 5;
							} else if ( (caseNo == 0 && Zone_GetTrappedZoneSize() == 0) || caseNo == 5 ) {
								caseNo = 2;
							} else if ( caseNo == 2 ) {
								caseNo = 1;
							} else if ( caseNo == 1 ) {
							/*----------------------- Edit By ZZ ----------------*/
								caseNo = 7;
							} else if ( caseNo == 7) {
							/****************************************************/
								caseNo = 3;
							} else {
								caseNo++;
								if ( caseNo == 4 )
									caseNo = 6;
							}
						}
#else
						if ( caseNo == 0 ) {
							caseNo = 2;
						} else if ( caseNo == 2 ) {
							caseNo = 1;
						} else if ( caseNo == 1 ) {
						/*------------------------- Edit By ZZ ---------------------*/
							caseNo = 7;
						} else if ( caseNo == 7) {
						/**************************************************************/
							caseNo = 3;
						} else {
							caseNo++;
							if ( caseNo == 4 )
								caseNo = 5;
						}
#endif							
							if(virtral_move_to_exit==1)
							{
								virtral_move_to_exit = 0;
								caseNo = 1;
							}
					
							USPRINTF("%s %d Case No: %d\n",  __FUNCTION__, __LINE__, caseNo);

							//2.2-2.2-2.2.2-1 Case 1: move to entrance and wall follow to exit
							if ( caseNo == 1 ) {
								//Set entrance cell 15 cell cleaned, make sure that it can find a path
								Map_Set_Cells(ROBOT_SIZE, entrCell.X, entrCell.Y, CLEANED);

							//Move to entrance
							arriveTarget = CM_MoveToCell( entrCell.X, entrCell.Y, 2, 4, 2 );

#ifdef ZONE_ESCAPE
							//Set escape reference cell
							pnt16ArTmp[0] = Zone_GetCurrentZoneExit();
							pnt16ArTmp[1] = Zone_GetCurrentZoneEntrance();
							path_escape_set_trapped_cell(pnt16ArTmp, 2);

							tryCount = 0;
							while ( arriveTarget == -2 &&
							        tryCount < 10 &&
							        TwoPointsDistance(entrCell.X, entrCell.Y, Map_GetXPos(), Map_GetYPos()) > 5 &&
							        is_block_accessible(entrCell.X, entrCell.Y) == 1
							      ) {
								tmpstate = Map_Wall_Follow(Map_Wall_Follow_Escape_Trapped);
								Stop_Brifly();
								USPRINTF("%s %d Entrance is blocked and it is too far away from entrance! Begin escape mode!\n", __FUNCTION__, __LINE__);
								if (tmpstate == 2) {
									Disable_Motors();
									Set_Clean_Mode(Clean_Mode_Userinterface);
									return 0;
								}
								if ( go_home == 1 )
									break;
								if (g_map_touring_cancel == 1) {
									return 0;
								}
								arriveTarget = CM_MoveToCell( entrCell.X, entrCell.Y, 2, 4, 2 );
								tryCount++;
								USPRINTF("%s %d Entrance Try Count: %d!\n", __FUNCTION__, __LINE__, tryCount);
							}
							if ( go_home == 1 )
								break;
#endif
							//Arrive entrance of zone
							if (arriveTarget == -3) {
								mt_state = MT_Battery;
								return 0;
							} else if (arriveTarget == -4) {
								mt_state = MT_Remote_Home;
								go_home = 1;
								break;
							} else if (arriveTarget == -5) {
								mt_state = MT_Remote_Clean;
								return 0;
							}  else if (arriveTarget == -6) {
								mt_state = MT_Battery_Home;
								go_home = 1;
								break;
							} else if ( arriveTarget == 1 ) {
								USPRINTF("Arrive Entrance!\n");

								//Rotate to the wall
								CM_HeadToCourse(ROTATE_TOP_SPEED_10, Zone_GetCurrentZoneEntranceRobotHeading() - 900);

								if (Touch_Detect()) {
									return 0;
								}								
								if (g_map_touring_cancel == 1) {
									return 0;
								}

								//Wall follow to exit
								USPRINTF("Wall Follow to Exit...\n");
								Map_Wall_Follow(Map_Wall_Follow_Left_Target);
								Stop_Brifly();
								if (g_map_touring_cancel == 1) {
									return 0;
								}

								if ( go_home == 1 )
									break;

								//Check if it turns around at the same point
								//Turns around at the same point
								if ( WFM_IsIsolatedWallFollow() == 1 ) {

									CM_ResetBoundaryBlocks();

									USPRINTF("Wall follow failed! Directly move to entrance and try second wall follow!\n");

									//Set entrance cell 15 cell cleaned, make sure that it can find a path
									Map_Set_Cells(ROBOT_SIZE, entrCell.X, entrCell.Y, CLEANED);

									//Go to entrance of zone
									arriveTarget = CM_MoveToCell( entrCell.X, entrCell.Y, 2, 4, 2 );

#ifdef ZONE_ESCAPE
									tryCount = 0;
									while ( arriveTarget == -2 &&
									        tryCount < 10 &&
									        TwoPointsDistance(entrCell.X, entrCell.Y, Map_GetXPos(), Map_GetYPos()) > 5 &&
									        is_block_accessible(entrCell.X, entrCell.Y) == 1
									      ) {
										USPRINTF("%s %d Entrance is blocked and it is too far away from entrance! Begin escape mode!\n", __FUNCTION__, __LINE__);
										tmpstate = Map_Wall_Follow(Map_Wall_Follow_Escape_Trapped);
										Stop_Brifly();
										if (tmpstate == 2) {
											Disable_Motors();
											Set_Clean_Mode(Clean_Mode_Userinterface);
											return 0;
										}
										if ( go_home == 1 )
											break;
										if (g_map_touring_cancel == 1) {
											return 0;
										}
										arriveTarget = CM_MoveToCell( entrCell.X, entrCell.Y, 2, 4, 2 );
										tryCount++;
										USPRINTF("%s %d Entrance Try Count: %d!\n", __FUNCTION__, __LINE__, tryCount);
									}
									if ( go_home == 1 )
										break;
#endif
									if (arriveTarget == -3) {
										mt_state = MT_Battery;
										return 0;
									} else if (arriveTarget == -4) {
										mt_state = MT_Remote_Home;
										go_home = 1;
										break;
									} else if (arriveTarget == -5) {
										mt_state = MT_Remote_Clean;
										return 0;
									} else if (arriveTarget == -6) {
										mt_state = MT_Battery_Home;
										go_home = 1;
										break;
									} else if ( arriveTarget == 1 ) {

											//Rotate to the wall
										isolated_angle = isolated_angle + 150;
										isolated_angle = isolated_angle%3600;
										CM_HeadToCourse(ROTATE_TOP_SPEED_10, Zone_GetCurrentZoneEntranceRobotHeading() - isolated_angle);

										if (Touch_Detect()) {
											return 0;
										}
											if (g_map_touring_cancel == 1) {
												return 0;
											}
										//Move forward
										delay(1000);

										if ( CM_MoveForward() == 1 ) {
											return 0;
										}

										USPRINTF("%s %d: Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
										//Rotate to the wall
										CM_HeadToCourse(ROTATE_TOP_SPEED_10, Zone_GetCurrentZoneEntranceRobotHeading() - 900);

										if (Touch_Detect()) {
											return 0;
										}

										//Start second wall follow to exit!
										USPRINTF("Start second wall follow to exit...\n");
										Map_Wall_Follow(Map_Wall_Follow_Left_Target);
										Stop_Brifly();
										if (g_map_touring_cancel == 1) {
											return 0;
										}

										if ( go_home == 1 )
											break;

										//If wall follow stop position is near exit point, arrive exit and rotate to wall to begin a new wall follow
										if ( WFM_IsIsolatedWallFollow() != 1 ) {
											USPRINTF("Wall Follow Arrive!\n");

											//No need to move forward in wall follow
											WFM_SetDirectWallFollow(1);

											USPRINTF("Now: x:%d\ty:%d\n", countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));
											break;
										}
									}
								}
								//Turn around at the same point end
								//Wall follow arrive
								else {
									USPRINTF("Wall Follow Arrive!\n");

									//No need to move forward in wall follow
									WFM_SetDirectWallFollow(1);

									USPRINTF("Now: x:%d\ty:%d\n", countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));
									break;
								}							
							}//Wall follow arrive end							
						}//Arrive entrance of zone
						//2.2-2.2-2.2.2-1 Case 1: move to entrance and wall follow to exit end

						//2.2-2.2-2.2.2-2 Case 2: move to exit
						else if ( caseNo == 2 ) {
							USPRINTF("Exit need to go: x:%d\ty:%d\n", exitCell.X, exitCell.Y);
							Zone_SetCurrentZoneBoundary(BLOCKED_BOUNDARY, 1);
							//Set exit cell 15 cell cleaned, make sure that it can find a path
							Map_Set_Cells(ROBOT_SIZE, exitCell.X, exitCell.Y, CLEANED);

							//Go to exit of zone
							arriveTarget = CM_MoveToCell( exitCell.X, exitCell.Y, 2, 4, 2 );
							Zone_SetCurrentZoneBoundary(CLEANED, 1);
							//Arrive exit of wall follow
							if (arriveTarget == -3) {
								mt_state = MT_Battery;
								return 0;
							} else if (arriveTarget == -4) {
								mt_state = MT_Remote_Home;
								go_home = 1;
								break;
							} else if (arriveTarget == -5) {
								mt_state = MT_Remote_Clean;
								return 0;
							} else if (arriveTarget == -6) {
								mt_state = MT_Battery_Home;
								go_home = 1;
								break;
							} else if ( arriveTarget == 1 ) {
								
							random_wallfollow = 0;
							USPRINTF("Arrive Exit!\n");
							WallFollow_Point_Exit=1;
							//Rotate to the wall
							CM_HeadToCourse(ROTATE_TOP_SPEED_10, Zone_GetCurrentZoneExitRobotHeading() - 900);

							if (Touch_Detect()) {
								return 0;
							}

							USPRINTF("Now: x:%d\ty:%d\n", countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));
							break;
						}							
						}//Arrive exit of wall follow end
						//2.2-2.2-2.2.2-2 Case 2: move to exit end

						//2.2-2.2-2.2.2-3 Case 3: move to significant point
						else if ( caseNo == 3 ) {
							USPRINTF("%s %d: Hit significant point!\n", __FUNCTION__, __LINE__);

							//Calculate cell
							tmpCDN.X = cellToCount( exitCell.X +
							                        (int8_t)( 3 * cos( (double)(Zone_GetCurrentZoneExitRobotHeading() + 900) / 1800 * PI) ) );
							tmpCDN.Y = cellToCount( exitCell.Y +
							                        (int8_t)( 3 * sin( (double)(Zone_GetCurrentZoneExitRobotHeading() + 900) / 1800 * PI) ) );
							Target_Course = course2dest(Map_GetXCount(), Map_GetYCount(), tmpCDN.X, tmpCDN.Y);

							CM_HeadToCourse(ROTATE_TOP_SPEED_10, Target_Course);

							if (Touch_Detect()) {
								return 0;
							}
							delay(1000);

							if ( CM_MoveForward() == 1 ) {
								return 0;
							}
							USPRINTF("%s %d: Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
							break;
						}
						//2.2-2.2-2.2.2-3 Case 3: move to significant point end

						//2.2-2.2-2.2.2-4 Case 4: move to significant cell for wall follow too far
						else if ( caseNo == 4 ) {
							crtZone = Zone_GetCurrentZone();
							//Find the zone corner point
							if ( Zone_GetLastZoneDirection() > 450 &&
							     Zone_GetLastZoneDirection() <= 1350 ) {
								tmpPoint16.X = crtZone.X - ZONE_SIZE / 2;
								tmpPoint16.Y = crtZone.Y + ZONE_SIZE / 2;
							} else if ( Zone_GetLastZoneDirection() > 1350 &&
							            Zone_GetLastZoneDirection() <= 2250 ) {
								tmpPoint16.X = crtZone.X - ZONE_SIZE / 2;
								tmpPoint16.Y = crtZone.Y - ZONE_SIZE / 2;
							} else if ( Zone_GetLastZoneDirection() > 2250 &&
							            Zone_GetLastZoneDirection() <= 3150 ) {
								tmpPoint16.X = crtZone.X + ZONE_SIZE / 2;
								tmpPoint16.Y = crtZone.Y - ZONE_SIZE / 2;
							} else {
								tmpPoint16.X = crtZone.X + ZONE_SIZE / 2;
								tmpPoint16.Y = crtZone.Y + ZONE_SIZE / 2;
							}

							USPRINTF("%s %d Last Direction: %d\n", __FUNCTION__, __LINE__, Zone_GetLastZoneDirection());
							USPRINTF("%s %d Corner Point: x: %d, y: %d\n", __FUNCTION__, __LINE__, tmpPoint16.X, tmpPoint16.Y);

							//Find outest point
							signDistance = (uint16_t)(TwoPointsDistance( Map_GetXPos(), Map_GetYPos(), tmpPoint16.X, tmpPoint16.Y ));
							signCell.X = Map_GetXPos();
							signCell.Y = Map_GetYPos();

							tmpDis = 0;
							for ( j = crtZone.Y - ZONE_SIZE_HALF + 3; j <= crtZone.Y + ZONE_SIZE_HALF - 3; j = j + 2 ) {
								for ( i = crtZone.X - ZONE_SIZE_HALF + 3; i <= crtZone.X + ZONE_SIZE_HALF - 3; i = i + 2 ) {
									if ( is_block_accessible(i, j) == 1 && Map_GetCell(MAP, i, j) == CLEANED ) {
										tmpDis = (uint16_t)(TwoPointsDistance( i, j, tmpPoint16.X, tmpPoint16.Y ));
										if ( tmpDis < signDistance ) {
											signCell.X = i;
											signCell.Y = j;
											signDistance = tmpDis;
										}
									}
								}
							}
							USPRINTF("%s %d Outest Point: x: %d, y: %d, Distance: %d\n", __FUNCTION__, __LINE__,
							         signCell.X, signCell.Y, signDistance);

							//Go to significant point of zone
							arriveTarget = CM_MoveToCell( signCell.X, signCell.Y, 2, 4, 2 );
								//Arrive exit of wall follow
								if (arriveTarget == -3) {
									mt_state = MT_Battery;
									return 0;
								} else if (arriveTarget == -4) {
									mt_state = MT_Remote_Home;
									go_home = 1;
									break;
								} else if (arriveTarget == -5) {
									mt_state = MT_Remote_Clean;
									return 0;
								} else if (arriveTarget == -6) {
									mt_state = MT_Battery_Home;
									go_home = 1;
									break;
								} else if ( arriveTarget == 1 ) {
									USPRINTF("Arrive signCell!\n");

									//Rotate to the wall
									CM_HeadToCourse(ROTATE_TOP_SPEED_10, Zone_GetLastZoneDirection() - 900);

									if (Touch_Detect()) {
										return 0;
									}

									USPRINTF("%s %d Move forward!\n", __FUNCTION__, __LINE__);

									if ( CM_MoveForward() == 1 ) {
										return 0;
									}

									USPRINTF("Now: x:%d\ty:%d\n", countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));

									//Remove current zone
									if (Zone_GetZoneSize() != 0) {
										Zone_RemoveCurrentZone();
									}
									USPRINTF("%s %d Remove current zone!\n", __FUNCTION__, __LINE__);
									break;
								}

								//Remove current zone
								if (Zone_GetZoneSize() != 0) {
									Zone_RemoveCurrentZone();
								}
								USPRINTF("%s %d Remove current zone!\n", __FUNCTION__, __LINE__);

							}
							//2.2-2.2-2.2.2-4 Case 4: move to significant cell for wall follow too far end
							//2.2-2.2-2.2.2-5 Case 5: find trapped zone
							else if ( caseNo == 5 ) {
								//Set start cell
								trappedStartCell.X = Map_GetXPos();
								trappedStartCell.Y = Map_GetYPos();

								//Set the status 3, mean that now it is in finding trapped zone mode and search only in this fix map
								Zone_SetZoneWallFollowStatus(3);

								//Set search area
								xMinSearch = xMin;
								xMaxSearch = xMax;
								yMinSearch = yMin;
								yMaxSearch = yMax;

								for ( k = 0; k < Zone_GetTrappedZoneSize(); ++k ) {
									trappedZoneExit = Zone_GetZoneExit(Zone_GetTrappedZoneIdx(k));
									USPRINTF("Try %d Trapped Zone: Idx: %d\n", k, Zone_GetTrappedZoneIdx(k));
									arriveTarget = CM_MoveToCell(trappedZoneExit.X, trappedZoneExit.Y, 2, 2, 2 );

									//Arrive entrance of zone
									if (arriveTarget == -2) {
										if (abs(Zone_GetTrappedZoneIdx(k) - Zone_GetCurrentZoneIdx()) > 1) {
											USPRINTF("%s %d: removing trapped zone, too far from current zone %d(%d)\n",
												__FUNCTION__, __LINE__, Zone_GetTrappedZoneIdx(k), Zone_GetCurrentZoneIdx());
											Zone_DeleteTrappedZoneIdx(k);
										}
										continue;
									} else if (arriveTarget == -3) {
										mt_state = MT_Battery;
										return 0;
									} else if (arriveTarget == -4) {
										mt_state = MT_Remote_Home;
										go_home = 1;
										break;
									} else if (arriveTarget == -5) {
										mt_state = MT_Remote_Clean;
										return 0;
									}  else if (arriveTarget == -6) {
										mt_state = MT_Battery_Home;
										go_home = 1;
										break;
									} else if ( arriveTarget == 1 ) {
										USPRINTF("Arrive %d Trapped Zone(Idx: %d) Exit!\n", k, Zone_GetTrappedZoneIdx(k));

										//Rotate to the wall
										CM_HeadToCourse(ROTATE_TOP_SPEED_10, Zone_GetZoneExitRobotHeading(Zone_GetTrappedZoneIdx(k)) - 900);
										if (Touch_Detect()) {
											return 0;
										}
										//Delete trapped zone
										Zone_DeleteTrappedZoneIdx(k);

										//Set zone is continual
										if ( Zone_GetCurrentContinualZoneIdx() == Zone_GetTrappedZoneIdx(k) )
											Zone_SetZoneContinual(1);

										USPRINTF("Now: x:%d\ty:%d\n", countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));
										break;
									}
								}

								//Return to start cell
								if ( arriveTarget == 1 || go_home == 1 ) {
									break;
								} else {
									USPRINTF("Return to last cell!\n");
									arriveTarget = CM_MoveToCell(trappedStartCell.X, trappedStartCell.Y, 2, 4, 2 );

								//Arrive last trapped entrance of zone
								if (arriveTarget == -3) {
									mt_state = MT_Battery;
									return 0;
								} else if (arriveTarget == -4) {
									mt_state = MT_Remote_Home;
									go_home = 1;
									break;
								} else if (arriveTarget == -5) {
									mt_state = MT_Remote_Clean;
									return 0;
								}  else if (arriveTarget == -6) {
									mt_state = MT_Battery_Home;
									go_home = 1;
									break;
								} else if ( arriveTarget == 1 ) {
									Zone_SetZoneWallFollowStatus(2);
								}
							}
						}
						//2.2-2.2-2.2.2-5 Case 5: find trapped zone end
						/*------------------- Edit By ZZ --------------------------*/								
							//2.2-2.2-2.2.2-6 Case 7: wall follow to zone exit
							else if ( caseNo == 7) {
								wall_follow_to_zone_cell_timer = Work_Timer;
								CM_HeadToCourse(ROTATE_TOP_SPEED_10,course2dest(Map_GetXCount(),Map_GetYCount(),cellToCount(Zone_GetCurrentZoneEntrance().X),cellToCount(Zone_GetCurrentZoneEntrance().Y)));
								Zone_SetCurrentZoneBoundary(BLOCKED_BOUNDARY, 1);
								delay(6000);

								while((state = Map_Wall_Follow(Map_Wall_Follow_To_Zone_Exit)) == 100);//绕孤岛后直行沿墙
								Stop_Brifly();

								Zone_SetCurrentZoneBoundary(CLEANED, 1);
								if ( g_map_touring_cancel == 1 ) {
									return 0;
								}
								
								if ( go_home == 1 ) {
									continue;
								}
								
								/*----------------- Edit By ZZ -----------------*/
								if(Get_Clean_Mode() == Clean_Mode_Userinterface)
								{
									USPRINTF("%s %d: Clean mode:%d\n",__FUNCTION__,__LINE__,Get_Clean_Mode());
									return 0;
								}
								/*************************************************/
								
								if((state == 0)	&& (WallFollowARoundFlag == 0))//在转一圈之前到达分区终点
								{
									//Rotate to the wall
									random_wallfollow = 0;
									CM_HeadToCourse(ROTATE_TOP_SPEED_10, Zone_GetCurrentZoneExitRobotHeading() - 900);
									break;
								}
								
								if((state == 0)	&& (WallFollowARoundFlag == 1))//转完一圈还未到达分区终点
								{
									//进入下一步
								}
								//沿墙超时，进入下一步
								isolatedWallFollowCount = 0;
					
								CM_HeadToCourse(ROTATE_TOP_SPEED_10,course2dest(Map_GetXCount(),Map_GetYCount(),cellToCount(entrCell.X),cellToCount(entrCell.Y)));

								if (Touch_Detect()) {
									return 0;
								}

								CM_MoveForward_Ignore_Rcon = 0;
								if ( CM_MoveForward() == 1 ) {
									return 0;
								}

								USPRINTF("%s %d: Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
								continue;								
							}
						//2.2-2.2-2.2.2-5 Other Cases						
						else {
							break;
						}
						//2.2-2.2-2.2.2-5 Other Cases end
					}
					/***************************2.2-2.2-2.2 Move to Next Zone Cell End*************************/
					} else {
						go_home = 1;
					}
#else
					go_home = 1;
#endif
					break;
				}
				/***************************2.2-2.2-2 Start State 0 End*************************/

				/***************************2.2-2.2-3 Start State 1*************************/
				//Move to target
				else if (state == 1) {
				
#ifdef	PP_ROUNDING_OBSTCAL					
					/*------------------- Edit By ZZ ------------------*/
					while(GoToTempTargetFlag == 1)
					{
						if((Next_Point.Y == TempTarget.Y) &&//下一个目标点在反方向，需要转180度
							(((Next_Point.X < TempTarget.X) && (StartPointToTempTarget.X < TempTarget.X)) || ((Next_Point.X > TempTarget.X) && (StartPointToTempTarget.X > TempTarget.X))) &&
							(CM_ABS(countToCell(TempTarget.Y),countToCell(StartPointToTempTarget.Y)) == 2) &&
							(CM_ABS(TempTarget.X,StartPointToTempTarget.X) <= (CELL_COUNT_MUL + CELL_COUNT_MUL_1_2)) &&
							(CM_ABS(TempTarget.X,StartPointToTempTarget.X) >= (CELL_COUNT_MUL_1_2)))
						{
							if(CM_ABS(countToCell(Next_Point.X),countToCell(TempTarget.X)) <= 1)
								SpecialPIDFlag = 0;
							else
								SpecialPIDFlag = 1;
						}
					}
					SpecialPIDFlag = 0;
					dir = CM_get_robot_direction(Gyro_GetAngle(0));
					if(GoToTempTargetFlag >= 3)
					{
						IsMovingFlag = 0;
						if(GoToTempTargetFlag == 4)
						{
							if(Touch_Detect())
							{
								Set_Clean_Mode(Clean_Mode_Userinterface);
								Beep(5);
								return 0;
							}
							if (Remote_Key(Remote_Home) && (go_home == 0)) 
							{
								SetHomeRemote();
								go_home = 1;
								remote_go_home = 1;
								continue;
							}
						}
						else if(GoToTempTargetFlag == 3)//保险杠
						{
							CM_signed_obstacle(0,BumperTemp,0);
							CM_CorBack(COR_BACK_20MM);
							CM_signed_obstacle(0,BumperTemp,0);
						}
						else if(GoToTempTargetFlag == 5)//探地
						{
							CM_signed_obstacle(0,0,CliffTemp);
							CM_CorBack(COR_BACK_20MM);
							CM_signed_obstacle(0,0,CliffTemp);
						}
						else if(GoToTempTargetFlag == 6)//充电座
						{
							USPRINTF("Rcon:%x,head:%d\n",RconTemp,Gyro_GetAngle(0));
						}
						Wheel_Stop();
						CheckGyroCalibrationTime(120);

						USPRINTF("%s %d:GoToTempTargetFlag:%d!\n",__FUNCTION__,__LINE__,GoToTempTargetFlag);
						GoToTempTargetFlag = 0;
						if(countToCell(StartPointToTempTarget.Y) != countToCell(TempTarget.Y))
						{
							USPRINTF( "%s %d:Here!\n",__FUNCTION__,__LINE__);
							dir = CM_get_robot_direction(StartAngleToTempTarget);
							if((dir == NORTH) || (dir == SOUTH))
							{
								USPRINTF("%s %d:Here!\n",__FUNCTION__,__LINE__);	
								if((dir == NORTH) && (countToCell(Next_Point.X) > (Map_GetXPos()+3)))
								{
									USPRINTF("%s %d:Here!\n",__FUNCTION__,__LINE__);	
									if(TempTarget.Y > StartPointToTempTarget.Y)
										pp_rounding(PP_ROUNDING_LEFT,TempTarget);
									else
										pp_rounding(PP_ROUNDING_RIGHT,TempTarget);
									state = -1;
									continue;
								}
								else if((dir == SOUTH) && (countToCell(Next_Point.X) < (Map_GetXPos()-3)))
								{
									USPRINTF("%s %d:Here!\n",__FUNCTION__,__LINE__);	
									if(TempTarget.Y > StartPointToTempTarget.Y)
										pp_rounding(PP_ROUNDING_RIGHT,TempTarget);
									else
										pp_rounding(PP_ROUNDING_LEFT,TempTarget);
									state = -1;
									continue;
								}
								Next_Point = TempTarget;
							}
							else
							{
								state = -1;
								continue;
							}
						}
						else
						{
							state = -1;
							continue;
						}
					}
					/**************************************************/
					
					if (should_follow_wall == 1) {
						if (Get_Cliff_Trig()) {
							USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						} else if (Get_Bumper_Status()) {
							USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						} else if (Cormove_Get_OBSStatus()) {
							USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						} else if (Is_FrontOBS_Trig()) {
							USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						} else if (Get_LWall_ADC() > 370) {
							USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						} else if (Get_RWall_ADC() > 370) {
							USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						}
						if (abs(countToCell(Next_Point.Y)) != abs(Map_GetYPos())) {
							if (dir == NORTH) {
								if (countToCell(Next_Point.Y) < Map_GetYPos()) {
									if (countToCell(Next_Point.Y) == Map_GetYPos() - 1 || countToCell(Next_Point.Y) == Map_GetYPos() - 2) {
										USPRINTF("%s %d: left\n", __FUNCTION__, __LINE__);
										pp_rounding(PP_ROUNDING_LEFT, Next_Point);
									} else {
										USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
										mt_state = CM_MoveToPoint(Next_Point, 0);
									}
								} else {
									if (countToCell(Next_Point.Y) == Map_GetYPos() + 1 || countToCell(Next_Point.Y) == Map_GetYPos() + 2) {
										USPRINTF("%s %d: right\n", __FUNCTION__, __LINE__);
										pp_rounding(PP_ROUNDING_RIGHT, Next_Point);
									} else {
										USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
										mt_state = CM_MoveToPoint(Next_Point, 0);
									}
								}
							} else if (dir == SOUTH) {
								if (countToCell(Next_Point.Y) < Map_GetYPos()) {
									if (countToCell(Next_Point.Y) == Map_GetYPos() - 1 || countToCell(Next_Point.Y) == Map_GetYPos() - 2) {
										USPRINTF("%s %d: right\n", __FUNCTION__, __LINE__);
										pp_rounding(PP_ROUNDING_RIGHT, Next_Point);
									} else {
										USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
										mt_state = CM_MoveToPoint(Next_Point, 0);
									}
								} else {
									if (countToCell(Next_Point.Y) == Map_GetYPos() + 1 || countToCell(Next_Point.Y) == Map_GetYPos() + 2 ) {
										USPRINTF("%s %d: left\n", __FUNCTION__, __LINE__);
										pp_rounding(PP_ROUNDING_LEFT, Next_Point);
									} else {
										USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
										mt_state = CM_MoveToPoint(Next_Point, 0);
									}
								}
							} else {
								USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
								mt_state = CM_MoveToPoint(Next_Point, 0);
							}
						} else {
							if (path_targets_get_last(&tmpPnt.X, &tmpPnt.Y) == 1 && (countToCell(Next_Point.X) != SHRT_MAX || countToCell(Next_Point.X) != SHRT_MIN)) {
								if (abs(tmpPnt.Y) != abs(Map_GetYPos())) {
									if (dir == NORTH) {
										if (tmpPnt.Y < Map_GetYPos()) {
											if (tmpPnt.Y == Map_GetYPos() - 1 || tmpPnt.Y == Map_GetYPos() - 2) {
												USPRINTF("%s %d: left\n", __FUNCTION__, __LINE__);
												Next_Point.Y = cellToCount(tmpPnt.Y);
												pp_rounding(PP_ROUNDING_LEFT, Next_Point);
											} else {
												USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
												mt_state = CM_MoveToPoint(Next_Point, 0);
											}
										} else {
											if (tmpPnt.Y == Map_GetYPos() + 1 || tmpPnt.Y == Map_GetYPos() + 2) {
												USPRINTF("%s %d: right\n", __FUNCTION__, __LINE__);
												Next_Point.Y = cellToCount(tmpPnt.Y);
												pp_rounding(PP_ROUNDING_RIGHT, Next_Point);
											} else {
												USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
												mt_state = CM_MoveToPoint(Next_Point, 0);
											}
										}
									} else if (dir == SOUTH) {
										if (tmpPnt.Y < Map_GetYPos()) {
											if (tmpPnt.Y == Map_GetYPos() - 1 || tmpPnt.Y == Map_GetYPos() - 2) {
												USPRINTF("%s %d: right\n", __FUNCTION__, __LINE__);
												Next_Point.Y = cellToCount(tmpPnt.Y);
												pp_rounding(PP_ROUNDING_RIGHT, Next_Point);
											} else {
												USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
												mt_state = CM_MoveToPoint(Next_Point, 0);
											}
										} else {
											if (tmpPnt.Y == Map_GetYPos() + 1 || tmpPnt.Y == Map_GetYPos() + 2 ) {
												USPRINTF("%s %d: left\n", __FUNCTION__, __LINE__);
												Next_Point.Y = cellToCount(tmpPnt.Y);
												pp_rounding(PP_ROUNDING_LEFT, Next_Point);
											} else {
												USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
												mt_state = CM_MoveToPoint(Next_Point, 0);
											}
										}
									} else {
										USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
										mt_state = CM_MoveToPoint(Next_Point, 0);
									}
								} else {
									USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
									mt_state = CM_MoveToPoint(Next_Point, 0);
								}
							} else {
								USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
								mt_state = CM_MoveToPoint(Next_Point, 0);
							}
						}
					} else {
						USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						mt_state = CM_MoveToPoint(Next_Point, 0);
					}
#else
					mt_state = CM_MoveToPoint(Next_Point, 0);
#endif

					if (Work_Timer - work_timer_start > cleanning_time_allowed) {
						USPRINTF("Cleanning timeout %d.\n", cleanning_time_allowed);
						go_home = 1;
					}

					if (mt_state == MT_Battery) {
						return 0;
					} else if (mt_state == MT_Remote_Home) {
						go_home = 1;
						Stop_Brifly();
					} else if (mt_state == MT_Remote_Clean || mt_state == MT_Cliff || mt_state == MT_Key_Clean) {
						Disable_Motors();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return 0;
					} else if (mt_state == MT_Battery_Home) {
						Display_Battery_Status(Display_Low);
						go_home = 1;
						Stop_Brifly();
					}
					if(VirtualWall_Path_Error==1)
					{
						VirtualWall_Path_Error = 0;
						virtral_move_to_exit = 1;
						state = 0;						
					}
					else
					{				
						state = -1;
					}
				}
				/***************************2.2-2.2-3 Start State 1 End*************************/

				/***************************2.2-2.2-4 Start State 2*************************/
				//Trapped, escape mode
				else if (state == 2) {
#ifdef ZONE_WALLFOLLOW	
						/*------------------- Edit By ZZ --------------------------*/
						//参照国外版小房间模式的脱困方法
						escape_trapped_timer_ZZ = Work_Timer;

						while((state = Map_Wall_Follow(Map_Wall_Follow_Escape_Trapped_ZZ)) == 100);//绕孤岛后直行沿墙
						Stop_Brifly();
						if ( g_map_touring_cancel == 1 ) {
							return 0;
						}						
						if ( go_home == 1 ) {
							continue;
						}
						
						/*----------------- Edit By ZZ -----------------*/
						if(Get_Clean_Mode() == Clean_Mode_Userinterface)
						{
							USPRINTF("%s %d: Clean mode:%d\n",__FUNCTION__,__LINE__,Get_Clean_Mode());
							return 0;
						}
						/*************************************************/
						
						if((state == 0)	&& (WallFollowARoundFlag == 0))//在转一圈之前脱困
						{
							state = -1;
							USPRINTF("%s %d:Program Run at here!\n",__FUNCTION__,__LINE__);
							continue;						
						}						
						if((state == 0)	&& (WallFollowARoundFlag == 1))//转完一圈
						{
							state = 0;
							path_targets_clear_list();
							continue;
						}
						
						if(Zone_Run_Time>1800)//15min
						{
							Zone_Run_Time = 0;
							path_targets_clear_list();
							state = 0;
							continue;
						}
					
						//超时认为脱困不成功，接着下面的程序
						/**************************************************************/						
					path_targets_clear_list();

					//Set boundary CLEANED
					Zone_SetCurrentZoneBoundary(CLEANED, 1);

					USPRINTF("Trapped! Start new wall follow!\n");

					if ( WFM_IsMiniRoom() != 1 ) {
						//Add trapped zone idx
						Zone_AddTrappedZoneIdx();
#ifdef STOP_WALL_FOLLOW_M1
						//Set zone is not continual
						Zone_SetZoneContinual(0);
#endif
					} else {
						WFM_SetMiniRoomChecking(1);
					}

					break;
#else
					state = Map_Wall_Follow(Map_Wall_Follow_Escape_Trapped);
					Stop_Brifly();
					if ( g_map_touring_cancel == 1 ) {
						return 0;
					}
					if ( go_home == 1 ) {
						break;
					}
					if (state == 2) {
						Disable_Motors();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return 0;
					}
					state = -1;
#endif
				}
				/***************************2.2-2.2-4 Start State 2 End*************************/

				/***************************2.2-2.2-5 Start Other State*************************/
				else {
					state = -1;
				}
				/***************************2.2-2.2-5 Start Other State End*************************/
			}
			/***************************2.2-2 Normal Cleaning End***************************/

			/***************************2.3 Last Common Process*************************/
			if (Work_Timer - work_timer_start > cleanning_time_allowed) {
				USPRINTF("Cleanning timeout %d.\n", cleanning_time_allowed);
				go_home = 1;
			}
			if (mt_state == MT_Battery) {
				return 0;
			} else if (mt_state == MT_Remote_Home) {
				go_home = 1;
				Stop_Brifly();
			} else if (mt_state == MT_Remote_Clean || mt_state == MT_Cliff || mt_state == MT_Key_Clean) {
				Disable_Motors();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return 0;
			} else if (mt_state == MT_Battery_Home) {
				Display_Battery_Status(Display_Low);
				go_home = 1;
				Stop_Brifly();
			}
			/***************************2.3 Last Common Process End*************************/
		}
		/*************************************2 Cleaning Main Loop End*************************************/
	}
	/*****************************************************Cleaning End*****************************************************/

}

/*
 * Robot move to target cell
 * @param x	cell x
 * @param y	cell y
 * @param mode 2: Dynamic change cells near target cell
 *             1: with escape mode, not finish
 *             0: no escape mode
 * @return	-2: Robot is trapped
 *		-1: Robot cannot move to target cell
 *		1: Robot arrive target cell
 */
int8_t CM_MoveToCell( int16_t x, int16_t y, uint8_t mode, uint8_t length, uint8_t step )
{
	Point32_t		Next_Point;
	int8_t		pathFind;
	int16_t		i, j, k;
	uint16_t	last_dir, offsetIdx = 0;
	Point16_t	tmp, pos;
	MapTouringType	mt_state = MT_None;
	static uint8_t MapOptimizationFlag = 0;
	static Point16_t MapOptimizationCell;
	
	LED_Blink = 0;

	if (is_block_accessible(x, y) == 0) {
		USPRINTF("%s %d: target is blocked.\n\n", __FUNCTION__, __LINE__);
		Map_Set_Cells(ROBOT_SIZE, x, y, CLEANED);
	}

	//Escape mode
	//TODO: Escape
	if ( mode ==  1 ) {
		USPRINTF("%s %d Path Find: Escape Mode\n", __FUNCTION__, __LINE__);
		LED_Blink = (remote_go_home != 1 ? 1 : 2);
		pos.X = Map_GetXPos();
		pos.Y = Map_GetYPos();
		pathFind = path_move_to_unclean_area(pos, x, y, &tmp.X, &tmp.Y, 0 );
		LED_Blink = 0;

		return 0;
	} else if ( mode == 2 ) {
		i = j = k = offsetIdx = 0;
		Point16_t relativePosTmp = {0, 0};
		USPRINTF("%s %d Path Find: Dynamic Target Mode, target: (%d, %d)\n", __FUNCTION__, __LINE__, x, y);
		relativePos[k].X = 0;
		relativePos[k].Y = 0;
		k = 1;
		for ( i = -length; i <= length; i += step ) {
			for ( j = -length; j <= length; j += step ) {
				if ( x + i <= xMax && x + i >= xMin &&  y + j <= yMax && y + j >= yMin ) {
					if ( i == 0 && j == 0 ) {
						continue;
					}
					relativePos[k].X = i;
					relativePos[k].Y = j;
					USPRINTF("Id: %d\tPoint: (%d, %d)\n", k, relativePos[k].X, relativePos[k].Y);
					++k;
				}
			}
		}
		USPRINTF("%s %d Size: %d\n", __FUNCTION__, __LINE__, k);

		//Position sort, two case: 1. sort for the previous half size of point; 2. sort the rest.
		//Sort from the nearest point to the farest point, refer to the middle point
		for ( i = 1 ; i < k; ++i) {
			for ( j = 1; j < k - i; ++j ) {
				if ( TwoPointsDistance( relativePos[j].X * 1000,     relativePos[j].Y * 1000,     0, 0 ) >
				     TwoPointsDistance( relativePos[j + 1].X * 1000, relativePos[j + 1].Y * 1000, 0, 0 ) ) {
					relativePosTmp = relativePos[j + 1];
					relativePos[j + 1] = relativePos[j];
					relativePos[j] = relativePosTmp;
				}
			}
		}
		USPRINTF("Bubble sort:\n");
		for ( i = 0; i < k; i++ ) {
			USPRINTF("Id: %d\tPoint: (%d, %d)\tDis:%d\n", i, relativePos[i].X, relativePos[i].Y,
			         TwoPointsDistance( relativePos[i].X * 1000, relativePos[i].Y * 1000, 0, 0 ));
		}

		LED_Blink = (remote_go_home != 1 ? 1 : 2);
		last_dir = path_get_robot_direction();
		pos.X = Map_GetXPos();
		pos.Y = Map_GetYPos();
		pathFind = path_move_to_unclean_area(pos, x + relativePos[0].X, y + relativePos[0].Y, &tmp.X, &tmp.Y, 0 );
		LED_Blink = 0;					
		/*---------------------------- Edit By ZZ --------------------*/
		ResetMapOffset();
		/**************************************************************/			
		//Set cell
		Map_Set_Cells(ROBOT_SIZE, x + relativePos[0].X, y + relativePos[0].Y, CLEANED);

		USPRINTF("%s %d Path Find: %d\n", __FUNCTION__, __LINE__, pathFind);
		USPRINTF("%s %d Target need to go: x:%d\ty:%d\n", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
		USPRINTF("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
		while (1) {
			if ( pathFind == 1 || pathFind == SCHAR_MAX ) {
				path_set_current_pos();
				USPRINTF("%s %d Move to target...\n", __FUNCTION__, __LINE__ );
				Next_Point.X = cellToCount(tmp.X);
				Next_Point.Y = cellToCount(tmp.Y);

				while(GoToTempTargetFlag == 1)
				{
					Touch_Detect();//主要是为了处理wifi和风力切换
				}
				if(GoToTempTargetFlag >= 3)
				{
					IsMovingFlag = 0;
					if(GoToTempTargetFlag == 4)
					{
						if(Touch_Detect())
						{
							Beep(5);
							Set_Clean_Mode(Clean_Mode_Userinterface);
							return -5;
						}
						if (Remote_Key(Remote_Home) && (go_home == 0)) 
						{
							SetHomeRemote();
							go_home = 1;
							remote_go_home = 1;
							return -4;
						}
					}
					else if(GoToTempTargetFlag == 3)//保险杠
					{
						CM_signed_obstacle(0,BumperTemp,0);
						CM_CorBack(COR_BACK_20MM);
						CM_signed_obstacle(0,BumperTemp,0);
					}
					else if(GoToTempTargetFlag == 5)//探地
					{
						CM_signed_obstacle(0,0,CliffTemp);
						CM_CorBack(COR_BACK_20MM);
						CM_signed_obstacle(0,0,CliffTemp);
					}
					else if(GoToTempTargetFlag == 6)//充电座
					{

					}
					GoToTempTargetFlag = 0;
					Wheel_Stop();
					CheckGyroCalibrationTime(120);
					mt_state = MT_None;
				}
				else
				{
					mt_state = CM_MoveToPoint(Next_Point, 0);				
				}

				if(VirtualWall_Path_Error==1)
				{
					VirtualWall_Path_Error = 0;
					return 1;						
				}					
				USPRINTF("%s %d Arrive Target! Now: (%d, %d)\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
				if (mt_state == MT_Battery) {
					USPRINTF("%s %d: low battery is detected, battery < 1200\n", __FUNCTION__, __LINE__);
					return -3;
				} else if (mt_state == MT_Remote_Home) {
					USPRINTF("%s %d: home is pressed\n", __FUNCTION__, __LINE__);
					return -4;
				} else if (mt_state == MT_Remote_Clean || mt_state == MT_Cliff || mt_state == MT_Key_Clean) {
					USPRINTF("%s %d: remote is pressed, clean key is pressed,  or cliff is reached\n", __FUNCTION__, __LINE__);
					return -5;
				} else if (mt_state == MT_Battery_Home) {
					USPRINTF("%s %d: low battery is detected, battery < 1300\n", __FUNCTION__, __LINE__);
					return -6;
				} else if ( mt_state == MT_None ) {
					if ( go_home == 1 && Is_Station() == 1 ) {
						return -7;
					}
				}

				//Arrive exit cell, set < 3 when ROBOT_SIZE == 5
				if ( TwoPointsDistance( x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y, Map_GetXPos(), Map_GetYPos() ) < ROBOT_SIZE / 2 + 1 ) {
					USPRINTF("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
					USPRINTF("%s %d Destination: x:%d\ty:%d\n", __FUNCTION__, __LINE__, x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y);
					/*------------------- Edit By ZZ -------------------------*/
					IsMovingFlag = 0;
					GoToTempTargetFlag = 0;
					Wheel_Stop();
					ResetMapOffset();
					/******************************************************************/
					return 1;
				}

				if (is_block_accessible(x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y) == 0) {
					USPRINTF("%s %d: Target is blocked. Try to find new target.\n", __FUNCTION__, __LINE__);
					pathFind = -2;
					/*------------------- Edit By ZZ -------------------------*/
					Wheel_Stop();
					IsMovingFlag = 0;
					GoToTempTargetFlag = 0;
					ResetMapOffset();
					/******************************************************************/
					continue;
				}

				LED_Blink = (remote_go_home != 1 ? 1 : 2);
				last_dir = path_get_robot_direction();
				pos.X = Map_GetXPos();
				pos.Y = Map_GetYPos();
				pathFind = path_move_to_unclean_area(pos, x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y,
				                                      &tmp.X, &tmp.Y, last_dir );
							
				/*---------------------------- Edit By ZZ ---------------------------*/
				ResetMapOffset();
				if(GoToTempTargetFlag >= 3)//思考时前行过程中遇到障碍物,重新计算路径
				{
					IsMovingFlag = 0;
					if(GoToTempTargetFlag == 4)
					{
						if(Touch_Detect())
						{
							Beep(5);
							Set_Clean_Mode(Clean_Mode_Userinterface);
							return -5;
						}
						if (Remote_Key(Remote_Home) && (go_home == 0)) 
						{
							SetHomeRemote();
							go_home = 1;
							remote_go_home = 1;
							continue;
						}
					}
					else if(GoToTempTargetFlag == 3)//保险杠
					{
						CM_signed_obstacle(0,BumperTemp,0);
						CM_CorBack(COR_BACK_20MM);
						CM_signed_obstacle(0,BumperTemp,0);
					}
					else if(GoToTempTargetFlag == 5)//探地
					{
						CM_signed_obstacle(0,0,CliffTemp);
						CM_CorBack(COR_BACK_20MM);
						CM_signed_obstacle(0,0,CliffTemp);
					}
					else if(GoToTempTargetFlag == 6)//充电座
					{

					}
					Wheel_Stop();					
					GoToTempTargetFlag = 0;
					last_dir = path_get_robot_direction();
					pos.X = Map_GetXPos();
					pos.Y = Map_GetYPos();
					pathFind = path_move_to_unclean_area(pos, x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y,
				                                      &tmp.X, &tmp.Y, last_dir );
				}				
				if(pathFind == -2 || pathFind == -1)
				{
					Wheel_Stop();
					IsMovingFlag = 0;
					GoToTempTargetFlag = 0;
					USPRINTF("pathFind :%d\n",pathFind);
				}
				
				LED_Blink = 0;

				if (CM_CheckLoopBack(tmp) == 1) {
					return -2;
				}
				USPRINTF("%s %d Path Find: %d, target: (%d, %d)\n", __FUNCTION__, __LINE__, pathFind,
				         x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y);
				USPRINTF("%s %d Target need to go: x:%d\ty:%d\n", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
				USPRINTF("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));
			} else if ( pathFind == -2 || pathFind == -1 ) {
				//Add offset
				offsetIdx++;
				if ( relativePos[offsetIdx].X == 0 && relativePos[offsetIdx].Y == 0 )
					offsetIdx++;

				if ( offsetIdx >= k ) {
					return -2;
				}

				LED_Blink = (remote_go_home != 1 ? 1 : 2);
				last_dir = path_get_robot_direction();
				pos.X = Map_GetXPos();
				pos.Y = Map_GetYPos();
				pathFind = path_move_to_unclean_area(pos, x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y,
				                                      &tmp.X, &tmp.Y, last_dir );
				LED_Blink = 0;

				if (Touch_Detect()) {
					Set_Touch();
					CM_TouringCancel();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return -2;
				}

				if (Get_Cliff_Trig() == (Status_Cliff_All)) {
					USPRINTF("%s %d: robot is taken up.\n", __FUNCTION__, __LINE__);
					Stop_Brifly();
					return -2;
				}

				USPRINTF("%s %d Path Find: %d, %d Target Offset: (%d, %d)\n", __FUNCTION__, __LINE__, pathFind, offsetIdx,
				         relativePos[offsetIdx].X, relativePos[offsetIdx].Y);
				USPRINTF("%s %d Path Find: %d, target: (%d, %d)\n", __FUNCTION__, __LINE__, pathFind,
				         x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y);
			} else {
				return pathFind;
			}
		}
	}	
	else {//Normal mode
		USPRINTF("%s %d Path Find: Normal Mode, target: (%d, %d)\n", __FUNCTION__, __LINE__, x, y);
		LED_Blink = (remote_go_home != 1 ? 1 : 2);
		last_dir = path_get_robot_direction();
		pos.X = Map_GetXPos();
		pos.Y = Map_GetYPos();
		pathFind = path_move_to_unclean_area(pos, x, y, &tmp.X, &tmp.Y, 0 );
		LED_Blink = 0;

		USPRINTF("%s %d Path Find: %d\n", __FUNCTION__, __LINE__, pathFind);
		USPRINTF("%s %d Target need to go: x:%d\ty:%d\n", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
		USPRINTF("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());

		//Note that path_move_to_unclean_area only can get the next cell to the destination cell
		while ( pathFind == 1 || pathFind == SCHAR_MAX ) {
			path_set_current_pos();

			USPRINTF("%s %d Move to target...\n", __FUNCTION__, __LINE__ );
			Next_Point.X = cellToCount(tmp.X);
			Next_Point.Y = cellToCount(tmp.Y);

			mt_state = CM_MoveToPoint(Next_Point, 0);

			if(VirtualWall_Path_Error==1)
			{
				VirtualWall_Path_Error = 0;
				return 1;
			}
			USPRINTF("%s %d Arrive Target! Now: (%d, %d)\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());

			if (mt_state == MT_Battery) {
				USPRINTF("%s %d: low battery is detected, battery < 1200\n", __FUNCTION__, __LINE__);
				return -3;
			} else if (mt_state == MT_Remote_Home) {
				USPRINTF("%s %d: home is pressed\n", __FUNCTION__, __LINE__);
				return -4;
			} else if (mt_state == MT_Remote_Clean || mt_state == MT_Cliff || mt_state == MT_Key_Clean) {
				USPRINTF("%s %d: remote is pressed, clean key is pressed,  or cliff is reached\n", __FUNCTION__, __LINE__);
				return -5;
			} else if (mt_state == MT_Battery_Home) {
				USPRINTF("%s %d: low battery is detected, battery < 1300\n", __FUNCTION__, __LINE__);
				return -6;
			} else if ( mt_state == MT_None ) {
				if ( go_home == 1 && Is_Station() == 1 ) {
					return -7;
				}								
			}

			//Arrive exit cell, set < 3 when ROBOT_SIZE == 5
			if ( TwoPointsDistance( x, y, Map_GetXPos(), Map_GetYPos() ) < ROBOT_SIZE / 2 + 1 ) {
				USPRINTF("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
				USPRINTF("%s %d Destination: x:%d\ty:%d\n", __FUNCTION__, __LINE__, x, y);
				/*------------------- Edit By ZZ -------------------------*/
					IsMovingFlag = 0;
					GoToTempTargetFlag = 0;
					Wheel_Stop();
					ResetMapOffset();
					/******************************************************************/
				return 1;
			}

			if (is_block_accessible(x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y) == 0) {
					USPRINTF("%s %d: Target is blocked. Try to find new target.\n", __FUNCTION__, __LINE__);
					pathFind = -2;
					/*------------------- Edit By ZZ -------------------------*/
					Wheel_Stop();
					IsMovingFlag = 0;
					GoToTempTargetFlag = 0;
					ResetMapOffset();
					/******************************************************************/
					continue;
				}

			LED_Blink = (remote_go_home != 1 ? 1 : 2);
			last_dir = path_get_robot_direction();
			pos.X = Map_GetXPos();
			pos.Y = Map_GetYPos();
			pathFind = path_move_to_unclean_area(pos, x, y, &tmp.X, &tmp.Y, last_dir );
			LED_Blink = 0;
	    /*---------------------------- Edit By ZZ ---------------------------*/
				ResetMapOffset();
				if(GoToTempTargetFlag >= 3)//思考时前行过程中遇到障碍物,重新计算路径
				{
					IsMovingFlag = 0;
					if(GoToTempTargetFlag == 4)
					{
						if(Touch_Detect())
						{
							Beep(5);
							Set_Clean_Mode(Clean_Mode_Userinterface);
							return -5;
						}
						if (Remote_Key(Remote_Home) && (go_home == 0)) 
						{
							SetHomeRemote();
							go_home = 1;
							remote_go_home = 1;
							continue;
						}
					}
					else if(GoToTempTargetFlag == 3)//保险杠
					{
						CM_signed_obstacle(0,BumperTemp,0);
						CM_CorBack(COR_BACK_20MM);
						CM_signed_obstacle(0,BumperTemp,0);
					}
					else if(GoToTempTargetFlag == 5)//探地
					{
						CM_signed_obstacle(0,0,CliffTemp);
						CM_CorBack(COR_BACK_20MM);
						CM_signed_obstacle(0,0,CliffTemp);
					}
					else if(GoToTempTargetFlag == 6)//充电座
					{

					}
					Wheel_Stop();
					CheckGyroCalibrationTime(120);
					GoToTempTargetFlag = 0;
					last_dir = path_get_robot_direction();
					pos.X = Map_GetXPos();
					pos.Y = Map_GetYPos();
					pathFind = path_move_to_unclean_area(pos, x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y,
				                                      &tmp.X, &tmp.Y, last_dir );
				}
				
				if(pathFind == -2 || pathFind == -1)
				{
					Wheel_Stop();
					IsMovingFlag = 0;
					GoToTempTargetFlag = 0;
					USPRINTF("pathFind :%d\n",pathFind);
					/*------------------------------- Edit By ZZ ------------------------*/
					USPRINTF("MoveToCell Trapped!  Optimization the map of cuurent cell:(%d,%d)!\n",Map_GetXPos(),Map_GetYPos());
					if(MapOptimizationFlag == 0)
					{
						USPRINTF("Optimization map!\n");
						Map_Set_Cells_IgnoreCells(ROBOT_SIZE+1,Map_GetXPos(),Map_GetYPos(),CLEANED,BLOCKED_BOUNDARY);
						MapOptimizationCell = Map_GetCurrentCell();
						MapOptimizationFlag = 1;
						pathFind = 1;
					}
					else if((Map_GetXPos() != MapOptimizationCell.X) || (Map_GetYPos() != MapOptimizationCell.Y))
					{
						USPRINTF("Reset MapOptimizationFlag!\n");
						MapOptimizationFlag = 0;
					}
					/**********************************************************************/
				}
				/*******************************************************************/	
			USPRINTF("%s %d Path Find: %d, target: (%d, %d)\n", __FUNCTION__, __LINE__, pathFind, x, y);
			USPRINTF("%s %d Target need to go: x:%d\ty:%d\n", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
			USPRINTF("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));
		}
		return pathFind;
	}
}

/*-------------- Move Back -----------------------------*/
void CM_CorBack(uint16_t dist)
{
	uint32_t SP = 10;
	uint16_t Counter_Watcher = 0;
	uint8_t motor_check=0;
	USPRINTF("%s %d: Moving back...\n", __FUNCTION__, __LINE__);
	Stop_Brifly();
	Set_Dir_Backward();
	Set_Wheel_Speed(RUN_SPEED_4, RUN_SPEED_4);
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;
	while ((Get_LeftWheel_Step() < dist) || (Get_RightWheel_Step() < dist)) {
		delay(10);
		Counter_Watcher++;
		SP = 8 + Counter_Watcher / 100;
		SP = (SP > RUN_SPEED_8) ? RUN_SPEED_8 : SP;

		Set_Wheel_Speed(SP, SP);
		if (Counter_Watcher > 3000) {
			if (Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			break;
		}
		if (Touch_Detect()) {
			break;
		}

		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			break;
		}
		
	}
	Reset_TempPWM();
	USPRINTF("%s %d: Moving back done!\n", __FUNCTION__, __LINE__);
}

void CM_SetGoHome(uint8_t remote) {
	go_home = 1;
	if (remote == 1) {
		remote_go_home = 1;
	}
}

void CM_TouringCancel(void)
{
	g_map_touring_cancel = 1;
}

void CM_SetGyroOffset(int16_t offset)
{
	map_gyro_offset = offset;
}

void CM_SetHome(int32_t x, int32_t y) {
	USPRINTF("%s %d: set new reachable home: (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x), countToCell(y));
	g_home_point.X = x;
	g_home_point.Y = y;
}

void CM_SetStationHome(void) {
	
	if(Get_Station_Position()==1)
	{
		if ( TwoPointsDistance(countToCell(g_home_point.X), countToCell(g_home_point.Y), Map_GetXPos(), Map_GetYPos()) > 20 )
		{
			g_home_point.X = Map_GetXCount();
			g_home_point.Y = Map_GetYCount();
		}
	}
	else
	{
		g_home_point.X = Map_GetXCount();
		g_home_point.Y = Map_GetYCount();
	}

#ifdef ZONE_WALLFOLLOW
	if ( isSingleRoom == 0 ) {
		USPRINTF("%s %d: from_station: %d\tstation_zone: %d\tzone diff: %d\n", __FUNCTION__, __LINE__, from_station, station_zone, Zone_GetCurrentZoneIdx() - station_zone);
		if (from_station == 2 || (station_zone != -1 && Zone_GetCurrentZoneIdx() - station_zone > 10 && from_station == 1)) {
			from_station = 2;
		} else {
			if (station_zone == -1) {
				station_zone = Zone_GetCurrentZoneIdx();
				return;
			}
			from_station = 1;
		}
	}
#else

#endif
}

#ifdef ZONE_WALLFOLLOW
void CM_ResetBoundaryBlocks(void)
{
	int16_t i, j, x_min, x_max, y_min, y_max;
	Zone_GetRange(&x_min, &x_max, &y_min, &y_max);
	USPRINTF("%s %d: x_min: %d\tx_max: %d\ty_min: %d\ty_max: %d\n", __FUNCTION__, __LINE__, x_min, x_max, y_min, y_max);
	for (i = x_min - BOUNDARY_INCREMENT; i <= x_max + BOUNDARY_INCREMENT; i++) {
		for (j = y_min - BOUNDARY_INCREMENT; j <= y_max + BOUNDARY_INCREMENT; j++) {
			if (Map_GetCell(MAP, i, j) == BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, cellToCount(i), cellToCount(j), UNCLEAN);
			}
		}
	}
}

void CM_AddTargets(Point16_t zone)
{
	int16_t i, m, n;	
	int32_t	x, y;
	#ifdef VIRTUAL_WALL
	if(WallFollowMulti_Pass_VirtualWall==1)
	{
		WallFollowMulti_Pass_VirtualWall = 0;
		Draw_VirtualWall_Line(Start_WallFollowMulti_VirtualWall_Point,End_WallFollowMulti_VirtualWall_Point,BLOCKED_BUMPER);//BLOCKED_BOUNDARY);			
	}
	#endif
	
	USPRINTF("%s %d: too less target (%d), adding by wall following path.\n", __FUNCTION__, __LINE__, path_targets_get_count());
	for ( i = 0; i < alignmentPtr; i++ ) {
		#ifdef RIGHT_WALL
		CM_count_normalize(angleAlignment[i] - (Zone_GetZoneSize() == 1 ? map_gyro_offset : 0),  CELL_SIZE_2, 0, &x, &y);
		#endif
		x = positionAlignment[i].X + x - Map_GetXCount();
		y = positionAlignment[i].Y + y - Map_GetYCount();
		if (path_targets_get_last(&m, &n) == 1) {
			if (TwoPointsDistance(countToCell(x), countToCell(y), m, n) >= 2) {
				USPRINTF("%s %d: testing adding target (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x), countToCell(y));
				if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == UNCLEAN ) {
					path_targets_add_one(countToCell(x), countToCell(y), 0);
				}
			}
		} else {
			USPRINTF("%s %d: testing adding target (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x), countToCell(y));
			if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == UNCLEAN ) {
				path_targets_add_one(countToCell(x), countToCell(y), 0);
			}
		}
	}
	ShortestPath_Last_Cell.X = 250;
	ShortestPath_Last_Cell.Y = 250;	
	Zone_Run_Time =0;
}
#endif

uint8_t CM_IsLowBattery(void) {
	return lowBattery;
}

uint8_t CM_CheckLoopBack( Point16_t target ) {
	uint8_t retval = 0;
	/*------------------------- Edit By ZZ --------------------------*/
	if (( target.X == positions[2].x && target.Y == positions[2].y &&
	     target.X == positions[4].x && target.Y == positions[4].y )	//两个点来回循环
		|| ( target.X == positions[1].x && target.Y == positions[1].y &&
	     target.X == positions[4].x && target.Y == positions[4].y ))//三个点来回循环
	{
	/******************************************************************/
		USPRINTF("%s %d Possible loop back (%d, %d)\n", __FUNCTION__, __LINE__, target.X, target.Y);
		retval  = 1;
	}
	return retval;
}

void CM_Matrix_Rotate(int32_t x_in, int32_t y_in, int32_t *x_out, int32_t *y_out, double theta)
{
	double dd, de;

	dd = (double) x_in;
	de = (double) y_in;

	Matrix_Rotate(&dd, &de, theta);

	*x_out = (int32_t)dd;
	*y_out = (int32_t)de;
}

uint8_t CM_IsSingleRoomMode(void) {
	return isSingleRoom;
}

void CM_SetSingleRoomMode( uint8_t val ) {
	isSingleRoom = val;
}

uint8_t CM_IsFromStation(void)
{
	return from_station;
}

MapTouringType CM_handleExtEvent()
{
	/* Check low battery event, if battery is low, go home directly. */
	if ((Check_Bat_Home() == 1) && go_home != 1) {
		lowBattery = 1;
		if ( Get_VacMode() == Vac_Max ) {
			Switch_VacMode();
		}
		Stop_Brifly();
		USPRINTF("%s %d: low battery, battery < 1300 is detected.\n", __FUNCTION__, __LINE__);
		remote_go_home = 1;
		return MT_Battery_Home;
	}

	/* Check key press events. */
	if (Touch_Detect()) {
		Stop_Brifly();
		USPRINTF("%s %d: clean key is pressed.\n", __FUNCTION__, __LINE__);
		Beep(5);
		Reset_Touch();
		CM_TouringCancel();			
		Set_Clean_Mode(Clean_Mode_Userinterface);
		return MT_Key_Clean;
	}

	/* Check remote events. */
	if (Get_Rcon_Remote() != 0) {
		/* Check remote home key press event, if home key is pressed, go home directly. */
		if (Remote_Key(Remote_Home) && (go_home == 0)) {
			Set_BLDC_Speed(Vac_Speed_NormalL);
			Check_Bat_SetMotors(Home_Vac_Power, Home_SideBrush_Power, Home_MainBrush_Power);
			Deceleration();
			Stop_Brifly();
			USPRINTF("%s %d: remote home is pressed.\n", __FUNCTION__, __LINE__);
			remote_go_home = 1;
			go_home = 1;
			return MT_Remote_Home;
		}

		/*
		 * Check remote spot key press event, if spot key is pressed,
		 * change to spot mode, after spot mode finished, back to zig-zag clean.
		 */
		#ifdef BLDC_INSTALL
		if (Remote_Key(Remote_Max)) {
			if (lowBattery == 0) {
				Switch_VacMode();
			}
		}
		#endif

		/* Check remote clean key press event, if clean key is pressed, stop robot directly. */
		if (Remote_Key(Remote_Clean)) {
			Stop_Brifly();
			Set_Touch();
			CM_TouringCancel();
			USPRINTF("%s %d: remote clean is pressed.\n", __FUNCTION__, __LINE__);
			return MT_Remote_Clean;
		}
		Reset_Rcon_Remote();
	}

	/* Check whether robot is taken up. */
	if (Get_Cliff_TT() == (Status_Cliff_All)) {
		USPRINTF("%s %d: robot is taken up.\n", __FUNCTION__, __LINE__);
		Stop_Brifly();
		return MT_Cliff;
	}
	return MT_None;
}

void Set_Station_Position(uint8_t data)
{
	station_position =  data;
}

uint8_t Get_Station_Position(void)
{
	return station_position;
}

void CheckGyroCalibrationTime(uint16_t time_limited)
{
	if(moveToPointTimeCount > time_limited)
	{
		Gyro_Calibration_Cmd(ENABLE);
		delay(6000);
		moveToPointTimeCount = 0;
	}
}


