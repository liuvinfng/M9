/**
******************************************************************************
* @file    AI Cleaning Robot
* @author  Wfliu
* @version V1.0
* @date    17-Nov-2011
* @brief   Move near the wall on the left in a certain distance
******************************************************************************
* <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "Display.h"
#include "Movement.h"
#include "Rcon.h"
#include "TouchPad.h"
#include "MyMath.h"
#include "math.h"
#include "PathPlanning.h"
#include "Map.h"
#include "Gyro.h"
#include "CorMove.h"
#include "PP_rounding.h"
#include "WallFollowMulti.h"

#ifdef PP_ROUNDING_ALIGNMENT

#define ALIGNMENT_COUNT				(30)
#define ALIGNMENT_POLYFIT_MAX_ERROR		(20)
#define ALIGNMENT_POLYFIT_MAX_TIME		(5)
#define ALIGNMENT_DATA_STEP			(50)
#define LINES_SIZE				(5)
#define MAX_SEGMENT_DELTA_ANGLE			(100)
#define MAX_WALL_DELTA_ANGLE			(200)
#define MIN_SEGMENT_SIZE			(10)

static int16_t pp_angle_offset = 0;
int16_t pp_angle_offset_tmp[LINES_SIZE] = {0};
uint16_t pp_angles[ALIGNMENT_COUNT] = {0};

uint16_t pp_positions_ptr = 0;
Point32_t pp_positions[ALIGNMENT_COUNT] = {{0, 0}};

Point32_t pp_point32_ary_tmp[ALIGNMENT_COUNT] = {{0, 0}};
uint16_t pp_u16_ary_tmp[ALIGNMENT_COUNT] = {0};

static LineABC pp_lines[LINES_SIZE] = {{0.0, 0.0, 0.0}};
static uint8_t pp_lines_ptr = 0;
#endif

typedef enum {
	ACTION_NONE	= 0x01,
	ACTION_GO	  = 0x02,
	ACTION_BACK	= 0x04,
	ACTION_LT	  = 0x08,
	ACTION_RT	  = 0x10,
} ActionType;

uint8_t	should_mark = 0;

void pp_rounding_move_back(uint16_t dist);
void pp_rounding_Big_move_back(uint8_t dir,uint16_t angle);
void pp_rounding_Small_move_back(uint8_t dir,uint16_t angle);
void pp_update_position(uint16_t heading_0, int16_t heading_1, int16_t left, int16_t right);

extern int16_t WheelCount_Left, WheelCount_Right;
extern void CM_update_map_bumper(ActionType action, uint8_t bumper);

PpRoundingType	rounding_type;

extern uint8_t IsMovingFlag;
extern uint8_t GoToTempTargetFlag;	
extern uint8_t PP_RoundingMovingFlag;

#define STOP_BRIFLY	{				\
				Wheel_Stop();		\
			}

void pp_update_position(uint16_t heading_0, int16_t heading_1, int16_t left, int16_t right) {
	int32_t	i, j;
		if (should_mark == 1) {
			if (rounding_type == PP_ROUNDING_LEFT) {
				i = Map_GetRelativeX(heading_0, CELL_SIZE_2, 0);
				j = Map_GetRelativeY(heading_0, CELL_SIZE_2, 0);
				if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BOUNDARY) {
					Map_SetCell(MAP, i, j, BLOCKED_OBS);
				}
			} else if (rounding_type == PP_ROUNDING_RIGHT) {
				i = Map_GetRelativeX(heading_0, -CELL_SIZE_2, 0);
				j = Map_GetRelativeY(heading_0, -CELL_SIZE_2, 0);
				if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BOUNDARY) {
					Map_SetCell(MAP, i, j, BLOCKED_OBS);
				}
			}
		}
}

int8_t pp_rounding_update()
{
	pp_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1), WheelCount_Left, WheelCount_Right); 

	return 0;
}

void pp_rounding_turn(uint8_t dir, uint16_t speed, uint16_t angle)
{
	uint16_t Counter_Watcher = 0;
	uint32_t step;
	uint8_t motor_check=0;
	STOP_BRIFLY;

	if (dir == 0) {
		Reset_LeftWheel_Step();
		Set_Dir_Left();
	} else {
		Reset_RightWheel_Step();
		Set_Dir_Right();
	}

	USPRINTF("%s %d: %d %d %d\n", __FUNCTION__, __LINE__, dir, speed, angle);
	Set_Wheel_Speed(speed, speed);
	Counter_Watcher = should_mark = 0;
	angle = ANGLE_MUL*angle;
	while (1) {
		pp_rounding_update();
		delay(10);
		step = (dir == 0 ? Get_LeftWheel_Step() : Get_RightWheel_Step());
		if (step >= angle) {
			USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
			break;
		}

		Counter_Watcher++;
		if (Counter_Watcher > 4000) {
			if (Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
			break;
		}
		if (Is_Turn_Remote()){
			USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
			break;
		}
		if (Touch_Detect()) {
			USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
			break;
		}
	  if(Get_Bumper_Status())return;	
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
			break;
		}
	}
	pp_rounding_update();

	should_mark = 1;
	USPRINTF("%s %d: %d %d %d\n", __FUNCTION__, __LINE__, dir, speed, angle);
}

void pp_rounding_move_back(uint16_t dist)
{
	uint16_t Counter_Watcher = 0;
	uint8_t motor_check=0;
	STOP_BRIFLY;
	pp_rounding_update();
	Set_Dir_Backward();
	Set_Wheel_Speed(RUN_SPEED_6, RUN_SPEED_6);
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;

	should_mark = 0;

	while ((Get_LeftWheel_Step() < dist) || (Get_RightWheel_Step() < dist)) {
		pp_rounding_update();
		delay(10);
		Counter_Watcher++;
		if (Counter_Watcher > 3000) {
			if(Is_Encoder_Fail()) {
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
	pp_rounding_update();
	Reset_TempPWM();
	should_mark = 1;
}

//0:left 1:right
void pp_rounding_Big_move_back(uint8_t dir,uint16_t angle)
{
	uint16_t Counter_Watcher = 0;
	uint8_t motor_check=0,flag=1;
	STOP_BRIFLY;
	pp_rounding_update();
	Set_Dir_Backward();
	Set_Wheel_Speed(RUN_SPEED_3, RUN_SPEED_3);
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;

	should_mark = 0;
	while ((Get_RightWheel_Step() < 12*DISTANCE_1CM))
	{
		if(!dir)
		{
			Set_RightWheel_Step(Get_LeftWheel_Step());
		}
		pp_rounding_update();
		if(Get_RightWheel_Step()<3*DISTANCE_1CM)
		{
			Set_Wheel_Speed(RUN_SPEED_5,RUN_SPEED_5);
		}
		else if((Get_RightWheel_Step()<=6*DISTANCE_1CM)&&flag)
		{
			if (Get_Bumper_Status())
			{
				Set_RightWheel_Step(6*DISTANCE_1CM);
				Reset_TempPWM();
				flag=0;
			}
			if(dir)//right
			{
				Set_Wheel_Speed(0,RUN_SPEED_7);			
			}
			else
			{
				Set_Wheel_Speed(RUN_SPEED_7,0);				
			}
		}
		else
		{
			Set_Wheel_Speed(RUN_SPEED_7,RUN_SPEED_7);
		}
		
		delay(10);
		Counter_Watcher++;
		if (Counter_Watcher > 5000) {
			if(Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if (Touch_Detect()) {
			return;
		}	
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}
	
	if(dir)
	{
		Set_Dir_Left();
		Set_Wheel_Speed(RUN_SPEED_5, RUN_SPEED_9);	
	}
	else
	{
		Set_Dir_Right();
		Set_Wheel_Speed(RUN_SPEED_9, RUN_SPEED_5);		
	}
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;

	while ((Get_RightWheel_Step() < angle)) 
	{
		if(!dir)
		{
			Set_RightWheel_Step(Get_LeftWheel_Step());
		}		
		pp_rounding_update();
		delay(10);
		Counter_Watcher++;
		if (Counter_Watcher > 5000) {
			if(Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if (Touch_Detect()) {
			return;
		}
		if (Get_Bumper_Status()) {
			Reset_TempPWM();
			return;
		}
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}		
	
	pp_rounding_update();
	Reset_TempPWM();
	should_mark = 1;
}

void pp_rounding_Small_move_back(uint8_t dir,uint16_t angle)
{
	uint16_t Counter_Watcher = 0;
	uint8_t motor_check=0;
	STOP_BRIFLY;
	pp_rounding_update();
	Set_Dir_Backward();
	if(dir)
	{
		Set_Wheel_Speed(RUN_SPEED_4, RUN_SPEED_5);	
	}
	else
	{
		Set_Wheel_Speed(RUN_SPEED_5, RUN_SPEED_4);	
	}
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;

	should_mark = 0;
	
	while ((Get_RightWheel_Step() < 5*DISTANCE_1CM)) 
	{
		if(!dir)
		{
			Set_RightWheel_Step(Get_LeftWheel_Step());
		}		
		pp_rounding_update();
		delay(10);
		Counter_Watcher++;
		if (Counter_Watcher > 5000) {
			if(Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if (Touch_Detect()) {
			return;
		}	
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}
	
	Set_Wheel_Step(0, 0);
	Counter_Watcher = 0;

	while ((Get_RightWheel_Step() < 7*DISTANCE_1CM)) 
	{
		if(!dir)
		{
			Set_RightWheel_Step(Get_LeftWheel_Step());
		}		
		if(Get_RightWheel_Step()<3*DISTANCE_1CM)
		{
			if(dir)Set_Dir_Left();
			else Set_Dir_Right();
			Set_Wheel_Speed(RUN_SPEED_5, RUN_SPEED_5);
		}
		else if(Get_RightWheel_Step()<7*DISTANCE_1CM)
		{
			Set_Dir_Forward();
//			Set_Wheel_Speed(RUN_SPEED_5, RUN_SPEED_5);
			if(dir)Set_Wheel_Speed(0, RUN_SPEED_5);//0, RUN_SPEED_5
			else Set_Wheel_Speed(RUN_SPEED_5, 0);//RUN_SPEED_5, 0			
		}
		else
		{
			if(dir)Set_Wheel_Speed(0, RUN_SPEED_5);//0, RUN_SPEED_5
			else Set_Wheel_Speed(RUN_SPEED_5, 0);//RUN_SPEED_5, 0
		}
		
		pp_rounding_update();
		delay(10);
		Counter_Watcher++;
		if (Counter_Watcher > 5000) {
			if(Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if (Touch_Detect()) {
			return;
		}
		if (Get_Bumper_Status()) {
			Reset_TempPWM();
			return;
		}
		motor_check = Check_Motor_Current();
		if((motor_check==Check_Left_Wheel)||(motor_check==Check_Right_Wheel)) {
			return;
		}
	}			
	pp_rounding_update();
	Reset_TempPWM();
	should_mark = 1;
}

uint8_t pp_rounding_boundary_check()
{
	uint8_t boundary_reach = 0;
	int16_t j;
	int32_t x, y;

	for (j = -1; boundary_reach == 0 && j <= 1; j++) {
		x = Map_GetRelativeX(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_3);
		y = Map_GetRelativeY(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_3);

		if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY) {
			boundary_reach = 1;
			pp_rounding_update();
			Set_Wheel_Speed(0, 0);
			delay(10);
		}
	}
	return boundary_reach;
}

#ifdef PP_ROUNDING_ALIGNMENT
void pp_rounding_alignment_init(void)
{
        int16_t i = 0;

        for (i = 0; i < ALIGNMENT_COUNT; ++i) {
		pp_positions[i].X = pp_positions[i].Y = pp_angles[i] = 0;
        }
        pp_positions_ptr = 0;

        for ( i = 0; i < LINES_SIZE; ++i ) {
                pp_lines[i].A = pp_lines[i].B = pp_lines[i].C = 0.0;
        }

        pp_lines_ptr = 0;

        pp_angle_offset = 0;
}

uint8_t pp_rounding_calculate_angle_offset(LineABC *lineabc) {
        double	sinTmp, cosTmp;
        uint8_t	lineABCPtr, i, pp_angle_offsetPtr = 0, retval = 0;
        int16_t	angleTmp;

	lineABCPtr = pp_lines_ptr;

	//Find which line is near 0, 90, 180 and 360 degree and save them
	for (i = 0; i < lineABCPtr; ++i) {
		angleTmp = (int16_t)(LineAngle(lineabc[i], 1) * 1800 / PI);
		USPRINTF("%s %d: Line %d Angle: %d\n", __FUNCTION__, __LINE__, i, angleTmp);

		if (abs(angleTmp) < MAX_WALL_DELTA_ANGLE) {
			pp_angle_offset_tmp[pp_angle_offsetPtr] = -angleTmp;
			pp_angle_offsetPtr++;
		} else if (abs(angleTmp) > 900 - MAX_WALL_DELTA_ANGLE) {
			if (angleTmp < 0) {
				angleTmp += 1800;
			}
			pp_angle_offset_tmp[pp_angle_offsetPtr] = 900 - angleTmp;
			pp_angle_offsetPtr++;
		}
	}

	//Calculate mean
	angleTmp = 0; sinTmp = 0.0; cosTmp = 0.0;
	for (i = 0; i < pp_angle_offsetPtr; ++i) {
		sinTmp += sin( (double)pp_angle_offset_tmp[i] * PI / 1800 );
		cosTmp += cos( (double)pp_angle_offset_tmp[i] * PI / 1800 );
		USPRINTF("%s %d: Line Angle %d Offset: %d\n", __FUNCTION__, __LINE__, i, pp_angle_offset_tmp[i]);
	}

	//Set offset
	pp_angle_offset = -(int16_t)( atan2(sinTmp, cosTmp) * 1800 / PI );
	USPRINTF("%s %d: Angle Offset: %d\n", __FUNCTION__, __LINE__, pp_angle_offset);

	if (pp_angle_offsetPtr > 0) {
		retval = 1;
	}
	return retval;
}

//Fit lines
uint8_t pp_find_line( Point32_t *point32AryPtr, uint16_t *u16AryPtr, uint16_t point32Size,
                  uint8_t mode, LineABC *lineabcTmp, Point32_t *cdnTmp, int16_t *int16Tmp, double *angleVec ) {
	uint8_t i = 0, j = 0, retval = 0, fitTimes = 0, meetRequirement = 0;
	double sinTmp = 0.0, cosTmp = 0.0, tmp, vectorX, vectorY, xys = 0.0, xs = 0.0, ys = 0.0, x2s = 0.0, y2s = 0.0;
	uint32_t fitErr;
	uint16_t k;
	Point32_t p1 = {0, 0}, p2 = {0, 0};

	USPRINTF("Begin to find lines...\n")
	while ( point32Size > MIN_SEGMENT_SIZE && fitTimes < ALIGNMENT_POLYFIT_MAX_TIME ) {
		fitTimes++;
		USPRINTF("Times of fitting line: %d\n", fitTimes);

		//Formula
		//Line: y = a * x + b
		//a = ( Sum(x * y) - Sum(x) * Sum(y) / N ) / ( Sum(x^2) - Sum(x)^2 / N );

		xys = 0.0; xs = 0.0; ys = 0.0; x2s = 0.0; y2s = 0.0;

		//Calclate sum of x, y x*y, x*x and y*y
		for ( k = 0; k < point32Size; ++k ) {
			xs  += (double)point32AryPtr[k].X;
			ys  += (double)point32AryPtr[k].Y;
			xys += (double)point32AryPtr[k].X * point32AryPtr[k].Y;
			x2s += (double)point32AryPtr[k].X * point32AryPtr[k].X;
			y2s += (double)point32AryPtr[k].Y * point32AryPtr[k].Y;
		}

		//If not vertical
		tmp = ( xys / xs - ys / point32Size ) / ( x2s / xs - xs / point32Size );
		//USPRINTF("%s %d Tmp: %d\n", __FUNCTION__, __LINE__, (int32_t)(tmp * 1000));
		if ( tmp <= 1.0 && tmp >= -1.0 ) {
			USPRINTF("%s %d FindLine Horizontal!\n", __FUNCTION__, __LINE__);
			lineabcTmp->A = - tmp;
			lineabcTmp->B = 1.0;
			lineabcTmp->C = - ys / point32Size - lineabcTmp->A / point32Size * xs;

			USPRINTF("%s %d: Line: A: %d\tB: %d\tC: %d\n", __FUNCTION__, __LINE__,
							 (int32_t)(lineabcTmp->A * 1000), (int32_t)(lineabcTmp->B * 1000), (int32_t)(lineabcTmp->C * 1000));

		} else if ( tmp > 1.0 || tmp < -1.0 ) {
			USPRINTF("%s %d FindLine Vertical!\n", __FUNCTION__, __LINE__);
			lineabcTmp->A = 1.0;
			lineabcTmp->B = - ( xys / ys - xs / point32Size ) / ( y2s / ys - ys / point32Size );
			lineabcTmp->C = - xs / point32Size - lineabcTmp->B / point32Size * ys;

			USPRINTF("%s %d: Line: A: %d\tB: %d\tC: %d\n", __FUNCTION__, __LINE__,
							 (int32_t)(lineabcTmp->A * 1000), (int32_t)(lineabcTmp->B * 1000), (int32_t)(lineabcTmp->C * 1000));
		}

		//Calculate fit error
		meetRequirement = 1;
		USPRINTF("Fit error of lines: Size: %d\n", point32Size);
		for ( j = 0; j < point32Size; ++j ) {

			fitErr = (uint32_t)( absolute( lineabcTmp->A * (double)(point32AryPtr[j].X) +
			                               lineabcTmp->B * (double)(point32AryPtr[j].Y) + lineabcTmp->C ) /
			                        sqrt( (lineabcTmp->A) * (lineabcTmp->A) + (lineabcTmp->B) * (lineabcTmp->B) ) );

			USPRINTF("Idx: %d\tError: %d\n", j, fitErr);
#ifdef ENABLE_DEBUG
			delay(10);
#endif

			//Delete this point if error too large
			//Mode 1: Will dynamic calculate the error, delete some large error point and find the line again until it converaged
			//Mode Ohters: Only fit the line use the points. If it is larger than the max error, then no line fits these data
			if ( fitErr > ALIGNMENT_POLYFIT_MAX_ERROR ) {
				if ( mode == 1 ) {
					for ( i = j; i < point32Size - 1; ++i ) {
						point32AryPtr[i] = point32AryPtr[i + 1];
						u16AryPtr[i] = u16AryPtr[i + 1];
					}
					USPRINTF("Remove Idx: %d\n", j);
					point32Size--;
					meetRequirement = 0;
					--j;
					//break;
				} else {
					meetRequirement = 0;
					break;
				}
			}
		}

		//all error is smaller enough
		if ( meetRequirement == 1 ) {
			retval = 1;
			break;
		}

		//If mode is not 1, then break
		if ( mode != 1 ) {
			break;
		}
	}

	//If line meets the requirement
	if ( meetRequirement == 1 ) {
		//Calculate mean gyro angle
		for ( k = 0; k < point32Size; ++k ) {
			sinTmp += sin((double)u16AryPtr[k] * PI / 1800);
			cosTmp += cos((double)u16AryPtr[k] * PI / 1800);
		}
		*int16Tmp = (int16_t)( atan2(sinTmp, cosTmp) * 1800 / PI + 3600 ) % 3600;
		USPRINTF("Gyro angle mean: %d\n", *int16Tmp);

		//Set significant point
		cdnTmp->X = point32AryPtr[0].X;
		cdnTmp->Y = point32AryPtr[0].Y;
		USPRINTF("Significant Point: x: %d\ty: %d\n", cdnTmp->X, cdnTmp->Y);

		//Set vector
		tmp = LineAngle(*lineabcTmp, 1);
		if ( (int16_t)(tmp * 1800 / PI) <= 450 && (int16_t)(tmp * 1800 / PI) >= -450 ) {
			p1.X = point32AryPtr[0].X;
			p1.Y = (int32_t)(- lineabcTmp->C - lineabcTmp->A * p1.X);

			p2.X = point32AryPtr[point32Size - 1].X;
			p2.Y = (int32_t)(- lineabcTmp->C - lineabcTmp->A * p2.X);
		} else {
			p1.Y = point32AryPtr[0].Y;
			p1.X = (int32_t)(- lineabcTmp->C - lineabcTmp->B * p1.Y);

			p2.Y = point32AryPtr[point32Size - 1].Y;
			p2.X = (int32_t)(- lineabcTmp->C - lineabcTmp->B * p2.Y);
		}

		vectorX = (double)(p2.X - p1.X);
		vectorY = (double)(p2.Y - p1.Y);

		*angleVec = atan2( vectorY, vectorX );
	}

	return retval;
}

uint8_t pp_rounding_line_segment(LineABC *lineABC)
{
	double		angleVec;
	uint8_t		retval = 0, isFit = 0;
	int16_t		i = 0, j = 0, int16Tmp, delta = 0;
	uint16_t	k = 0, point32Size = 0;

	LineABC		lineabcTmp;
	Point32_t	cdnTmp;

	USPRINTF("%s %d Line Segment...\n", __FUNCTION__, __LINE__);
	for (i = 0; i < pp_positions_ptr - MIN_SEGMENT_SIZE; ++i) {
		//USPRINTF("%s %d Line Segment i: %d\n", __FUNCTION__, __LINE__, i);
		if (lineABC == pp_lines && pp_lines_ptr >= LINES_SIZE) {
			break;
		}

		//Find line segment
		for (j = i + 1; j < pp_positions_ptr; ++j) {
			//USPRINTF("%s %d j: %d\n", __FUNCTION__, __LINE__, j);
			delta = degreeDeltaAngleVector( pp_angles[j], pp_angles[i] );
			if (delta >= MAX_SEGMENT_DELTA_ANGLE || delta < -MAX_SEGMENT_DELTA_ANGLE || j == pp_positions_ptr - 1) {
				if (j - i > MIN_SEGMENT_SIZE) {
					USPRINTF("%s %d Segment Idx: %d ~ %d\n", __FUNCTION__, __LINE__, i, j - 1);

					//Copy array
					//USPRINTF("point32Size before: %d\n", point32Size);
					point32Size = j - i;
					//USPRINTF("point32Size after: %d\n", point32Size);
					for (k = 0; k < point32Size; ++k) {
						pp_point32_ary_tmp[k] = pp_positions[i + k];
						pp_u16_ary_tmp[k] = pp_angles[i + k];
						//USPRINTF("PosSize: %d\tId: %d\tPosition: (%d, %d)\n", point32Size, k, pp_point32_ary_tmp[k].X, pp_point32_ary_tmp[k].Y);
					}

					//Fit the line segment
					isFit = pp_find_line( pp_point32_ary_tmp, pp_u16_ary_tmp, point32Size, 1, &lineabcTmp, &cdnTmp, &int16Tmp, &angleVec);

					//Find line
					if (isFit == 1) {
						if (pp_lines_ptr < LINES_SIZE) {
							//Save the significant point
							pp_lines[pp_lines_ptr] = lineabcTmp;
							pp_lines_ptr++;


						} else {
							USPRINTF("First: Too Many lines!\n");
							break;
						}
					}
				}

				i = j - 1;
				break;
			}
		}
	}
	//Set return value
	retval = (lineABC == pp_lines && pp_lines_ptr > 0) ? 1 : 0;

	return retval;
}
#endif

uint8_t pp_rounding(PpRoundingType type, Point32_t target)
{
	uint8_t		Jam = 0, Temp_Counter = 0, RandomRemoteWall_Flag = 0;
	#ifdef MOBILITY
	uint8_t		Mobility_Temp_Error = 0;
	uint32_t	Temp_Mobility_Distance = 0;
	#endif
	int16_t		Left_Wall_Buffer[3] = { 0 }, Right_Wall_Buffer[3] = { 0 };
	int32_t		y_start, R = 0, Proportion = 0, Delta = 0, Previous = 0;
	uint32_t	WorkTime_Buffer = 0, Temp_Status = 0;

#ifdef PP_ROUNDING_ALIGNMENT
	uint8_t		wall_aligned = 0;
	uint16_t	wall_dist = 0;
	Point16_t	wallFollowCell, crtCell = {0, 0};
#endif

	volatile uint8_t	Motor_Check_Code = 0;
	volatile int32_t	L_B_Counter = 0, Wall_Distance = 400, Wall_Straight_Distance, Left_Wall_Speed = 0, Right_Wall_Speed = 0;

	rounding_type = type;
	y_start = Map_GetYCount();
	USPRINTF("%s %d: %d %d %d %d\n", __FUNCTION__, __LINE__, Map_GetYCount(), abs(Map_GetYCount()), target.Y, abs(target.Y));

	pp_rounding_turn((type == PP_ROUNDING_LEFT ? 1 : 0), TURN_SPEED, 900);

	Reset_Rcon_Status();
	#ifdef MOBILITY
	Temp_Mobility_Distance = Get_Move_Distance();
	#endif
	Reset_Wheel_Step();
	Set_Mobility_Step(1000);
	Reset_Wall_Step();
	Reset_WallAccelerate();
	Reset_LeftWallAccelerate();
	Wall_Straight_Distance = 6*DISTANCE_1CM;

#ifdef PP_ROUNDING_ALIGNMENT
	wallFollowCell.X = Map_GetXPos();
	wallFollowCell.Y = Map_GetYPos();
	pp_rounding_alignment_init();
#endif

/*-------------------------- Edit By ZZ -------------------*/
	IsMovingFlag = 0;
	GoToTempTargetFlag = 0;
	PP_RoundingMovingFlag = 0;
	/**********************************************************/
	
	should_mark = 1;
	while (1) {
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

		#ifdef OBS_DYNAMIC
		OBS_Dynamic_Base(100);
		#endif

		pp_rounding_update();

		#ifdef PP_ROUNDING_ALIGNMENT
		if (wall_aligned == 0) {
			crtCell.X = Map_GetXPos();
			crtCell.Y = Map_GetYPos();

			if (pp_positions_ptr == 0 || (pp_positions_ptr < ALIGNMENT_COUNT &&
				TwoPointsDistance(pp_positions[pp_positions_ptr - 1].X, pp_positions[pp_positions_ptr - 1].Y, Map_GetXCount(), Map_GetYCount()) >=
					ALIGNMENT_DATA_STEP * CELL_COUNT_MUL / CELL_SIZE )) {

				pp_positions[pp_positions_ptr].X = Map_GetXCount();
				pp_positions[pp_positions_ptr].Y = Map_GetYCount();
				pp_angles[pp_positions_ptr] = Gyro_GetAngle(0);
				pp_positions_ptr++;
			}

			if (((uint16_t)(TwoPointsDistance( wallFollowCell.X, wallFollowCell.Y, crtCell.X, crtCell.Y ))) > ROBOT_SIZE - 2) {
				wall_dist++;
				wallFollowCell = crtCell;
			}

			if (wall_dist > 7 && pp_rounding_line_segment(pp_lines) == 1 && pp_rounding_calculate_angle_offset(pp_lines) == 1) {
				USPRINTF("%s %d: angle offset %d:\n", __FUNCTION__, __LINE__, pp_angle_offset);
				Gyro_SetOffset(pp_angle_offset);
				wall_aligned = 1;
			}
		}
		#endif

		//USPRINTF("%s %d: %d (%d, %d)\n", __FUNCTION__, __LINE__, target.Y, Map_GetXCount(), Map_GetYCount());
		if ((y_start > target.Y && Map_GetYCount() < target.Y) || (y_start < target.Y && Map_GetYCount() > target.Y)) {
			USPRINTF("%s %d: %d %d %d %d\n", __FUNCTION__, __LINE__, Map_GetYCount(), abs(Map_GetYCount()), target.Y, abs(target.Y));
			STOP_BRIFLY;
			return 0;
		}

		/* Tolerance of distance allow to move back when roundinging the obstcal. */
		if ((target.Y > y_start && (y_start - Map_GetYCount()) > 120) || (target.Y < y_start && (Map_GetYCount() - y_start) > 120)) {
			USPRINTF("%s %d: %d (%d, %d)\n", __FUNCTION__, __LINE__, target.Y, Map_GetXCount(), Map_GetYCount());
			Set_Wheel_Speed(0,0);
			return 0;
		}

		#ifdef MOBILITY
		if (Get_LeftWheel_Step() < 500) {
			Mobility_Temp_Error = 0;
			Temp_Mobility_Distance = Get_Move_Distance();
		} else {
			if ((Get_Move_Distance() - Temp_Mobility_Distance) > 500) {
				Temp_Mobility_Distance = Get_Move_Distance();
				if (Get_Mobility_Step() < 1) {
					Mobility_Temp_Error++;
					if (Mobility_Temp_Error > 5) {
						Mob_Error_Add();
						if (Get_Mob_Error() < 4) {
							Move_Back();
							Turn_Right(Turn_Speed, 1200);
							Reset_Wheel_Step();
							Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
						}
						Mobility_Temp_Error = 0;
					}
				} else {
					Mobility_Temp_Error = 0;
				}
				Reset_Mobility_Step();
			}
		}
		#endif

		/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code = Check_Motor_Current();
		if (Motor_Check_Code) {
			if (Self_Check(Motor_Check_Code)) {
				Set_Clean_Mode(Clean_Mode_Userinterface);
				USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
				break;
			}
			Initialize_Motor();
			USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
			break;
		}

		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if (Touch_Detect()) {
			CM_TouringCancel();
			Set_Clean_Mode(Clean_Mode_Userinterface);
			USPRINTF("%s %d: Check: Touch detect! return 0!\n", __FUNCTION__, __LINE__);
			break;
		}
		if(Get_Rcon_Remote()!=0) {
			#ifdef BLDC_INSTALL
			if(Remote_Key(Remote_Max)) {
				if (CM_IsLowBattery() == 0) {
					Switch_VacMode();
				}
			}
			#endif

			if (Remote_Key(Remote_Home)) {	
				#ifdef BLDC_INSTALL
				Set_BLDC_Speed(Vac_Speed_NormalL);
				#endif
				Deceleration();
				Stop_Brifly();
				CM_SetGoHome(1);
				Map_SetCell(MAP, Map_GetXCount(), Map_GetYCount(), CLEANED);
				USPRINTF("%s %d: Check: Remote_Home! return 0!\n", __FUNCTION__, __LINE__);
				break;
			}
			Reset_Rcon_Remote();
		}

		/*------------------------------------------------------Check Battery-----------------------*/
		if (Check_Bat_SetMotors(Clean_Vac_Power, Clean_SideBrush_Power, Clean_MainBrush_Power)) {		
			Display_Battery_Status(Display_Low);
			Set_Clean_Mode(Clean_Mode_Userinterface);
			USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
			break;
		}
		/*Run wall follow over 10minutes if random mode remote wallfollow*/
		if (RandomRemoteWall_Flag) {
			if ((Get_WorkTime() - WorkTime_Buffer) > 1200) {// wall follow over 10 minutes
				Set_Clean_Mode(Clean_Mode_RandomMode);
				USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
				break;
			}
		}

		Temp_Status = Get_Rcon_Status();
		if ( Temp_Status & (RconFR_HomeT | RconFL_HomeT | RconL_HomeT | RconR_HomeT) ) {
			USPRINTF("%s %d: home detected!\n", __FUNCTION__, __LINE__);
			break;
		}

		#ifdef VIRTUAL_WALL
		/*
		 * When checking virtual wall signals, ignore the RconBL_Wall signal.
		 */
		if (Temp_Status & (RconL_Wall | RconR_Wall | RconBR_Wall | RconFR_Wall | RconFL_Wall|RconL_Wall_T | RconR_Wall_T  | RconFR_Wall_T | RconFL_Wall_T)) {
			Stop_Brifly();
			USPRINTF("%s %d: virtual wall detected! %d\n", __FUNCTION__, __LINE__, Temp_Status);

			if (Temp_Status & (RconFR_Wall | RconFL_Wall|RconFR_Wall_T | RconFL_Wall_T)) {
				Turn_Right(Turn_Speed, 450);
			} else if (Temp_Status & (RconR_Wall|RconR_Wall_T)) {
				Turn_Right(Turn_Speed, 300);
			} else if (Temp_Status & (RconBR_Wall)) {
				Turn_Right(Turn_Speed, 300);
			} else if (Temp_Status & (RconL_Wall|RconL_Wall_T)) {
				Turn_Right(Turn_Speed, 600);
			}
			Reset_VirtualWall();
			USPRINTF("%s %d: Check: virtual wall! Break!\n", __FUNCTION__, __LINE__);
			break;
		}
		#endif

    /*------------------------------------------------------Cliff Event-----------------------*/
		if (Get_Cliff_Trig()) {
				WFM_update();
				WFM_move_back(9*DISTANCE_1CM);
				WFM_update();
				if (Get_Cliff_Trig() == (Status_Cliff_All)) {
					Set_Clean_Mode(Clean_Mode_Userinterface);
					CM_TouringCancel();
					USPRINTF("%s %d: Check: Cliff 1! break\n", __FUNCTION__, __LINE__);
					break;
				}
				if (Get_Cliff_Trig()) {
					if (Cliff_Escape()) {
						Set_Error_Code(Error_Code_Cliff);
						Set_Clean_Mode(Clean_Mode_Userinterface);
						CM_TouringCancel();
						USPRINTF("%s %d: Check: Cliff 2! break\n", __FUNCTION__, __LINE__);
						break;
					}
				}
				if (type == PP_ROUNDING_LEFT && Get_LeftWheel_Step() < 12500) {
					pp_rounding_turn(1, TURN_SPEED, 750);

					Stop_Brifly();
					Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
					Reset_WallAccelerate();
					Wall_Straight_Distance = 6*DISTANCE_1CM;
				} else if (type == PP_ROUNDING_RIGHT && Get_RightWheel_Step() < 12500) {
					pp_rounding_turn(0, TURN_SPEED, 750);

					Stop_Brifly();
					Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
					Wall_Straight_Distance = 6*DISTANCE_1CM;
				} else {
					USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
					break;
				}
				for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++) {
					Left_Wall_Buffer[Temp_Counter] = 0;
				}
				Reset_Wheel_Step();
		}

		if (pp_rounding_boundary_check() == 1) {
			USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
			break;
		}

		if (type == PP_ROUNDING_LEFT) {
			if (Get_Bumper_Status() & RightBumperTrig) {
				STOP_BRIFLY;
				CheckGyroCalibrationTime(120);
				if(Get_WallAccelerate()>2000)
				{
					Jam=0;
				}
				Stop_Brifly();
				/*pp_rounding_move_back(5*DISTANCE_1CM);*/
				pp_rounding_Big_move_back(0,400);	
				if (Get_WallAccelerate() > 100) {
					if (Is_Bumper_Jamed()) {
						USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						break;
					}
				}
				/*pp_rounding_turn(1, TURN_SPEED, 900);*/
				Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
				Reset_WallAccelerate();
				Wall_Straight_Distance = 6*DISTANCE_1CM;
				Reset_Wheel_Step();
				Wall_Distance=Wall_High_Limit;	
	    }
			if (Get_Bumper_Status() & LeftBumperTrig) {
				if(Get_WallAccelerate()>2000)
				{
					Jam=0;
				}
				L_B_Counter++;
				Set_Wheel_Speed(0, 0);
				Reset_TempPWM();
				delay(300);
				if (Get_LWall_ADC() > (Wall_Low_Limit)) {
					Wall_Distance = Get_LWall_ADC() / 1;
				} else {
					Wall_Distance += 200;
				}

				if ((Get_Bumper_Status() & RightBumperTrig)||Is_FrontOBS_Trig()) {
					/*Move_Back();*/
					pp_rounding_Big_move_back(0,400);	
					if (Is_Bumper_Jamed()) {
						USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						break;;
					}
					/*pp_rounding_turn(1, TURN_SPEED , 600);*/
					Wall_Straight_Distance=5*DISTANCE_1CM;
				  Wall_Distance=Wall_High_Limit;	
				} else {
					Wall_Move_Back();
					Stop_Brifly();
					/*Wall_Distance=Wall_Low_Limit;	
					if(Jam<2)
					{
						pp_rounding_Small_move_back(0,500);				
					}
					else
					{ 
						pp_rounding_Big_move_back(0,400);
					}*/					
					if (Is_Bumper_Jamed()) {
						USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						break;
					}
					if (Jam < 3) {
						if (Wall_Distance < 200) {
							if (Get_LeftOBS() > (Get_LeftOBST_Value() - 200)){
								Wall_Distance = Wall_High_Limit;
								pp_rounding_turn(1, TURN_SPEED , 300);
							} else{
								pp_rounding_turn(1, TURN_SPEED , 150);
							}
						} else {
							pp_rounding_turn(1, TURN_SPEED , 300);
						}
					} else {
						pp_rounding_turn(1, TURN_SPEED , 150);
					}
					Wall_Straight_Distance = 5*DISTANCE_1CM;
				}
				
				if(Get_WallAccelerate()<1000)
				{
					Jam++;
				}
				else
				{
					Jam=0;
				}				
				Reset_WallAccelerate();

				Move_Forward(RUN_SPEED_5, RUN_SPEED_5);

				for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++) {
					Left_Wall_Buffer[Temp_Counter] = 0;
				}
				Reset_Wheel_Step();
				if (Wall_Distance < Wall_Low_Limit) {
					Wall_Distance = Wall_Low_Limit;
				}
				if (Wall_Distance > Wall_High_Limit) {
					Wall_Distance = Wall_High_Limit;
				}								
			}
			
			if(Get_WallAccelerate()>1000)
			{
				Jam=0;
			}
			
			if (Wall_Distance >= Wall_Low_Limit) {
				Left_Wall_Buffer[2] = Left_Wall_Buffer[1];
				Left_Wall_Buffer[1] = Left_Wall_Buffer[0];
				Left_Wall_Buffer[0] = Get_LWall_ADC();
				if (Left_Wall_Buffer[0] < 200) {
					if ((Left_Wall_Buffer[1] - Left_Wall_Buffer[0]) > (Wall_Distance / 25)) {
						if ((Left_Wall_Buffer[2] - Left_Wall_Buffer[1]) > (Wall_Distance / 25)) {
							if (Get_WallAccelerate()>300) {
								if ((Get_RightWheel_Speed()-Get_LeftWheel_Speed()) >= -3) {
									Move_Forward(RUN_SPEED_9, RUN_SPEED_9);
									delay(1000);
									Reset_WallAccelerate();
									Wall_Straight_Distance = 6*DISTANCE_1CM;
								}
							}
						}
					}
				}
			}

			/*------------------------------------------------------Short Distance Move-----------------------*/
			if (Get_WallAccelerate() < (uint32_t) Wall_Straight_Distance) {
				if (Get_LeftWheel_Step() < 500) {
					if (Get_WallAccelerate() < 100) {
						Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
					} else {
						Move_Forward(RUN_SPEED_9, RUN_SPEED_9);
					}
				} else {
					Move_Forward(RUN_SPEED_10, RUN_SPEED_10);
				}
			} else {
				/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
				if (!Is_FrontOBS_Trig()) {
					Proportion = Get_LWall_ADC();

					Proportion = Proportion * 100 / Wall_Distance;

					Proportion -= 100;

					Delta = Proportion - Previous;					

					Delta /= 3;

					if (Wall_Distance > 300) {	//over left
						Left_Wall_Speed  = RUN_SPEED_12 + Proportion / 12 + Delta / 5;
						Right_Wall_Speed = RUN_SPEED_12 - Proportion / 10 - Delta / 5;
						if (Right_Wall_Speed > RUN_SPEED_14) {
							Left_Wall_Speed = RUN_SPEED_4; 
							Right_Wall_Speed = RUN_SPEED_14;
						}
					} else if (0 && Wall_Distance > 200){	//over left
						Left_Wall_Speed  = RUN_SPEED_10 + Proportion / 15 + Delta / 7;
						Right_Wall_Speed = RUN_SPEED_10 - Proportion / 12 - Delta / 7;

						if (Right_Wall_Speed > RUN_SPEED_12) {
							Left_Wall_Speed  = RUN_SPEED_3; //8;
							Right_Wall_Speed = RUN_SPEED_13;
						}
					} else {
						Left_Wall_Speed  = RUN_SPEED_8 + Proportion / 18 + Delta / 10;
						Right_Wall_Speed = RUN_SPEED_8 - Proportion / 15 - Delta / 10;
						if(Left_Wall_Speed > RUN_SPEED_10)Left_Wall_Speed = RUN_SPEED_10;
						if (Right_Wall_Speed > RUN_SPEED_9) {
							Left_Wall_Speed  = RUN_SPEED_3;
							Right_Wall_Speed = RUN_SPEED_10;
						}

						/*if (Left_Wall_Speed > 20)Left_Wall_Speed = 20; 													
						if (Left_Wall_Speed < 4)Left_Wall_Speed = 4; 													
						if (Right_Wall_Speed < 4)Right_Wall_Speed = 4; 													
						if ((Left_Wall_Speed - Right_Wall_Speed) > 5) {
							Left_Wall_Speed = Right_Wall_Speed+5;
						}*/
						
					}

					/*slow move if left obs near a wall*/
					if(Get_LWall_ADC()>Wall_Low_Limit){//200
						Wall_Distance+=5;
					}
					if(Get_LWall_ADC()>Wall_High_Limit/2){
						Wall_Distance+=10;
					}
					if(Get_LWall_ADC()>Wall_High_Limit){
						Wall_Distance+=20;
					}
					if(Get_LWall_ADC()<Wall_High_Limit/2)
					{
						if(Wall_Distance>Wall_High_Limit/2)Wall_Distance=Wall_High_Limit/2;
					}
					else if(Get_LWall_ADC()<Wall_High_Limit)
					{
						if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;
					}
					else
					{
						if(Wall_Distance>Wall_High_Limit*2)Wall_Distance=Wall_High_Limit*2;
					}
					if (Is_WallOBS_Near()){
						Left_Wall_Speed = Left_Wall_Speed / 2;
						Right_Wall_Speed = Right_Wall_Speed / 2;
					}

					Previous = Proportion;

					if (Left_Wall_Speed < 0) {
						Left_Wall_Speed = 0;
					}
					if (Left_Wall_Speed > RUN_SPEED_15) {
						Left_Wall_Speed = RUN_SPEED_15;
					}
					if (Right_Wall_Speed < 0) {
						Right_Wall_Speed = 0;
					}

					Move_Forward(Left_Wall_Speed, Right_Wall_Speed);

					if (Get_RightWall_Step() > Get_LeftWall_Step()) {
						R = Get_RightWall_Step() - Get_LeftWall_Step();
					}
					if (R > TURN_AROUND_CNT) {
						if (RandomRemoteWall_Flag) {
							USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
							Set_Clean_Mode(Clean_Mode_RandomMode);
							break;
						} else {
							USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
							Set_Clean_Mode(Clean_Mode_WallFollow);
						}
						break;
					}
					if (Get_WallAccelerate() > 750) {
						Set_Left_Brush(ENABLE);
						Set_Right_Brush(ENABLE);
					}
				} else {
					Stop_Brifly();
					pp_rounding_Big_move_back(0,900);
					if (Get_LeftWheel_Step() < 12500) {
						if (Is_FrontOBS_Trig()) {
							if (Get_WallAccelerate() < 2000) {
								Jam++;
							}
//							pp_rounding_turn(1, TURN_SPEED , 880);
						} else {
//							pp_rounding_turn(1, TURN_SPEED , 400);
						}
					} else {
//						pp_rounding_turn(1, TURN_SPEED , 900);						
					}
					Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
					Reset_Wheel_Step();
					Wall_Distance = Wall_High_Limit;
				}
			}
		} 
		else 
		{/* PP_ROUNDING_RIGHT */
			if (Get_Bumper_Status() & LeftBumperTrig) {
				STOP_BRIFLY;				
				CheckGyroCalibrationTime(120);
				if(Get_WallAccelerate()>2000)
				{
					Jam=0;
				}
				Stop_Brifly();
				/*pp_rounding_move_back(5*DISTANCE_1CM);*/
				pp_rounding_Big_move_back(1,400);
				if (Get_LeftWallAccelerate() > 100) {
					if (Is_Bumper_Jamed()) {
						USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						break;
					}
				}
				/*pp_rounding_turn(0, TURN_SPEED, 900);*/
				Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
				Reset_LeftWallAccelerate();
				Wall_Straight_Distance = 6*DISTANCE_1CM;
				Reset_Wheel_Step();
				Wall_Distance=Wall_High_Limit;
	      }

			if (Get_Bumper_Status() & RightBumperTrig) {
				if(Get_WallAccelerate()>2000)
				{
					Jam=0;
				}
				L_B_Counter++;
				Set_Wheel_Speed(0, 0);
				Reset_TempPWM();
				delay(300);
				if (Get_RWall_ADC() > (Wall_Low_Limit)) {
					Wall_Distance = Get_RWall_ADC() / 1;
				} else {
					Wall_Distance += 200;
				}

				if ((Get_Bumper_Status() & LeftBumperTrig)||Is_FrontOBS_Trig()) {
					/*Move_Back();*/
					pp_rounding_Big_move_back(1,400);	
					if (Is_Bumper_Jamed()) {
						USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						break;;
					}
					/*pp_rounding_turn(0, TURN_SPEED, 600);*/
					Wall_Straight_Distance = 5*DISTANCE_1CM;
					Wall_Distance=Wall_High_Limit;
				} else {
					Wall_Move_Back();
					Stop_Brifly();
					/*if(Jam<2)
					{
						pp_rounding_Small_move_back(1,500);				
					}
					else
					{ 
						pp_rounding_Big_move_back(1,400);
					}*/					
					if (Is_Bumper_Jamed()) {
						USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
						break;
					}
					if (Jam < 3) {
						if (Wall_Distance < 200) {
							if (Get_RightOBS() > (Get_RightOBST_Value())){
								Wall_Distance = Wall_High_Limit;
								pp_rounding_turn(0, TURN_SPEED , 300);
							} else{
								pp_rounding_turn(0, TURN_SPEED , 150);
							}
						} else {
							pp_rounding_turn(0, TURN_SPEED , 300);
						}
					} else {
						pp_rounding_turn(0, TURN_SPEED , 150);
					}
					Wall_Straight_Distance =5*DISTANCE_1CM;
				}
				if(Get_WallAccelerate()<1000)
				{
					Jam++;
				}
				else
				{
					Jam=0;
				}					
				Reset_LeftWallAccelerate();

				Move_Forward(RUN_SPEED_5, RUN_SPEED_5);

				for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++) {
					Right_Wall_Buffer[Temp_Counter] = 0;
				}
				Reset_Wheel_Step();
				if (Wall_Distance < Wall_Low_Limit) {
					Wall_Distance = Wall_Low_Limit;
				}
				if (Wall_Distance > Wall_High_Limit) {
					Wall_Distance = Wall_High_Limit;
				}				
			}
			
			if(Get_WallAccelerate()>1000)
			{
				Jam=0;
			}
			
			if (Wall_Distance >= Wall_Low_Limit) {
				Right_Wall_Buffer[2] = Right_Wall_Buffer[1];
				Right_Wall_Buffer[1] = Right_Wall_Buffer[0];
				Right_Wall_Buffer[0] = Get_RWall_ADC();
				if (Right_Wall_Buffer[0] < 200) {
					if ((Right_Wall_Buffer[1] - Right_Wall_Buffer[0]) > (Wall_Distance / 25)) {
						if ((Right_Wall_Buffer[2] - Right_Wall_Buffer[1]) > (Wall_Distance / 25)) {
							if (Get_LeftWallAccelerate()>300) {
								if ((Get_LeftWheel_Speed() - Get_RightWheel_Speed()) >= -3) {
									Move_Forward(RUN_SPEED_9, RUN_SPEED_9);
									delay(1000);
									Reset_LeftWallAccelerate();
									Wall_Straight_Distance = 6*DISTANCE_1CM;
								}
							}
						}
					}
				}
			}

			/*------------------------------------------------------Short Distance Move-----------------------*/
			if (Get_LeftWallAccelerate() < (uint32_t) Wall_Straight_Distance) {
				if (Get_RightWheel_Step() < 500) {
					if (Get_LeftWallAccelerate() < 100) {
						Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
					} else {
						Move_Forward(RUN_SPEED_9, RUN_SPEED_9);
					}
				} else {
					Move_Forward(RUN_SPEED_10, RUN_SPEED_10);
				}
			} else {
				/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
				if (!Is_FrontOBS_Trig()) {
					Proportion = Get_RWall_ADC();

					Proportion = Proportion * 100 / Wall_Distance;

					Proportion -= 100;

					Delta = Proportion - Previous;
					
					Delta /= 3;

					if (Wall_Distance > 300) {	//over left
						Right_Wall_Speed = RUN_SPEED_12 + Proportion / 12 + Delta / 5;
						Left_Wall_Speed  = RUN_SPEED_12 - Proportion / 10 - Delta / 5;
						if (Left_Wall_Speed > RUN_SPEED_14) {
							Right_Wall_Speed = RUN_SPEED_4; // 9;
							Left_Wall_Speed  = RUN_SPEED_14;
						}
					} else if (Wall_Distance > 150){	//over left
						Right_Wall_Speed = RUN_SPEED_10 + Proportion / 15 + Delta / 7;
						Left_Wall_Speed  = RUN_SPEED_10 - Proportion / 12 - Delta / 7;

						if (Left_Wall_Speed > RUN_SPEED_12) {
							Right_Wall_Speed = RUN_SPEED_3;
							Left_Wall_Speed  = RUN_SPEED_13;
						}
					} else {
						Right_Wall_Speed = RUN_SPEED_8 + Proportion / 18 + Delta / 10;
						Left_Wall_Speed  = RUN_SPEED_8 - Proportion / 15 - Delta / 10;
						if(Right_Wall_Speed > RUN_SPEED_10)Right_Wall_Speed = RUN_SPEED_10;
						if (Left_Wall_Speed > RUN_SPEED_9) {
							Right_Wall_Speed = RUN_SPEED_3;
							Left_Wall_Speed  = RUN_SPEED_10;
						}

//						if (Right_Wall_Speed > 20)Right_Wall_Speed = 20; 													
//						if (Right_Wall_Speed < 4)Right_Wall_Speed = 4; 													
//						if (Left_Wall_Speed < 4)Left_Wall_Speed = 4; 													
//						if ((Right_Wall_Speed - Left_Wall_Speed) > 5) {
//							Right_Wall_Speed = Left_Wall_Speed + 5;
//						}
					}

					/*slow move if left obs near a wall*/
					if(Get_RWall_ADC()>Wall_Low_Limit){//200
						Wall_Distance+=5;
					}
					if(Get_RWall_ADC()>Wall_High_Limit/2){
						Wall_Distance+=10;
					}
					if(Get_RWall_ADC()>Wall_High_Limit){
						Wall_Distance+=20;
					}
					if(Get_RWall_ADC()<Wall_High_Limit/2)
					{
						if(Wall_Distance>Wall_High_Limit/2)Wall_Distance=Wall_High_Limit/2;
					}
					else if(Get_RWall_ADC()<Wall_High_Limit)
					{
						if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;
					}
					else
					{
						if(Wall_Distance>Wall_High_Limit*3/2)Wall_Distance=Wall_High_Limit*3/2;
					}
					if (Is_WallOBS_Near()){
						Right_Wall_Speed = Right_Wall_Speed / 2;
						Left_Wall_Speed = Left_Wall_Speed/2;
					}

					Previous = Proportion;

					if (Right_Wall_Speed < 0) {
						Right_Wall_Speed = 0;
					}
					if (Right_Wall_Speed > RUN_SPEED_15) {
						Right_Wall_Speed = RUN_SPEED_15;
					}
					if (Left_Wall_Speed < 0) {
						Left_Wall_Speed = 0;
					}

					Move_Forward(Left_Wall_Speed, Right_Wall_Speed);

					if (Get_LeftWall_Step() > Get_RightWall_Step()) {
						R = Get_LeftWall_Step() - Get_RightWall_Step();
					}
					if (R > TURN_AROUND_CNT) {
						if (RandomRemoteWall_Flag) {
							Set_Clean_Mode(Clean_Mode_RandomMode);
							USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
							break;
						} else {
							USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
							Set_Clean_Mode(Clean_Mode_WallFollow);
						}
						break;
					}
					if (Get_LeftWallAccelerate() > 750) {
						Set_Left_Brush(ENABLE);
						Set_Right_Brush(ENABLE);
					}
				} else {
					Stop_Brifly();
					pp_rounding_Big_move_back(1,900);
					if (Get_RightWheel_Step() < 12500) {
						if (Is_FrontOBS_Trig()) {
							if (Get_LeftWallAccelerate() < 2000) {
								Jam++;
							}
							else
							{
								Jam = 0;
							}
//							pp_rounding_turn(0, TURN_SPEED , 880);
						} else {
//							pp_rounding_turn(0, TURN_SPEED , 400);
						}
					} else {
//						pp_rounding_turn(0, TURN_SPEED , 900);					
					}
					Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
					Reset_Wheel_Step();
					Wall_Distance = Wall_High_Limit;
				}
			}
		}

		if (Jam > 80) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Set_Error_Code(Error_Code_Stuck);
			USPRINTF("%s %d\n", __FUNCTION__, __LINE__);
			break;
		}
		if (L_B_Counter > 10) {

		}
	}
	return 0;
}
