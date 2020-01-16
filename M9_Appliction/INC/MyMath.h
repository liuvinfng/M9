#ifndef __MYMATH_H
#define __MYMATH_H


#include "SysInitialize.h"



#define PI  3.141592653589793

#ifndef M_PI

#define M_PI	3.141592653589793

#endif

typedef struct
{
	int8_t x;
	int8_t y;
} Point8_t;

typedef struct
{
	int16_t X;
	int16_t Y;
} Point16_t;

typedef struct
{
	int32_t X;
	int32_t Y;
} Point32_t;

typedef struct
{
  double A;
  double B;
  double C;
}LineABCDouble_t;

typedef struct
{
	int32_t A;
	int32_t B;
	int32_t C;
}LineABCInt_t;

typedef enum 
{
  SHORT_PATH = 1,
  NORMAL_CLEAN = 2,
	SPOT_CLEAN = 3,
} PathListState_t;

typedef enum
{
	WALL_NORMAL = 1,
	WALL_TRAPPED = 2,
	WALL_SPOT = 3,
	WALL_RANDOM = 4,
	WALL_AROUND = 5,
}WallTravel_t;



typedef struct
{
	Point16_t cell_pos;
	PathListState_t status;
}PathList_t;

uint16_t Math_NormalAngle(int16_t angle);
double Math_Absolute(double d);
int32_t Math_GetMax(int32_t a,int32_t b);
int32_t Math_GetMin(int32_t a,int32_t b);
double Math_Deg2Rad(double deg, int8_t scale);
double Math_Rad2Deg(double rad, int8_t scale);
uint16_t Math_Course2Dest(int32_t startx, int32_t starty, int32_t destx, int32_t desty);
uint32_t Math_TwoPointsDistance(int32_t startx, int32_t starty, int32_t destx, int32_t desty);
int32_t TwoPointsDistanceAtDirection(int32_t startx, int32_t starty, int32_t destx, int32_t desty, int16_t theta);
int16_t distance2line(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t px, int32_t py);
uint16_t angle_delta(uint16_t a, uint16_t b);
int32_t limit(int32_t i, int32_t lower_limit, int32_t upper_limit);
void Matrix_Translate(double * x, double * y, double offset_x, double offset_y);
void Matrix_Rotate(double * x, double * y, double theta);
Point16_t calIntersPoint( Point16_t l1StartPnt, Point16_t l1EndPnt,
                          Point16_t l2StartPnt, Point16_t l2EndPnt );

double radDeltaAngleVector( double a, double b );
int16_t degreeDeltaAngleVector( uint16_t a, uint16_t b ) ;
double radDeltaAngleMin( double a, double b );
int16_t degreeDeltaAngleMin( uint16_t a, uint16_t b );

double arctan( double deltay, double deltax );
double TwoLinesAngle( LineABCDouble_t la, LineABCDouble_t lb );
double LineAngle( LineABCDouble_t l, uint8_t mode );
uint8_t IsSamePointAndAngle( Point32_t pnt1, uint16_t angle1, Point32_t pnt2, uint16_t angle2,
                             uint32_t pntThres, uint16_t angleThres );
														 
int32_t Math_Abs_int(int32_t d);

LineABCInt_t Line_CalculateLine(Point32_t A,Point32_t B);
int32_t Line_CalculateX(LineABCInt_t L,int32_t Y);
int32_t Line_CalculateY(LineABCInt_t L,int32_t X);
int32_t Math_Diff_int(int32_t A, int32_t B);

int32_t Math_RoundAngle(int32_t angle);
int32_t Math_LimitingValue(int32_t input,int32_t limit);
uint32_t Math_TwoPoint_Dis(Point32_t start_point,Point32_t exit_point);

int16_t Math_TwoPoint_Angle(Point32_t start_point,Point32_t dest_point);

uint8_t Math_IsTwoCell_Orthogonal(Point16_t src,Point16_t dest);

uint32_t Math_TwoCell_Dis(Point16_t start_cell,Point16_t exit_cell);


#endif


