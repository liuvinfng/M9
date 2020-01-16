#ifndef __MYMATH_H
#define __MYMATH_H

#include "math.h"
#include "stdlib.h"
#include "SysInitialize.h"

#define PI  3.141592653589793

typedef struct{
	int16_t X;
	int16_t Y;
} Point16_t;

typedef struct{
  double A;
  double B;
  double C;
} LineABC;

typedef struct{
	int32_t X;
	int32_t Y;
} Point32_t;

double absolute(double d);
double deg2rad(double deg, int8_t scale);
double rad2deg(double rad, int8_t scale);
uint16_t course2dest(int32_t startx, int32_t starty, int32_t destx, int32_t desty);
uint32_t TwoPointsDistance(int32_t startx, int32_t starty, int32_t destx, int32_t desty);
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
double TwoLinesAngle( LineABC la, LineABC lb );
double LineAngle( LineABC l, uint8_t mode );
uint8_t IsSamePointAndAngle( Point32_t pnt1, uint16_t angle1, Point32_t pnt2, uint16_t angle2,
                             uint32_t pntThres, uint16_t angleThres );
#endif
