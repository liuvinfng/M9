#include "include.h"
#include "math.h"
#include "stdlib.h"
#include "main.h"
#include "charge.h"
#include "projecttask.h"
#include "userinterface.h"
#include "remote_mode.h"
#include "spot.h"
#include "standby.h"
#include "wallfollow.h"
#include "homestraight.h"
#include "cormove.h"
#include "pathplanning.h"
#include "shortestpath.h"
#include "map.h"
#include "wifi.h"
#include "movement.h"
#include "display.h"
#include "usart.h"
#include "spi.h"
#include "speaker.h"
#include "rcon.h"
#include "rtc.h"
#include "touchpad.h"
#include "gyro.h"
#include "wheel.h"
#include "obscliff.h"
#include "bldc.h"
#include "brush.h"
#include "w25q16.h"
#include "mymath.h"
#include "debug.h"
#include "cmsis_os.h"
#include "config.h"


uint16_t Math_NormalAngle(int16_t angle)
{
	while(angle >= 3600)angle -= 3600;
	while(angle < 0)angle += 3600;

	return angle; 
}

double Math_Absolute(double d)
{
	return ((d < 0) ? (d * (-1)) : d);
}

int32_t Math_GetMax(int32_t a,int32_t b)
{
	return ((a > b) ? a : b);
}

int32_t Math_GetMin(int32_t a,int32_t b)
{
	return ((a < b) ? a : b);
}

int32_t Math_Diff_int(int32_t A, int32_t B)
{
	return ((A > B) ? (A - B) : (B - A));
}

int32_t Math_Abs_int(int32_t d)
{
	return ((d < 0) ? (-d) : d);
}

double Math_Deg2Rad(double deg, int8_t scale)
{
	return (deg * PI / (180 * scale));
}

double Math_Rad2Deg(double rad, int8_t scale)
{
	return (scale * rad * 180 / PI);
}


/*calculate a line using two different points
@ Point32_t A first point on the line
@	Point32_t B second point on the line
@	return LinaABC line
*/

LineABCInt_t Line_CalculateLine(Point32_t A,Point32_t B)
{
	LineABCInt_t Temp;
	Temp.A = B.Y - A.Y;//y2-y1
	Temp.B = A.X - B.X;//x1-x2
	Temp.C = B.X * A.Y - A.X*B.Y;//x2*y1-x1*y2
	return Temp;
}

int32_t Line_CalculateX(LineABCInt_t L,int32_t Y)
{
	if(L.A==0)
	{
		return 0;
	}
	else
	{
		return -(L.B*Y+L.C)/L.A;
	}
}

int32_t Line_CalculateY(LineABCInt_t L,int32_t X)
{
	if(L.B==0)
	{
		return 0;
	}
	else
	{
		return -(L.A*X+L.C)/L.B;
	}
}




uint16_t Math_Course2Dest(int32_t startx, int32_t starty, int32_t destx, int32_t desty)
{
	int16_t alpha = 0;

	if (startx == destx) {
		if (desty > starty) {
			alpha = 900;
		} else if (desty < starty) {
			alpha = 2700;
		} else {
			alpha = 0;
		}
	} else {
		alpha = round(Math_Rad2Deg(atan(((double)(desty - starty) / (destx - startx))), 10));

		if (destx < startx) {
			alpha += 1800;
		}

		if (alpha < 0) {
			alpha += 3600;
		}
	}

	return (uint16_t)alpha;
}

uint32_t Math_TwoPointsDistance(int32_t startx, int32_t starty, int32_t destx, int32_t desty)
{
	double d, e;

	d = destx - (double)startx;
	e = desty - (double)starty;
	d *= d;
	e *= e;

	return (uint32_t)round(sqrt(d + e));
}
uint32_t Math_TwoPoint_Dis(Point32_t start_point,Point32_t exit_point)
{
	int32_t d, e;
	
	d = start_point.X - exit_point.X;
	e = start_point.Y - exit_point.Y;
	d *= d;
	e *= e;

	return (uint32_t)round(sqrt(d + e));
}
int32_t TwoPointsDistanceAtDirection(int32_t startx, int32_t starty, int32_t destx, int32_t desty, int16_t theta)
{
	return (int32_t)round(cos(Math_Deg2Rad(theta, 10)) * (destx - startx) + sin(Math_Deg2Rad(theta, 10)) * (desty - starty));
}

int16_t distance2line(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t px, int32_t py) {
	double dx = x2 - x1;
	float dy = y2 - y1;
	if ((dx == 0) && (dy == 0)) {//line too short ..... start point == end point
		dx = px - x1;
		dy = py - y1;
		return (int16_t)(sqrt(dx * dx + dy * dy));//return distance of position (px,py) and (x1,y1)
	}

	float t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);

	if (t < 0) {
		/* point is nearest to the first point i.e x1 and y1 */
		dx = px - x1;
		dy = py - y1;
	} else if (t > 1) {
		/* point is nearest to the end point i.e x2 and y2. */
		dx = px - x2;
		dy = py - y2;
	} else {
		/* if perpendicular line intersect the line segment. */
		dx = px - (x1 + t * dx);
		dy = py - (y1 + t * dy);
	}
	/* returning shortest distance. */
	return (int16_t) (sqrt(dx * dx + dy * dy));
}

uint16_t angle_delta(uint16_t a, uint16_t b) {
	if ((a - b) > 1800) {
		b += 3600;
	} else if ((b - a) > 1800) {
		a += 3600;
	}
	return ((a > b) ? (a - b) : (b - a));
}

int32_t limit(int32_t i, int32_t lower_limit, int32_t upper_limit)
{
	if (i > upper_limit) {
		return upper_limit;
	} else if (i < lower_limit) {
		return lower_limit;
	} else {
		return i;
	}
}

void Matrix_Translate(double * x, double * y, double offset_x, double offset_y)
{
	*x += offset_x;
	*y += offset_y;
}

void Matrix_Rotate(double * x, double * y, double theta)
{
	double d, e;

	theta = Math_Deg2Rad(theta, 10);

	d = (*x) * cos(theta) - (*y) * sin(theta);
	e = (*x) * sin(theta) + (*y) * cos(theta);

	*x = d;
	*y = e;
}

Point16_t calIntersPoint( Point16_t l1StartPnt, Point16_t l1EndPnt,
                          Point16_t l2StartPnt, Point16_t l2EndPnt )
{
	Point16_t retval;
	double l1[4], l2[4], p[2] = { 32555, 32555 };
	l1[0] = (double)(l1StartPnt.X); l1[1] = (double)(l1StartPnt.Y);
	l1[2] = (double)(l1EndPnt.X);	 l1[3] = (double)(l1EndPnt.Y);

	l2[0] = (double)(l2StartPnt.X); l2[1] = (double)(l2StartPnt.Y);
	l2[2] = (double)(l2EndPnt.X);	 l2[3] = (double)(l2EndPnt.Y);
	//Two line is vertical, return a special point 32766, 32766, means that no intersection point
	if ( Math_Absolute(l1[0] - l1[2]) < 0.1 && Math_Absolute(l2[0] - l2[2]) < 0.1 ) {
		p[0] = 32666;
		p[1] = 32666;
	}
	//Two lines are horizontal
	else if ( Math_Absolute((l1[1] - l1[3]) / (l1[0] - l1[2]) -
	                   (l2[1] - l2[3]) / (l2[0] - l2[2])) < 0.1 ) {
		p[0] = 32767;
		p[1] = 32767;
	}
	//Line l1 is horizontal and line l2 is not vertical
	else if ( Math_Absolute(l1[1] - l1[3]) < 0.1 && Math_Absolute(l2[2] - l2[0]) > 0.1 ) {
		p[0] = (l1[1] - l2[1]) * (l2[2] - l2[0]) / (l2[3] - l2[1]) + l2[0];
		p[1] = l1[1];
	}
	//Line l1 is horizontal and line l2 is vertical
	else if ( Math_Absolute(l1[1] - l1[3]) < 0.1 && Math_Absolute(l2[2] - l2[0]) < 0.1 ) {
		p[0] = l2[0];
		p[1] = l1[1];
	}
	//Line l1 is not horizontal and line l2 is not vertical, math calculation
	else if ( Math_Absolute(l1[1] - l1[3]) > 0.1 && Math_Absolute(l2[2] - l2[0]) > 0.1 ) {
		double delta = ( l2[2] - l2[0] ) * ( l1[3] - l1[1] ) -
		               ( l2[3] - l2[1] ) * ( l1[2] - l1[0] ),
		          px = ( l2[1] - l1[1] ) * ( l1[0] - l1[2] ) * ( l2[0] - l2[2] ) -
		               l2[0] * ( l2[1] - l2[3] ) * ( l1[0] - l1[2] ) +
		               l1[0] * ( l1[1] - l1[3] ) * ( l2[0] - l2[2] );
		p[0] = px / delta;
		p[1] = ( l2[1] - l2[3] ) * ( p[0] - l2[0] ) / ( l2[0] - l2[2] ) + l2[1];
	}
	//Line l1 is not horizontal and line l2 is vertical
	else if ( Math_Absolute(l1[1] - l1[3]) > 0.1 && Math_Absolute(l2[2] - l2[0]) > 0.1 ) {
		p[0] = l2[0];
		p[1] = ( l1[1] - l1[3] ) * ( p[0] - l1[0] ) / ( l1[0] - l1[2] ) + l1[1];
	}

	retval.X = (int16_t)p[0];
	retval.Y = (int16_t)p[1];
	return retval;
}

//Calculate radian a - b and change into range (-pi, pi]
double radDeltaAngleVector( double a, double b ) {
	double tmp = a - b;
	while ( tmp > PI ) {
		tmp -= 2 * PI;
	}
	while ( tmp <= - PI ) {
		tmp += 2 * PI;
	}
	return tmp;
}

//Calculate degree a - b and change into range(-1800, 1800]
int16_t degreeDeltaAngleVector( uint16_t a, uint16_t b ) {
	int16_t tmp = a - b;
	while ( tmp > 1800 ) {
		tmp -= 3600;
	}
	while ( tmp <= - 1800 ) {
		tmp += 3600;
	}
	return tmp;
}

//Calculate minimum angle between a and b, and change into range (-pi/2, pi/2]
double radDeltaAngleMin( double a, double b ) {
	double tmp = radDeltaAngleVector(a, b);

	if ( tmp > PI / 2 ) {
		tmp -= PI;
	} else if ( tmp <= - PI / 2 ) {
		tmp += PI;
	}
	return tmp;
}

//Calculate minimum angle between a and b, and change into range (-900, 900]
int16_t degreeDeltaAngleMin( uint16_t a, uint16_t b ) {
	int16_t tmp = degreeDeltaAngleVector(a, b);
	if ( tmp > 900 ) {
		tmp -= 1800;
	} else if ( tmp <= - 900 ) {
		tmp += 1800;
	}
	return tmp;
}

//arctan, range is [0, 2 * PI)
double arctan( double deltay, double deltax ) {
	if ( deltax == 0 ) {
		if ( deltay >= 0 )
			return PI / 2;
		else return -PI / 2;
	} else {
		double angle = atan( deltay / deltax );
		if ( deltax < 0 && angle > 0 )
			angle -= PI;
		else if ( deltax < 0 && angle <= 0 )
			angle += PI;
		return angle;
	}
}

//Angle of two lines, range is [0, pi /2]
double TwoLinesAngle( LineABCDouble_t la, LineABCDouble_t lb ) {
	double tmp;
	if ( la.A != 0.0 || la.B != 0.0 || lb.A == 0.0 || lb.B == 0.0 ) {
		tmp = acos( Math_Absolute(la.A * lb.A + la.B * lb.B) / sqrt( (la.A * la.A + la.B * la.B) * (lb.A * lb.A + lb.B * lb.B) ) );
		if ( tmp > PI / 2 )
			tmp = PI - tmp;
		return tmp;
	}
	else return 0.0;
}

//Line's angle, range is (-pi/2, pi/2]
//Mode other value: easy mode, only calculate k = A/B;
//Mode 1: Precise Mode, A or B = 1.0 needed
double LineAngle( LineABCDouble_t l, uint8_t mode ) {
	double tmp;
	if ( mode == 1 ) {
		if ( l.A == 1.0 ) {
			tmp = PI / 2 - atan(-l.B);
			if ( tmp > PI / 2 )
				tmp -= PI;
		} else if ( l.B == 1.0 ) {
			tmp = atan(-l.A);
		} else {
			if ( Math_Absolute(l.B) < 0.000001 ) {
				tmp = PI / 2;
			} else {
				tmp = atan(- l.A/l.B);
			}
		}
	} else {
		if ( l.B != 0.0 ) {
			tmp = atan(-l.A / l.B);
		} else
			tmp = PI / 2;
	}
	return tmp;
}

uint8_t IsSamePointAndAngle( Point32_t pnt1, uint16_t angle1, Point32_t pnt2, uint16_t angle2,
                             uint32_t pntThres, uint16_t angleThres ) {
	if ( Math_TwoPointsDistance( pnt1.X, pnt1.Y, pnt2.X, pnt2.Y ) < pntThres &&
	     abs( degreeDeltaAngleVector(angle1, angle2) ) < angleThres )
		return 1;
	else return 0;
}
														 
int32_t Math_RoundAngle(int32_t angle)
{
	if (angle >= 1800) 
	{
		angle -= 3600;
	} 
	else if (angle <= -1800) 
	{
		angle += 3600;
	}
	return angle; 
}
int32_t Math_LimitingValue(int32_t input,int32_t limit)
{
	if(input > limit)return limit;
	if(input < 0)return 0;
	return input;
}

int16_t Math_TwoPoint_Angle(Point32_t start_point,Point32_t dest_point)
{
	int16_t rotate_angle = 0;

	rotate_angle = Math_Course2Dest(start_point.X, start_point.Y, dest_point.X, dest_point.Y) - Gyro_GetAngle(0);	

	rotate_angle = Math_RoundAngle(rotate_angle);
	return rotate_angle;
}

uint8_t Math_IsTwoCell_Orthogonal(Point16_t src,Point16_t dest)
{
	if(src.X == dest.X)return 1;
	if(src.Y == dest.Y)return 1;
	
	return 0;
}

uint32_t Math_TwoCell_Dis(Point16_t start_cell,Point16_t exit_cell)
{
	double d, e;

	d = start_cell.X - (double)exit_cell.X;
	e = start_cell.Y - (double)exit_cell.Y;
	d *= d;
	e *= e;

	return (uint32_t)round(sqrt(d + e));
}




