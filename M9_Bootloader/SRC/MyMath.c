#include "MyMath.h"
#include "USART.h"

double absolute(double d)
{
	return ((d < 0) ? (d * (-1)) : d);
}

double deg2rad(double deg, int8_t scale)
{
	return (deg * PI / (180 * scale));
}

double rad2deg(double rad, int8_t scale)
{
	return (scale * rad * 180 / PI);
}

uint16_t course2dest(int32_t startx, int32_t starty, int32_t destx, int32_t desty)
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
		alpha = round(rad2deg(atan(((double)(desty - starty) / (destx - startx))), 10));

		if (destx < startx) {
			alpha += 1800;
		}

		if (alpha < 0) {
			alpha += 3600;
		}
	}

	return (uint16_t)alpha;
}

uint32_t TwoPointsDistance(int32_t startx, int32_t starty, int32_t destx, int32_t desty)
{
	double d, e;

	d = destx - (double)startx;
	e = desty - (double)starty;
	d *= d;
	e *= e;

	return (uint32_t)round(sqrt(d + e));
}

int32_t TwoPointsDistanceAtDirection(int32_t startx, int32_t starty, int32_t destx, int32_t desty, int16_t theta)
{
	return (int32_t)round(cos(deg2rad(theta, 10)) * (destx - startx) + sin(deg2rad(theta, 10)) * (desty - starty));
}

int16_t distance2line(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t px, int32_t py) {
	double dx = x2 - x1;
	float dy = y2 - y1;
	if ((dx == 0) && (dy == 0)) {
		dx = px - x1;
		dy = py - y1;
		return (int16_t)(sqrt(dx * dx + dy * dy));
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

	theta = deg2rad(theta, 10);

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
	if ( absolute(l1[0] - l1[2]) < 0.1 && absolute(l2[0] - l2[2]) < 0.1 ) {
		p[0] = 32666;
		p[1] = 32666;
	}
	//Two lines are horizontal
	else if ( absolute((l1[1] - l1[3]) / (l1[0] - l1[2]) -
	                   (l2[1] - l2[3]) / (l2[0] - l2[2])) < 0.1 ) {
		p[0] = 32767;
		p[1] = 32767;
	}
	//Line l1 is horizontal and line l2 is not vertical
	else if ( absolute(l1[1] - l1[3]) < 0.1 && absolute(l2[2] - l2[0]) > 0.1 ) {
		p[0] = (l1[1] - l2[1]) * (l2[2] - l2[0]) / (l2[3] - l2[1]) + l2[0];
		p[1] = l1[1];
	}
	//Line l1 is horizontal and line l2 is vertical
	else if ( absolute(l1[1] - l1[3]) < 0.1 && absolute(l2[2] - l2[0]) < 0.1 ) {
		p[0] = l2[0];
		p[1] = l1[1];
	}
	//Line l1 is not horizontal and line l2 is not vertical, math calculation
	else if ( absolute(l1[1] - l1[3]) > 0.1 && absolute(l2[2] - l2[0]) > 0.1 ) {
		double delta = ( l2[2] - l2[0] ) * ( l1[3] - l1[1] ) -
		               ( l2[3] - l2[1] ) * ( l1[2] - l1[0] ),
		          px = ( l2[1] - l1[1] ) * ( l1[0] - l1[2] ) * ( l2[0] - l2[2] ) -
		               l2[0] * ( l2[1] - l2[3] ) * ( l1[0] - l1[2] ) +
		               l1[0] * ( l1[1] - l1[3] ) * ( l2[0] - l2[2] );
		p[0] = px / delta;
		p[1] = ( l2[1] - l2[3] ) * ( p[0] - l2[0] ) / ( l2[0] - l2[2] ) + l2[1];
	}
	//Line l1 is not horizontal and line l2 is vertical
	else if ( absolute(l1[1] - l1[3]) > 0.1 && absolute(l2[2] - l2[0]) > 0.1 ) {
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
double TwoLinesAngle( LineABC la, LineABC lb ) {
	double tmp;
	if ( la.A != 0.0 || la.B != 0.0 || lb.A == 0.0 || lb.B == 0.0 ) {
		tmp = acos( absolute(la.A * lb.A + la.B * lb.B) / sqrt( (la.A * la.A + la.B * la.B) * (lb.A * lb.A + lb.B * lb.B) ) );
		if ( tmp > PI / 2 )
			tmp = PI - tmp;
		return tmp;
	}
	else return 0.0;
}

//Line's angle, range is (-pi/2, pi/2]
//Mode other value: easy mode, only calculate k = A/B;
//Mode 1: Precise Mode, A or B = 1.0 needed
double LineAngle( LineABC l, uint8_t mode ) {
	double tmp;
	if ( mode == 1 ) {
		if ( l.A == 1.0 ) {
			tmp = PI / 2 - atan(-l.B);
			if ( tmp > PI / 2 )
				tmp -= PI;
		} else if ( l.B == 1.0 ) {
			tmp = atan(-l.A);
		} else {
			if ( absolute(l.B) < 0.000001 ) {
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
	if ( TwoPointsDistance( pnt1.X, pnt1.Y, pnt2.X, pnt2.Y ) < pntThres &&
	     abs( degreeDeltaAngleVector(angle1, angle2) ) < angleThres )
		return 1;
	else return 0;
}
