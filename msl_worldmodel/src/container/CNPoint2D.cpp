/*
 * Point2D.cpp
 *
 *  Created on: 19.11.2014
 *      Author: tobi
 */

#include "container/CNPoint2D.h"


namespace msl {

	CNPoint2D::CNPoint2D(double x, double y) {
		this->x = x;
		this->y = y;
	}


	double CNPoint2D::length() {
		return sqrt(x*x+y*y);
	}

	CNPoint2D CNPoint2D::rotate(double radian) {
		CNPoint2D newPoint(0,0);
		newPoint.x = this->x * cos(radian) - this->y * sin(radian);
		newPoint.y = this->x * sin(radian) + this->y * cos(radian);
		return newPoint;

	}

	double CNPoint2D::alpha(CNPosition me) {
		CNPoint2D p(0,0);
		p = this->alloToEgo(me);
		return atan2(p.y, p.x);
	}

	CNPoint2D CNPoint2D::alloToEgo(CNPosition me) {
		CNPoint2D ego;

		double x = this->x - me.x;
		double y = this->y - me.y;

		double angle = atan2(y, x) - me.theta;
		double dist = sqrt(x * x + y * y);

		ego.x = cos(angle) * dist;
		ego.y = sin(angle) * dist;

		return ego;
	}

	CNPoint2D::~CNPoint2D() {
		// TODO Auto-generated destructor stub
	}
}

