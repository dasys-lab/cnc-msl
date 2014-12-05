/*
 * Point2D.cpp
 *
 *  Created on: 19.11.2014
 *      Author: tobi
 */

#include "container/CNPoint2D.h"
#include "container/CNPosition.h"

using namespace std;

namespace msl {

	CNPoint2D::CNPoint2D(double x, double y) {
		this->x = x;
		this->y = y;
	}

	double CNPoint2D::length() {
		return sqrt(x * x + y * y);
	}

	CNPoint2D CNPoint2D::rotate(double radian) {
		CNPoint2D newPoint(0, 0);
		newPoint.x = this->x * cos(radian) - this->y * sin(radian);
		newPoint.y = this->x * sin(radian) + this->y * cos(radian);
		return newPoint;

	}

	double CNPoint2D::angleTo(CNPosition& me) {
		shared_ptr<CNPoint2D> p;
		p = this->alloToEgo(me);
		return atan2(p->y, p->x);
	}

	shared_ptr<CNPoint2D> CNPoint2D::alloToEgo(CNPosition& me) {
		shared_ptr<CNPoint2D> ego = make_shared<CNPoint2D>();

		double x = this->x - me.x;
		double y = this->y - me.y;

		double angle = atan2(y, x) - me.theta;
		double dist = sqrt(x * x + y * y);

		ego->x = cos(angle) * dist;
		ego->y = sin(angle) * dist;

		return ego;
	}

	shared_ptr<CNPoint2D> CNPoint2D::egoToAllo(CNPosition& me) {
		shared_ptr<CNPoint2D> allo = make_shared<CNPoint2D>();
		allo->x = me.x;
		allo->x = me.y;

		allo->x += cos(me.theta) * x - sin(me.theta) * y;
		allo->y += sin(me.theta) * x + cos(me.theta) * y;

		return allo;
	}

	CNPoint2D::~CNPoint2D() {
		// TODO Auto-generated destructor stub
	}
}

