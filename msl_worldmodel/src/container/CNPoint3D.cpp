/*
 * Point2D.cpp
 *
 *  Created on: 19.11.2014
 *      Author: tobi
 */

#include "container/CNPoint3D.h"
#include "container/CNPosition.h"

using namespace std;

namespace msl {

	CNPoint3D::CNPoint3D(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	double CNPoint3D::length() {
		return sqrt(x * x + y * y + z * z);
	}

	CNPoint3D::~CNPoint3D() {
	}
}

