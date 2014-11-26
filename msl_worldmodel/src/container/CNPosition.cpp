/*
 * CNPosition.cpp
 *
 *  Created on: 19.11.2014
 *      Author: tobi
 */

#include "container/CNPosition.h"
namespace msl {


	CNPosition::CNPosition(double x, double y, double theta) {
		this->x = x;
		this->y = y;
		this->theta = theta;
	}

	CNPosition::~CNPosition() {
		// TODO Auto-generated destructor stub
	}
}

