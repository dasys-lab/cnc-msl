/*
 * LaserPointGroup.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: cn
 */

#include "laserMotionCalibration/LaserPointGroup.h"

namespace laserMotionCalibration
{

	LaserPointGroup::LaserPointGroup()
	{
	}

	LaserPointGroup::~LaserPointGroup()
	{
	}

	double LaserPointGroup::getCenter()
	{
		return (points.at(points.size() - 1).index + points.at(0).index) / 2.0;
	}

	double LaserPointGroup::getDistance()
	{
		return 1000.0 * points.at(points.size() / 2).distance;
	}

} /* namespace laserMotionCalibration */
