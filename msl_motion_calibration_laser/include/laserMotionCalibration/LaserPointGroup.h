/*
 * LaserPointGroup.h
 *
 *  Created on: Aug 23, 2017
 *      Author: cn
 */

#ifndef SRC_LASERPOINTGROUP_H_
#define SRC_LASERPOINTGROUP_H_

#include <vector>
#include "LaserPoint.h"

namespace laserMotionCalibration
{

	class LaserPointGroup
	{
	public:
		LaserPointGroup();
		virtual ~LaserPointGroup();
		double getCenter();
		double getDistance();
		std::vector<LaserPoint> points;
	};

} /* namespace laserMotionCalibration */

#endif /* SRC_LASERPOINTGROUP_H_ */
