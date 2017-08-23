/*
 * LaserPointGroup.h
 *
 *  Created on: Aug 23, 2017
 *      Author: cn
 */

#ifndef SRC_LASERPOINTGROUP_H_
#define SRC_LASERPOINTGROUP_H_

#include <vector>

namespace laserMotionCalibration
{

	struct LaserPoint {
		int index;
		float intensity;
		float distance;

		LaserPoint(int newIndex, float newIntensity, float newDistance)
		{
			index = newIndex;
			intensity = newIntensity;
			distance = newDistance;
		}
	};

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
