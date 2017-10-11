/*
 * LaserPoint.cpp
 *
 *  Created on: Aug 30, 2017
 *      Author: cn
 */

#include <laserMotionCalibration/LaserPoint.h>
namespace laserMotionCalibration
{

	float LaserPoint::angleIncrement = LaserPoint::UNDEFINED_VALUE;
	float LaserPoint::minAngle = LaserPoint::UNDEFINED_VALUE;
	LaserPoint::LaserPoint(int newIndex, float newIntensity, float newDistance)
	{
		index = newIndex;
		intensity = newIntensity;
		distance = newDistance;

	}

	LaserPoint::~LaserPoint()
	{
		// TODO Auto-generated destructor stub
	}

	std::shared_ptr<geometry::CNPoint2D> LaserPoint::getXY()
	{

		std::shared_ptr < geometry::CNPoint2D > point = std::make_shared<geometry::CNPoint2D>();
		double angle = ((double)LaserPoint::minAngle) + index * LaserPoint::angleIncrement;
		point->x = cos(angle) * distance;
		point->y = sin(angle) * distance;
		return point;
	}

	void LaserPoint::setMinAngle(float minAngle)
	{
		if (LaserPoint::minAngle != UNDEFINED_VALUE)
		{
			std::cout << "WARNING minAngle already set!" << std::endl;
		}
		LaserPoint::minAngle = minAngle;
	}

	void LaserPoint::setAngleIncrement(float angleIncrement)
	{
		if (LaserPoint::angleIncrement != UNDEFINED_VALUE)
		{
			std::cout << "WARNING angleIncrement already set!" << std::endl;
		}
		LaserPoint::angleIncrement = angleIncrement;
	}

}
