/*
 * LaserPoint.h
 *
 *  Created on: Aug 30, 2017
 *      Author: cn
 */

#ifndef INCLUDE_LASERMOTIONCALIBRATION_LASERPOINT_H_
#define INCLUDE_LASERMOTIONCALIBRATION_LASERPOINT_H_

#include <memory>
#include <container/CNPoint2D.h>

namespace laserMotionCalibration
{
	class LaserPoint
	{
	public:
		LaserPoint(int newIndex, float newIntensity, float newDistance);
		virtual ~LaserPoint();

		int index;
		float intensity;
		float distance;

		std::shared_ptr<geometry::CNPoint2D> getXY();
		static void setMinAngle(float minAngle);
		static void setAngleIncrement(float angleIncrement);
	private:
		static float minAngle;
		static float angleIncrement;
		static constexpr float UNDEFINED_VALUE = -1.0f;
	};
}

#endif /* INCLUDE_LASERMOTIONCALIBRATION_LASERPOINT_H_ */
