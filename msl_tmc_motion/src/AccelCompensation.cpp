/*
 * AccelCompensation.cpp
 *
 *  Created on: Aug 24, 2015
 *      Author: cnpaul
 */

#include "AccelCompensation.h"
#include <container/CNPoint2D.h>
#include <GeometryCalculator.h>

namespace msl_driver
{

	AccelCompensation::AccelCompensation(double maxAccel, double maxRadAccel) : maxAccel(maxAccel), maxRadAccel(maxRadAccel)
	{
	}

	AccelCompensation::~AccelCompensation()
	{
	}

	msl_msgs::MotionInfo* AccelCompensation::compensate(msl_msgs::MotionInfo* motion, msl_msgs::MotionInfo* motionOld)
	{
		std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now();
		if ((abs(motion->translation) <= 50) && (abs(motion->rotation) <= M_PI / 9.0))
		{
			return motion;
		}

		double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - lastTimestamp).count() / 1000000000.0; // in seconds

		if (motionOld != nullptr && this->maxAccel >= 0.0)
		{
			geometry::CNPoint2D oldVel(motionOld->translation * cos(motionOld->angle),
										motionOld->translation * sin(motionOld->angle));

			geometry::CNPoint2D reqVel(motion->translation * cos(motion->angle),
										motion->translation * sin(motion->angle));

			geometry::CNPoint2D diff = reqVel - oldVel;

			double accel = diff.length() / dt;

			if (accel > this->maxAccel)
			{
				geometry::CNPoint2D newVel = oldVel + diff * (this->maxAccel / accel);
				motion->translation = newVel.length();
				motion->angle = newVel.angleTo();
			}

		}

		if (motionOld != nullptr && maxRadAccel >= 0.0)
		{
			double rotDiff = motion->rotation - motionOld->rotation;
			if (abs(rotDiff / dt) > this->maxRadAccel)
				motion->rotation = motionOld->rotation
						+ geometry::GeometryCalculator::sgn(rotDiff) * this->maxRadAccel * dt;
		}

		this->lastTimestamp = timestamp;

		return motion;
	}

} /* namespace msl_driver */
