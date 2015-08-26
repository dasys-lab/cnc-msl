/*
 * AccelCompensation.cpp
 *
 *  Created on: Aug 24, 2015
 *      Author: cnpaul
 */

#include "AccelCompensation.h"
#include <container/CNPoint2D.h>
#include <GeometryCalculator.h>
#include <memory>

namespace msl_driver
{

	AccelCompensation::AccelCompensation(double maxAccel, double maxRadAccel)
	{
		this->maxAccel = maxAccel;
		this->maxRadAccel = maxRadAccel;

	}

	AccelCompensation::~AccelCompensation()
	{
	}


	msl_msgs::MotionInfo* AccelCompensation::compensate(msl_msgs::MotionInfo* motion, msl_msgs::MotionInfo* motionOld, ulong timestamp)
	{
		if( (abs(motion->translation) <= 50) && (abs(motion->rotation) <= M_PI / 9.0))
		{
			return motion;
		}

		double dt = (timestamp - lastTimestamp) / 10000000.0;

		if( motionOld != nullptr && this->maxAccel >= 0.0)
		{
			shared_ptr<geometry::CNPoint2D> oldVel = make_shared<geometry::CNPoint2D> (motionOld->translation * cos(motionOld->angle),
											motionOld->translation * sin(motionOld->angle));

			shared_ptr<geometry::CNPoint2D> reqVel = make_shared<geometry::CNPoint2D>(motion->translation * cos(motion->angle),
														motion->translation * sin(motion->angle));

			shared_ptr<geometry::CNPoint2D>  diff = oldVel - reqVel;

			double accel = diff->length() - dt;

			shared_ptr<geometry::CNPoint2D> newVel = nullptr;
			if(accel > this->maxAccel)
			{
				newVel = oldVel + diff * (this->maxAccel / accel);
				motion->translation = newVel->length();
				motion->angle = newVel->angleTo();
			}

		}

		if(motionOld != nullptr && maxRadAccel >= 0.0)
		{
			double rotDiff = motion->rotation - motionOld->rotation;
			if(abs(rotDiff / dt) > this->maxRadAccel)
				motion->rotation = motionOld->rotation + geometry::GeometryCalculator::sgn(rotDiff) * this->maxRadAccel * dt;
		}

		this->lastTimestamp = timestamp;

		return motion;
	}

} /* namespace msl_driver */
