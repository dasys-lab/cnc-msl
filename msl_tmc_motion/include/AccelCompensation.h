/*
 * AccelCompensation.h
 *
 *  Created on: Aug 24, 2015
 *      Author: cnpaul
 */

#ifndef CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_ACCELCOMPENSATION_H_
#define CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_ACCELCOMPENSATION_H_

#include <msl_msgs/MotionInfo.h>
#include <cmath>
#include <math.h>
#include <memory>

namespace msl_driver
{

	class AccelCompensation
	{
	public:
		AccelCompensation(double maxAccel, double maxRadAccel);
		virtual ~AccelCompensation();
		msl_msgs::MotionInfo* compensate(msl_msgs::MotionInfo* motion, msl_msgs::MotionInfo* motionOld, ulong timestamp);
	protected:
		ulong lastTimestamp = 0;
		double maxAccel = 0.0;
		double maxRadAccel = 0.0;
	};

} /* namespace msl_driver */

#endif /* CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_ACCELCOMPENSATION_H_ */
