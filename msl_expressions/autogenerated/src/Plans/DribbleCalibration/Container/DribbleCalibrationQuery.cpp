/*
 * DribbleCalibrationQuery.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: cn
 */

#include <msl_msgs/MotionInfo.h>
#include <Plans/DribbleCalibration/Container/DribbleCalibrationQuery.h>

namespace alica
{
	DribbleCalibrationQuery::DribbleCalibrationQuery()
	{
	}

	DribbleCalibrationQuery::~DribbleCalibrationQuery()
	{
		// TODO Auto-generated destructor stub
	}

	const msl_actuator_msgs::BallHandleCmd& DribbleCalibrationQuery::getBhc()
	{
		return this->bhc;
	}

	void DribbleCalibrationQuery::setBhc(const msl_actuator_msgs::BallHandleCmd& bhc)
	{
		this->bhc = bhc;
	}

	const msl_actuator_msgs::MotionControl& DribbleCalibrationQuery::getMc()
	{
		return mc;
	}

	void DribbleCalibrationQuery::setMc(const msl_actuator_msgs::MotionControl& mc)
	{
		std::cout << "mc = " << mc.motion.translation << std::endl;
		std::cout << "this->mc = " << mc.motion.translation << std::endl;
		this->mc = mc;
	}
}
