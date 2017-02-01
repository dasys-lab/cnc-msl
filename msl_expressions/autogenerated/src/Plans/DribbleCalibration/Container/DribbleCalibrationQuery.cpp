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
		mc = std::make_shared<msl_actuator_msgs::MotionControl>();
		bhc = std::make_shared<msl_actuator_msgs::BallHandleCmd>();
	}

	DribbleCalibrationQuery::~DribbleCalibrationQuery()
	{
		// TODO Auto-generated destructor stub
	}

	const std::shared_ptr<msl_actuator_msgs::BallHandleCmd>& DribbleCalibrationQuery::getBhc()
	{
		return bhc;
	}

	void DribbleCalibrationQuery::setBhc(const std::shared_ptr<msl_actuator_msgs::BallHandleCmd>& bhc)
	{
		this->bhc = bhc;
	}

	const std::shared_ptr<msl_actuator_msgs::MotionControl>& DribbleCalibrationQuery::getMc()
	{
		return mc;
	}

	void DribbleCalibrationQuery::setMc(const std::shared_ptr<msl_actuator_msgs::MotionControl>& mc)
	{
		this->mc = mc;
	}

}
