/*
 * DribbleCalibrationQuery.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: cn
 */

#include <msl_msgs/MotionInfo.h>
#include <msl_robot/dribbleCalibration/container/DribbleCalibrationQuery.h>

namespace msl
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

	void DribbleCalibrationQuery::setBhc(const std::shared_ptr<msl_actuator_msgs::BallHandleCmd> bhc)
	{
		this->bhc = bhc;
	}

	const std::shared_ptr<msl_actuator_msgs::MotionControl>& DribbleCalibrationQuery::getMc()
	{
		return mc;
	}

	void DribbleCalibrationQuery::setMc(const std::shared_ptr<msl_actuator_msgs::MotionControl> mc)
	{
		std::cout << "this->mc =" << this->mc->motion.translation << std::endl;
		this->mc = mc;
	}

}
