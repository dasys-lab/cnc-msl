/*
 * DribbleCalibrationContainer.cpp
 *
 *  Created on: Jul 22, 2016
 *      Author: Carpe Noctem
 */

#include <Plans/DribbleCalibration/Container/DribbleCalibrationContainer.h>

#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>

namespace alica
{

	DribbleCalibrationContainer::DribbleCalibrationContainer()
	{
		this->wm = msl::MSLWorldModel::get();
	}

	DribbleCalibrationContainer::~DribbleCalibrationContainer()
	{

	}

    msl_actuator_msgs::MotionControl DribbleCalibrationContainer::getBall()
	{
		msl::RobotMovement rm;
		msl_actuator_msgs::MotionControl mc;
		auto me = wm->rawSensorData->getOwnPositionVision();
		auto egoBallPos = wm->ball->getAlloBallPosition()->alloToEgo(*me);

		query->egoDestinationPoint = egoBallPos;
		query->egoAlignPoint = egoBallPos;

		mc = rm.moveToPoint(query);
		return mc;
	}

	double DribbleCalibrationContainer::readConfigParameter(const char *path)
	{
		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
		return (*sys)["Actuation"]->get<double>(path, NULL);
	}

	void DribbleCalibrationContainer::writeConfigParameters(shared_ptr<vector<subsection>> sections)
	{
		for (subsection section : *sections)
		{

		}

	}
} /* namespace alica */
