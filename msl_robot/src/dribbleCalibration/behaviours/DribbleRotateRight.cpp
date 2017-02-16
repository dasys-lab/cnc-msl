/*
 * DribbleRotateRight.cpp
 *
 *  Created on: Jan 25, 2017
 *      Author: Michael Gottesleben
 */

#include <boost/lexical_cast.hpp>
#include <Configuration.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_msgs/MotionInfo.h>
#include <msl_robot/dribbleCalibration/behaviours/DribbleRotateRight.h>
#include <msl_robot/dribbleCalibration/container/DribbleCalibrationQuery.h>
#include <iostream>
#include <memory>
#include <string>

namespace msl
{

	DribbleRotateRight::DribbleRotateRight()
	{
		epsilonRot = 0;
		changingValue = 0;
		defaultValue = 0;
		rotationSpeed = 0;
		minValue = 0;
		maxValue = 0;
		readConfigParameters();
	}

	DribbleRotateRight::~DribbleRotateRight()
	{
	}

	shared_ptr<DribbleCalibrationQuery> DribbleRotateRight::move(int trans)
	{
		shared_ptr<MotionControl> mc;
		mc->motion.rotation = -rotationSpeed;
		shared_ptr<DribbleCalibrationQuery> query;
		query->setMc(mc);
		return query;
	}

	void DribbleRotateRight::writeConfigParameters()
	{
		double endValue = (minValue + maxValue) / 2;
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(epsilonRot), "DribbleAround.epsilonRot", NULL);
	}

	void DribbleRotateRight::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		// DribbleAround.conf
		epsilonRot = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.epsilonRot", NULL);

		// DribbleCalibration.conf
		changingValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleRotation.ChangingValue",
		NULL);
		defaultValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleRotation.DefaultValue",
		NULL);

		rotationSpeed = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleRotation.RotationSpeed",
		NULL);
	}

	void DribbleRotateRight::adaptParams()
	{
		epsilonRot = -changingValue;
		if (epsilonRot < 0)
		{
			cerr << redBegin << "DribbleRotateRight::adaptParams(): parameter < 0! parameter will be reset" << redEnd
					<< endl;
			resetParams();
		}
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(epsilonRot), "DribbleAround.epsilonRot", NULL);
	}

	void DribbleRotateRight::resetParams()
	{
		epsilonRot = defaultValue;
	}

	void DribbleRotateRight::saveParams()
	{
		if (maxValue == 0)
		{
			cout << "setting minimum parameter value to " << epsilonRot << "..." << endl;
			maxValue = epsilonRot;
		}
		else
		{
			cout << "setting maximum parameter value to " << epsilonRot << "..." << endl;
			minValue = epsilonRot;
		}
	}

} /* namespace alica */
