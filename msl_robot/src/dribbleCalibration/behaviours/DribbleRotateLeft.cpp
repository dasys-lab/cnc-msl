/*
 * DribbleRotateLeft.cpp
 *
 *  Created on: Jan 25, 2017
 *      Author: Michael Gottesleben
 */

#include <boost/lexical_cast.hpp>
#include <Configuration.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_msgs/MotionInfo.h>
#include <msl_robot/dribbleCalibration/behaviours/DribbleRotateLeft.h>
#include <msl_robot/dribbleCalibration/container/DribbleCalibrationQuery.h>
#include <SystemConfig.h>
#include <iostream>
#include <string>

namespace msl
{

	DribbleRotateLeft::DribbleRotateLeft()
	{
		epsilonRot = 0;
		changingValue = 0;
		defaultValue = 0;
		rotationSpeed = 0;
		minValue = 0;
		maxValue = 0;
		readConfigParameters();
	}

	DribbleRotateLeft::~DribbleRotateLeft()
	{
	}

	shared_ptr<DribbleCalibrationQuery> DribbleRotateLeft::move(int trans)
	{
		shared_ptr<msl_actuator_msgs::MotionControl> mc;

		mc->motion.rotation = rotationSpeed;
		shared_ptr<DribbleCalibrationQuery> query;
		query->setMc(mc);
		return query;
	}

	void DribbleRotateLeft::writeConfigParameters()
	{
		double endValue = (minValue + maxValue) / 2;
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(epsilonRot), "DribbleAround.epsilonRot", NULL);
	}

	void DribbleRotateLeft::readConfigParameters()
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

	void DribbleRotateLeft::adaptParams()
	{
		epsilonRot = -changingValue;

		if (epsilonRot < 0)
		{
			cerr << redBegin << "DribbleRotateLeft::adaptParams(): parameter < 0! parameter will be reset" << redEnd
					<< endl;
			resetParams();
		}

		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(epsilonRot), "DribbleAround.epsilonRot", NULL);
	}

	void DribbleRotateLeft::resetParams()
	{
		epsilonRot = defaultValue;
	}

	void DribbleRotateLeft::saveParams()
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
