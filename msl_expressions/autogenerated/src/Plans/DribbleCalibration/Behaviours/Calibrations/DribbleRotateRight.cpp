/*
 * DribbleRotateRight.cpp
 *
 *  Created on: Jan 25, 2017
 *      Author: Michael Gottesleben
 */

#include "Plans/DribbleCalibration/Behaviours/Calibrations/DribbleRotateRight.h"
#include "boost/lexical_cast.hpp"

namespace alica
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

	MotionControl DribbleRotateRight::move(int trans)
	{
		MotionControl mc;
		mc.motion.rotation = -rotationSpeed;
		return mc;
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
