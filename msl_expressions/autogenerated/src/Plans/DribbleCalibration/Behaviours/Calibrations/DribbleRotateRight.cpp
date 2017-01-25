/*
 * DribbleRotateRight.cpp
 *
 *  Created on: Jan 25, 2017
 *      Author: cn
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
	}

	DribbleRotateRight::~DribbleRotateRight()
	{
	}

	MotionControl DribbleRotateRight::move(int trans)
	{
		MotionControl mc;
		return mc;
	}

	void DribbleRotateRight::writeConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(epsilonRot), "DribbleAround.velToInput", NULL);
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
	}

	void DribbleRotateRight::adaptParams()
	{
		epsilonRot =- changingValue;
	}

	void DribbleRotateRight::resetParams()
	{
		epsilonRot = defaultValue;
	}

} /* namespace alica */
