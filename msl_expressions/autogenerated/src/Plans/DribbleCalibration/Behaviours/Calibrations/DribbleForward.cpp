/*
 * DribbleForward.cpp
 *
 *  Created on: Dec 14, 2016
 *      Author: cn
 */

#include "Plans/DribbleCalibration/Behaviours/Calibrations/DribbleForward.h"
#include "boost/lexical_cast.hpp"

namespace alica
{
	DribbleForward::DribbleForward()
	{
		velToInput = 0;
		changingValue = 0;
		defaultValue = 0;
		readConfigParameters();
	}

	DribbleForward::~DribbleForward()
	{
		// TODO Auto-generated destructor stub
	}

	MotionControl DribbleForward::move(int trans)
	{
		MotionControl mc;
		return mCon.move(mCon.Forward, trans);

	}

	void DribbleForward::writeConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(velToInput), "DribbleAround.velToInput", NULL);
	}

	void DribbleForward::adaptParams()
	{
		velToInput =- changingValue;
	}

	void DribbleForward::resetParams()
	{
		velToInput = defaultValue;
	}

	void DribbleForward::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		// DribbleAround.conf
		velToInput = (*sc)["DribbleAround"]->get<double>("DribbleAround.velToInput", NULL);

		// DribbleCalibration.conf
		changingValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.ChangingValue",
																	NULL);
		defaultValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.DefaultValue", NULL);
	}
}
