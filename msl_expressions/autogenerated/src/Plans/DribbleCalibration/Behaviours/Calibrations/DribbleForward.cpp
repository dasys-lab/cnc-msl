/*
 * DribbleForward.cpp
 *
 *  Created on: Dec 14, 2016
 *      Author: Michael Gottesleben
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
		minValue = 0;
		maxValue = 0;
		readConfigParameters();
	}

	DribbleForward::~DribbleForward()
	{
	}

	MotionControl DribbleForward::move(int trans)
	{
		MotionControl mc;
		return mCon.move(mCon.Forward, trans);

	}

	void DribbleForward::writeConfigParameters()
	{
		double endValue = (minValue + maxValue) / 2;
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(endValue), "DribbleAround.velToInput", NULL);
	}

	void DribbleForward::adaptParams()
	{
		velToInput = -changingValue;
		if (velToInput < 0)
		{
			cerr << redBegin << "DribbleForward::adaptParams(): parameter < 0! parameter will be reset" << redEnd
					<< endl;
			resetParams();
		}
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(velToInput), "DribbleAround.velToInput", NULL);
	}

	void DribbleForward::resetParams()
	{
		velToInput = defaultValue;
	}

	void DribbleForward::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		// DribbleAround.conf
		velToInput = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.velToInput", NULL);

		// DribbleCalibration.conf
		changingValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.ChangingValue",
		NULL);
		defaultValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.DefaultValue", NULL);
	}

	void DribbleForward::saveParams()
	{
		if (maxValue == 0)
		{
			cout << "setting minimum parameter value to " << velToInput << "..." << endl;
			maxValue = velToInput;
		}
		else
		{
			cout << "setting maximum parameter value to " << velToInput << "..." << endl;
			minValue = velToInput;
		}
	}
}

