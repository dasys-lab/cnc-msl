/*
 * DribbleBackward.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: Michael Gottesleben
 */

#include "Plans/DribbleCalibration/Behaviours/Calibrations/DribbleBackward.h"
#include "boost/lexical_cast.hpp"

namespace alica
{
	DribbleBackward::DribbleBackward()
	{
		epsilonT = 0;
		changingValue = 0;
		defaultValue = 0;
		minValue = 0;
		maxValue = 0;
		readConfigParameters();
	}

	DribbleBackward::~DribbleBackward()
	{
	}

	MotionControl DribbleBackward::move(int trans)
	{
		MotionControl mc;
		return mCon.move(mCon.Backward, trans);
	}

	void DribbleBackward::writeConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(epsilonT), "DribbleAround.epsilonT", NULL);
	}

	void DribbleBackward::adaptParams()
	{
		epsilonT = -changingValue;
		if (epsilonT < 0)
		{
			cerr << redBegin << "DribbleBackward::adaptParams(): parameter < 0! parameter will be reset" << redEnd
					<< endl;
			resetParams();
		}
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(epsilonT), "DribbleAround.epsilonT", NULL);
	}

	void DribbleBackward::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		// DribbleAround.conf
		epsilonT = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.epsilonT", NULL);

		// DribbleCalibration.conf
		changingValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleBackward.ChangingValue",
		NULL);
		defaultValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleBackward.DefaultValue",
		NULL);
	}

	void DribbleBackward::resetParams()
	{
		epsilonT = defaultValue;
	}

	void DribbleBackward::saveParams()
	{
		if (maxValue == 0)
		{
			cout << "setting minimum parameter value to " << epsilonT << "..." << endl;
			maxValue = epsilonT;
		}
		else
		{
			cout << "setting maximum parameter value to " << epsilonT << "..." << endl;
			minValue = epsilonT;
		}
	}

}

