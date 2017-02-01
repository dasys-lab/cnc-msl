/*
 * DribbleBackward.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: Michael Gottesleben
 */

#include <boost/lexical_cast.hpp>
#include <Configuration.h>
#include <Plans/DribbleCalibration/Behaviours/Calibrations/DribbleBackward.h>
#include <Plans/DribbleCalibration/Container/DribbleCalibrationQuery.h>
#include <SystemConfig.h>
#include <iostream>
#include <string>

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

	shared_ptr<DribbleCalibrationQuery> DribbleBackward::move(int trans)
	{
		MotionControl mc;
		BallHandleCmd bhc;

		shared_ptr<DribbleCalibrationQuery> query;
		mc = mCon.move(mCon.Forward, trans);
		query->setMc(mc);
		bhc.leftMotor = actuatorSpeed;
		bhc.rightMotor = actuatorSpeed;
		query->setBhc(bhc);
		return query;
	}

	void DribbleBackward::writeConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleCalibration"]->set(boost::lexical_cast<std::string>(actuatorSpeed), "DribbleCalibration.DribbleBackward.MeasuredActuatorSpeed", NULL);
	}

	void DribbleBackward::adaptParams()
	{
		actuatorSpeed = actuatorSpeed + changingValue;
		if (epsilonT < 0)
		{
			cerr << redBegin << "DribbleBackward::adaptParams(): parameter < 0! parameter will be reset" << redEnd << endl;
			resetParams();
		}
//		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
//		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(epsilonT), "DribbleAround.epsilonT", NULL);
	}

	void DribbleBackward::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		// DribbleAround.conf
//		epsilonT = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.epsilonT", NULL);

// DribbleCalibration.conf
		changingValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleBackward.ChangingValue",
		NULL);
		defaultValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleBackward.DefaultValue",
		NULL);
		resetParams();
	}

	void DribbleBackward::resetParams()
	{
		actuatorSpeed = defaultValue;
	}

	void DribbleBackward::saveParams()
	{
//		if (maxValue == 0)
//		{
//			cout << "setting minimum parameter value to " << epsilonT << "..." << endl;
//			maxValue = epsilonT;
//		}
//		else
//		{
//			cout << "setting maximum parameter value to " << epsilonT << "..." << endl;
//			minValue = epsilonT;
//		}
	}

}

