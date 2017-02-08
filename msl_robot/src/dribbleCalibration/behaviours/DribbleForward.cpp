/*
 * DribbleForward.cpp
 *
 *  Created on: Dec 14, 2016
 *      Author: Michael Gottesleben
 */

#include <boost/lexical_cast.hpp>
#include <Configuration.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_robot/dribbleCalibration/behaviours/DribbleForward.h>
#include <msl_robot/dribbleCalibration/container/DribbleCalibrationQuery.h>
#include <SystemConfig.h>
#include <iostream>
#include <string>

namespace msl
{
	DribbleForward::DribbleForward()
	{
		velToInput = 0;
		changingValue = 0;
		defaultValue = 0;
		minValue = 0;
		maxValue = 0;
		actuatorSpeed = 0;
		query = make_shared<DribbleCalibrationQuery>();
		readConfigParameters();
	}

	DribbleForward::~DribbleForward()
	{
	}

	shared_ptr<DribbleCalibrationQuery> DribbleForward::move(int trans)
	{
		shared_ptr<MotionControl> mc;
		shared_ptr<BallHandleCmd> bhc = make_shared<BallHandleCmd>();

		MotionControl mc1 = mCon.move(mCon.Forward, trans);
		mc = make_shared<MotionControl>(mc1);
//		mc->motion.translation = mc1.motion.translation;
//		mc->motion.angle = mc1.motion.angle;
//		mc->motion.rotation = mc1.motion.angle;
		cout << "mc.translation = " << mc->motion.translation << endl;
		query->setMc(mc);
		bhc->leftMotor = actuatorSpeed;
		bhc->rightMotor = actuatorSpeed;
		query->setBhc(bhc);
		return query;
	}

	void DribbleForward::writeConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleCalibration"]->set(boost::lexical_cast<std::string>(actuatorSpeed), "DribbleCalibration.DribbleForward.MeasuredActuatorSpeed", NULL);
	}

	void DribbleForward::adaptParams()
	{
		actuatorSpeed = actuatorSpeed - changingValue;
		if (actuatorSpeed < 0)
		{
			cerr << redBegin << "DribbleForward::adaptParams(): parameter < 0! parameter will be reset" << redEnd
					<< endl;
			resetParams();
		}
//		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
//		(*sc)["DribbleAround"]->set(boost::lexical_cast<std::string>(velToInput), "DribbleAround.velToInput", NULL);
	}

	void DribbleForward::resetParams()
	{
		actuatorSpeed = defaultValue;
	}

	void DribbleForward::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		// DribbleAround.conf
//		velToInput = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.velToInput", NULL);

		// DribbleCalibration.conf
		changingValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.ChangingValue",
		NULL);
		defaultValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.DefaultValue", NULL);
		resetParams();
	}

	void DribbleForward::saveParams()
	{
//		if (maxValue == 0)
//		{
//			cout << "setting minimum parameter value to " << velToInput << "..." << endl;
//			maxValue = velToInput;
//		}
//		else
//		{
//			cout << "setting maximum parameter value to " << velToInput << "..." << endl;
//			minValue = velToInput;
//		}
	}
}

