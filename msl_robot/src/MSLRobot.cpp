/*
 * MSLRobot.cpp
 *
 *  Created on: Jun 12, 2016
 *      Author: Stephan Opfer
 */

#include "msl_robot/MSLRobot.h"
#include "msl_robot/robotmovement/RobotMovement.h"
#include "msl_robot/kicker/Kicker.h"
#include <MSLWorldModel.h>

namespace msl
{
	MSLRobot* MSLRobot::get()
	{
		static MSLRobot instance;
		return &instance;
	}

	MSLRobot::MSLRobot()
	{
		this->wm = MSLWorldModel::get();
		this->robotMovement = new RobotMovement();
		this->kicker = new Kicker(wm);
	}

	MSLRobot::~MSLRobot()
	{
		delete this->robotMovement;
		delete this->kicker;
	}

} /* namespace msl */
