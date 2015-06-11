/*
 * RobotMovement.cpp *
 *
 *  Created on: 17.12.2014
 *      Author: tobi
 */

#include "robotmovement/RobotMovement.h"
#include "container/CNPoint2D.h"

#include <SystemConfig.h>

namespace msl
{
	double RobotMovement::defaultTranslation;
	double RobotMovement::defaultRotateP;
	double RobotMovement::interceptCarfullyRotateP;

	RobotMovement::~RobotMovement()
	{
		// TODO Auto-generated destructor stub
	}

	MotionControl RobotMovement::moveToPointCarefully(shared_ptr<geometry::CNPoint2D> egoTarget,
														shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance)
	{
		MotionControl mc;
		mc.motion.angle = egoTarget->angleTo();
		mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo() * defaultRotateP;
		if (egoTarget->length() > snapDistance)
		{
			mc.motion.translation = std::min(egoTarget->length(), defaultTranslation);
		}
		else
		{
			mc.motion.translation = 0;
		}

		return mc;
	}

	MotionControl RobotMovement::interceptCarefully(shared_ptr<geometry::CNPoint2D> egoTarget,
													shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance)
	{
		MotionControl mc;
		mc.motion.angle = egoTarget->angleTo();
		mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo() * interceptCarfullyRotateP;
		if (egoTarget->length() > snapDistance)
		{
			mc.motion.translation = min(defaultTranslation, egoTarget->length());
		}
		else
		{
			mc.motion.translation = 0;
		}
		return mc;
	}

	void RobotMovement::readConfigParameters()
	{
		defaultTranslation = (*supplementary::SystemConfig::getInstance())["Drive"]->get<double>("Drive",
																									"DefaultVelocity",	NULL);
		defaultRotateP = (*supplementary::SystemConfig::getInstance())["Drive"]->get<double>("Drive", "DefaultRotateP", NULL);
		interceptCarfullyRotateP = (*supplementary::SystemConfig::getInstance())["Drive"]->get<double>("Drive", "InterceptCarefullyRotateP", NULL);
	}
}
