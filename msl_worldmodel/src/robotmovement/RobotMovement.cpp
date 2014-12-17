/*
 * RobotMovement.cpp
 *
 *  Created on: 17.12.2014
 *      Author: tobi
 */

#include "robotmovement/RobotMovement.h"
#include "container/CNPoint2D.h"

#include <SystemConfig.h>

namespace msl {
	double RobotMovement::defaultTranslation;

	RobotMovement::~RobotMovement() {
		// TODO Auto-generated destructor stub
	}


	MotionControl RobotMovement::moveToPointCarefully(
		shared_ptr<CNPoint2D> egoTarget, shared_ptr<CNPoint2D> egoAlignPoint,
		double snapDistance)
	{

		MotionControl mc;

		mc.motion.angle = egoTarget->angleTo();
		mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo();

		if(egoTarget->length() > snapDistance)
		{
			mc.motion.translation = defaultTranslation;
		} else
		{
			mc.motion.translation = 0 ;
		}

		return mc;
	}

	MotionControl RobotMovement::interseptCarefully(shared_ptr<CNPoint2D> egoTarget,
			shared_ptr<CNPoint2D> egoAlignPoint, double snapDistance)
	{

		MotionControl mc;

		mc.motion.angle = egoTarget->angleTo();
		mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo();
		if(egoTarget->length() > snapDistance)

		{
			mc.motion.translation = min(defaultTranslation, egoTarget->length());
		} else
		{
			mc.motion.translation =  0;
		}
		return mc;
	}

	void RobotMovement::readConfigParameters()
	{
		defaultTranslation = (*supplementary::SystemConfig::getInstance())["Drive"]->get<double>("Drive", "DefaultVelocity", NULL);
	}
}
