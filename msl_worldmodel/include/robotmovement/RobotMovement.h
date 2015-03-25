/*
 * RobotMovement.h
 *
 *  Created on: 17.12.2014
 *      Author: tobi
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ROBOTMOVEMENT_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ROBOTMOVEMENT_H_

#include <memory>
#include "msl_actuator_msgs/MotionControl.h"

using namespace std;
using namespace msl_actuator_msgs;


namespace msl {
class CNPoint2D;
	class RobotMovement {
	public:
		virtual ~RobotMovement();
		static MotionControl moveToPointCarefully(shared_ptr<CNPoint2D> egoTarget, shared_ptr<CNPoint2D> egoAlignPoint, double snapDistance);
		static MotionControl interceptCarefully(shared_ptr<CNPoint2D> egoTarget, shared_ptr<CNPoint2D> egoAlignPoint, double snapDistance);
		static void readConfigParameters();
		static double defaultTranslation;
		static double defaultRotateP;
		static double interceptCarfullyRotateP;
	};
}

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ROBOTMOVEMENT_H_ */
