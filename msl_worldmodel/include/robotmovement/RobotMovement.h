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

namespace geometry
{
	class CNPoint2D;
}

using namespace std;
using namespace msl_actuator_msgs;

namespace msl
{

	class RobotMovement
	{
	public:
		virtual ~RobotMovement();
		static MotionControl moveToPointCarefully(shared_ptr<geometry::CNPoint2D> egoTarget,
													shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = nullptr);
		static MotionControl interceptCarefully(shared_ptr<geometry::CNPoint2D> egoTarget,
												shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = nullptr);
		//TODO needs to be tested
		static MotionControl alignToPointNoBall(shared_ptr<geometry::CNPoint2D> egoTarget,
												shared_ptr<geometry::CNPoint2D> egoAlignPoint, double angleTolerance);
		static MotionControl alignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
													shared_ptr<geometry::CNPoint2D> egoBallPos, double angleTolerance,
													double ballAngleTolerance);
		//TODO needs to be tested
		static MotionControl rapidAlignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
													shared_ptr<geometry::CNPoint2D> egoBallPos, double angleTolerance,
													double ballAngleTolerance);

		static void readConfigParameters();
		static double defaultTranslation;
		static double defaultRotateP;
		static double interceptCarfullyRotateP;

	private:
		static double lastRotError;
		static double lastRotErrorWithBall;
		static double alignToPointMinRotation;
		static double alignToPointMaxRotation;
		static double lastRotErrorWithBallRapid;
		static double alignToPointRapidMaxRotation;
		static double alignToPointpRot;
		static double alignMaxVel;
	};
}

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ROBOTMOVEMENT_H_ */
