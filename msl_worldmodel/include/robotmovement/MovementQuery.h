/*
 * MovementQuery.h
 *
 *  Created on: Apr 27, 2016
 *      Author: Carpe Noctem
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_MOVEMENTQUERY_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_MOVEMENTQUERY_H_

#include "msl_actuator_msgs/MotionControl.h"
#include "GeometryCalculator.h"

using namespace std;
using namespace msl_actuator_msgs;
namespace msl
{
	class MovementQuery
	{
	public:
		MovementQuery();
		virtual ~MovementQuery();
		shared_ptr<geometry::CNPoint2D> egoAlignPoint;
		shared_ptr<geometry::CNPoint2D> egoDestinationPoint;
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints;
	bool fast = false;
	bool dribble = false;
	double snapDistance;
//	double translation;
	double angleTolerance;
	shared_ptr<geometry::CNPoint2D> teamMatePosition;

// PD variables for RobotMovement::moveToPoint() and RobotMovement::rotationDribblePD()
	double curRotDribble;
	double lastRotDribbleErr;

	// PD variables for RobotMovement::moveToPoint() and RobotMovement::transaltionDribblePD()
	double curTransDribble;
	double transControlIntegralDribble;

};
}
#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_MOVEMENTQUERY_H_ */
