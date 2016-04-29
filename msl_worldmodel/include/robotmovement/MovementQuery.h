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

class MovementQuery
{
public:
	MovementQuery();
	virtual ~MovementQuery();
	shared_ptr<geometry::CNPoint2D> egoAlignPoint;
	shared_ptr<geometry::CNPoint2D> egoDestinationPoint;
//	double angleTolerance;
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints;
	bool fast = false;
	bool dribble = false;
};

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_MOVEMENTQUERY_H_ */
