/*
 * SingleEvaluator.h
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_SINGLEEVALUATOR_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_SINGLEEVALUATOR_H_

#include <pathplanner/evaluator/PathEvaluator.h>

namespace msl
{

	class SingleEvaluator : public PathEvaluator
	{
	public:
		SingleEvaluator(PathPlanner* planner);
		virtual ~SingleEvaluator();
		double eval(double costsSoFar, shared_ptr<vector<shared_ptr<CNPoint2D>>> path, CNPoint2D goal, double nextEdgeLength);
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_SINGLEEVALUATOR_H_ */
