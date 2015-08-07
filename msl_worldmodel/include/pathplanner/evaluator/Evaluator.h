/*
 * SingleEvaluator.h
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_EVALUATOR_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_EVALUATOR_H_

#include <pathplanner/evaluator/PathEvaluator.h>

namespace msl
{

	class Evaluator : public PathEvaluator
	{
	public:
		Evaluator(PathPlanner* planner);
		virtual ~Evaluator();
		double eval(shared_ptr<VoronoiNet> voronoi, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path,
					geometry::CNPoint2D startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode);
};

}
	/* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_EVALUATOR_H_ */
