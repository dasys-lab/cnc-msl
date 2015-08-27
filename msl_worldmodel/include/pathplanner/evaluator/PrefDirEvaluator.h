/*
 * SinglePrefDirEvaluator.h
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_PREFDIREVALUATOR_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_PREFDIREVALUATOR_H_

#include <pathplanner/evaluator/PathEvaluator.h>

namespace msl
{

	class PrefDirEvaluator : public PathEvaluator
	{
	public:
		PrefDirEvaluator(PathPlanner* planner);
		virtual ~PrefDirEvaluator();
		double eval(shared_ptr<VoronoiNet> voronoi,
					shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path,
				geometry::CNPoint2D startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode);
	private:
		double clearSpaceWeight;
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_PREFDIREVALUATOR_H_ */