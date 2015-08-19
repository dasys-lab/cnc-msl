/*
 * IPathEvaluator.h
 *
 *  Created on: Aug 17, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_IPATHEVALUATOR_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_IPATHEVALUATOR_H_
#include <memory>

using namespace std;

namespace msl
{

	class VoronoiNet;
	/**
	 * Interface class for Pathplanning evaluators
	 */
	class IPathEvaluator
	{
	public:
		virtual ~IPathEvaluator() {}
		virtual double eval(shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal,
									shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode,
									VoronoiNet* voronoi = nullptr,
									shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path= nullptr) = 0;
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_IPATHEVALUATOR_H_ */
