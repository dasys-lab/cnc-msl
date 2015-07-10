/*
 * PathEvaluator.h
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_PATHEVALUATOR_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_PATHEVALUATOR_H_

#include <vector>
#include <memory>
#include "container/CNPoint2D.h"
#include "pathplanner/PathPlanner.h"
#include "pathplanner/VoronoiNet.h"
#include <SystemConfig.h>
namespace msl
{
	class PathPlanner;
	class VoronoiNet;
	class PathEvaluator
	{
	public:
		PathEvaluator(PathPlanner* planner);
		virtual ~PathEvaluator();
		virtual double eval(double costsSoFar, shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal,
							shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode,
							VoronoiNet* voronoi = nullptr,
							shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path= nullptr);

	protected:
		double clearSpaceWeight;
		double robotDiameter;
		double additionalCorridorWidth;
		double obstacleDistanceWeight;
		PathPlanner* planner;
		static double distance(shared_ptr<geometry::CNPoint2D> first, shared_ptr<geometry::CNPoint2D> second);
		static double square(double a);
		supplementary::SystemConfig* sc;

	};

}/* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_PATHEVALUATOR_H_ */
