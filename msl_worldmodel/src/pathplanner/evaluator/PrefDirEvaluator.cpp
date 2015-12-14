/*
 * SinglePrefDirEvaluator.cpp
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#include <pathplanner/evaluator/PrefDirEvaluator.h>

namespace msl
{
	PrefDirEvaluator::PrefDirEvaluator() :
			PathEvaluator()
	{
		//#cost function parameters for the dribbling case
		//#(apply to all methods that take a preferred direction argument)
		//#clearSpaceWeight same as above
		this->clearSpaceWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "Evaluation",
																			"dribble_clearSpaceWeight", NULL);
		;
	}

	PrefDirEvaluator::~PrefDirEvaluator()
	{
	}

	double PrefDirEvaluator::eval(shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal,
									shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode,
									VoronoiNet* voronoi,
									shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path, shared_ptr<geometry::CNPoint2D> lastTarget)
	{
//		//CALCULATE COST FUNCTION AND HEURISTIC FUNCTION
//
//		// HEURISTIC FUNCTION: distance to target
//		double h = path->at(path->size() - 1)->distanceTo(goal);
//
//		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > teamMatePositions = voronoi->getTeamMatePositions();
//		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > obstaclePositions = voronoi->getObstaclePositions();
//		// COST FUNCTION: width and angle
//
//		// cost due to narrowness
//		//TODO fix bugs
//		double widthc = 0;
//		if (/*path.lastPEdge.MaxRadius > 0 &&*/ path->size() > 1)
//		{
////			// edges with atleast one real obstacle on one side
//			//widthc = path.lastPEdge.MinDistance - path.lastPEdge.MaxRadius;
////
////			// substract robotRadius only, if its not in neighbourhood to robot
//			if (!voronoi->isOwnCellEdge(startPos, currentNode, nextNode))
//			{
//				widthc -= this->planner->getRobotDiameter() / 2;
//			}
////
//			if (widthc < 1)
//		{
//				widthc = 1;
//		}
//			widthc = (this->clearSpaceWeight * this->clearSpaceWeight) / (widthc * widthc);
//		}
//
//		// cost due to deviation from last path
//		double lastAnglec = 0;
////		if (path.pEdges.Count == 1)
//		if(path->size() == 2)
//		{
//			lastAnglec = this->planner->getPathDeviationWeight(); //* path.lastPEdge.AngleCost;
//		}
//		// COST FUNCTION: angle between last and next edge
//
//		// costs for turning from last edge to next edge
////		double deltaangle = std::abs(path.lastFacing - path.nextToLastFacing);
//		double deltaangle = 0;
//		if (deltaangle > M_PI)
//		{ // normalize
//			deltaangle = std::abs(2.0 * M_PI - deltaangle);
//		}
//		double anglec = this->planner->getDribbleRotationWeight()
//				* (0.0, deltaangle * deltaangle - this->planner->getDribbleAngleTolerance());
////
////		// heuristic cost for turning from last edge to goal
//		if (h > 0) // if (not at the goal, yet)
//		{
////			deltaangle = std::abs(path.angleToTarget - path.lastFacing);
//			if (deltaangle > M_PI)
//			{ // normalize
//				deltaangle = std::abs(2.0 * M_PI - deltaangle);
//			}
//			h += this->planner->getDribbleRotationWeight() * std::max(0.0, deltaangle * deltaangle - this->planner->getDribbleAngleTolerance());
//		}
//
//		return /*costsSoFar // cost so far
//		+*/ voronoi->calcDist(currentNode->getVertex(), nextNode->getVertex()) // length of the path
//				+ widthc + anglec + lastAnglec + h;
		return -1;
	}

} /* namespace msl */
