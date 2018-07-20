/*
 * SingleEvaluator.cpp
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#include <SystemConfig.h>
#include <pathplanner/evaluator/Evaluator.h>

namespace msl
{

Evaluator::Evaluator()
    : PathEvaluator()
{
    //# how much the robot values wide corridors compared to short paths
    //# 1,000,000 means that 10m detour is worth avoiding a 10cm wide gap
    //# and 5m detour is worth avoiding a 20cm wide gap
    this->clearSpaceWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "Evaluation", "normal_clearSpaceWeight", NULL);
}

Evaluator::~Evaluator()
{
}

double Evaluator::eval(shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<SearchNode> currentNode,
                       shared_ptr<SearchNode> nextNode, VoronoiNet *voronoi, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path,
                       shared_ptr<geometry::CNPoint2D> lastTarget)
{
    //		//CALCULATE COST FUNCTION AND HEURISTIC FUNCTION
    //
    //		// HEURISTIC FUNCTION: distance to target
    //		double h = path->at(path->size() - 1)->distanceTo(goal);
    //
    //		// COST FUNCTION: width and angle
    //
    //		// cost due to narrowness
    //		double widthc = 0;
    //		//TODO fix bugs
    //		if (path.lastPEdge.MaxRadius > 0)
    //		{
    //			// edges with atleast one real obstacle on one side
    //			widthc = path.lastPEdge.MinDistance - path.lastPEdge.MaxRadius;
    //
    //			// substract robotRadius only, if its not in neighbourhood to robot
    //			//if (!path.lastPEdge.OwnCellEdge(robotQData.ID))
    //			if (!voronoi->isOwnCellEdge(startPos, currentNode, nextNode))
    //			{
    //				widthc -= this->planner->getRobotDiameter() / 2;
    //			}
    //
    //			widthc = std::max(0.0, this->clearSpaceWeight / std::max(1.0, widthc));
    //		}
    //
    //		// cost due to deviation from last path
    //		double lastAnglec = this->planner->getPathDeviationWeight() /* * path.lastPEdge.AngleCost*/;
    //		return (/*costsSoFar // costs so far
    //		+ */voronoi->calcDist(currentNode->getVertex(), nextNode->getVertex()) // cost due to additional length of path
    //				+ widthc // cost due to minimal width of the edge
    //				+ lastAnglec, // cost due to deviation to last path
    //		+h); // heuristic cost due to distToTarget
    return -1;
}

} /* namespace msl */
