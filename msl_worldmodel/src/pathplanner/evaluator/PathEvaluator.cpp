/*
 * PathEvaluator.cpp
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#include <pathplanner/evaluator/PathEvaluator.h>
namespace msl
{

	PathEvaluator::PathEvaluator(PathPlanner* planner)
	{
		this->clearSpaceWeight = 0;
		this->planner = planner;
		this->sc = SystemConfig::getInstance();
	}

	PathEvaluator::~PathEvaluator()
	{
	}

	double PathEvaluator::distance(CNPoint2D first, CNPoint2D second)
	{
		return std::sqrt(std::pow(first.x - second.x, 2) + std::pow(first.y - second.y, 2));
	}

	double PathEvaluator::square(double a)
	{
		return a * a;
	}

	double PathEvaluator::eval(double costsSoFar, CNPoint2D startPos,
									CNPoint2D goal, shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode, shared_ptr<VoronoiNet> voronoi,
									shared_ptr<vector<shared_ptr<CNPoint2D> > > path)
	{

	}

} /* namespace msl */

