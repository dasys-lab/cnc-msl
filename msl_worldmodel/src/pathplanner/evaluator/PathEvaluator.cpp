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
		this->additionalCorridorWidth = (*this->sc)["PathPlanner"]->get<double>("PathPlanner",
																				"additionalCorridorWidth", NULL);
		this->robotDiameter = (*this->sc)["Globals"]->get<double>("Globals", "Dimensions", "DiameterRobot", NULL);
		this->obstacleDistanceWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "obstacleDistanceWeight",
																				NULL);
	}

	PathEvaluator::~PathEvaluator()
	{
	}

	double PathEvaluator::distance(shared_ptr<geometry::CNPoint2D> first, shared_ptr<geometry::CNPoint2D> second)
	{
		return std::sqrt(std::pow(first->x - second->x, 2) + std::pow(first->y - second->y, 2));
	}

	double PathEvaluator::square(double a)
	{
		return a * a;
	}

	double PathEvaluator::eval(double costsSoFar, shared_ptr<geometry::CNPoint2D> startPos,
								shared_ptr<geometry::CNPoint2D> goal, shared_ptr<SearchNode> currentNode,
								shared_ptr<SearchNode> nextNode, VoronoiNet* voronoi,
								shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > path)
	{
		double ret = distance(currentNode->getVertex(), nextNode->getVertex());
		if (voronoi != nullptr)
		{
			pair<shared_ptr<geometry::CNPoint2D>, shared_ptr<geometry::CNPoint2D>> obs =
					voronoi->getSitesNextToHalfEdge(currentNode->getVertex(), nextNode->getVertex());
			if (obs.first != nullptr && obs.second != nullptr)
			{
				double dist = distance(obs.first, obs.second);
				if (dist < robotDiameter + 2 * additionalCorridorWidth)
				{
					ret += this->additionalCorridorWidth;
				}

			}
		}
		return ret;
	}

} /* namespace msl */

