/*
 * PathEvaluator.cpp
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#include <pathplanner/evaluator/PathEvaluator.h>
#include "GeometryCalculator.h"
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
		this->pathAngleWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "pathAngleWeight",
		NULL);
		this->pathLengthWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "pathLengthWeight",
		NULL);
		this->pathDeviationWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "pathDeviationWeight",
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
		double ret = pathLengthWeight * distance(currentNode->getVertex(), nextNode->getVertex());
		cout << "path weight: " << ret << endl;
		auto p = planner->getLastPath();
		if (currentNode->getPredecessor() == nullptr && p != nullptr && p->size() > 1)
		{
			double a = startPos->x - currentNode->getVertex()->x;
			double b = startPos->y - currentNode->getVertex()->y;
			double c = p->at(0)->x - p->at(1)->x;
			double d = p->at(0)->y - p->at(1)->y;

			double mag_v1 = sqrt(a * a + b * b);
			double mag_v2 = sqrt(c * c + d * d);

			double cos_angle = (a * c + b * d) / (mag_v1 * mag_v2);
			double theta = acos(cos_angle);

			if (theta > M_PI / 2)
			{
				theta = M_PI - theta;
			}
			if (theta != NAN)
			{
				ret += pathDeviationWeight * theta;
			}
		}
		cout << "after path deviation: " << ret << endl;
		if (voronoi != nullptr)
		{
			pair<shared_ptr<geometry::CNPoint2D>, shared_ptr<geometry::CNPoint2D>> obs =
					voronoi->getSitesNextToHalfEdge(currentNode->getVertex(), nextNode->getVertex());
			if (obs.first != nullptr && obs.second != nullptr)
			{

				double dist = obstacleDistanceWeight
						* ((1.0
								/ geometry::GeometryCalculator::distancePointToLineSegment(obs.first->x, obs.first->y,
																							currentNode->getVertex(),
																							nextNode->getVertex()))
								+ obstacleDistanceWeight
										* (1.0
												/ geometry::GeometryCalculator::distancePointToLineSegment(
														obs.second->x, obs.second->y, currentNode->getVertex(),
														nextNode->getVertex())));
				ret += dist;
			}
		}
		cout << "after obstacle distance: " << ret << endl;
		if (currentNode->getPredecessor() != nullptr)
		{
			double dx21 = nextNode->getVertex()->x - currentNode->getVertex()->x;
			double dx31 = currentNode->getPredecessor()->getVertex()->x - currentNode->getVertex()->x;
			double dy21 = nextNode->getVertex()->y - currentNode->getVertex()->y;
			double dy31 = currentNode->getPredecessor()->getVertex()->y - currentNode->getVertex()->y;
			double m12 = sqrt(dx21 * dx21 + dy21 * dy21);
			double m13 = sqrt(dx31 * dx31 + dy31 * dy31);
			double theta = acos((dx21 * dx31 + dy21 * dy31) / (m12 * m13));
			if (theta > M_PI / 2)
			{
				theta = M_PI - theta;
			}
			if (theta != NAN)
			{
				ret += pathAngleWeight * theta;
			}
		}
		cout << "final weight: " << ret << endl;
		return ret;
	}

} /* namespace msl */

