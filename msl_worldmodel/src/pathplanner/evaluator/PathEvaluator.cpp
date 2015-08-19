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
		this->planner = planner;
		this->sc = SystemConfig::getInstance();
		voroniPub = n.advertise<msl_msgs::VoronoiNetInfo>("/PathPlanner/VoronoiNet", 10);
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

	/**
	 * Calculates the cost for a voronoi vertex
	 */
	double PathEvaluator::eval(shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal,
								shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode,
								VoronoiNet* voronoi, shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > path)
	{

		// add the cost of current node to return
		double ret = currentNode->getCost();
		// add weighted distance to return
		ret += pathLengthWeight * currentNode->getVertex()->distanceTo(nextNode->getVertex());
		auto p = planner->getLastPath();
		auto lastGoal = planner->getLastTarget();
		//if we are in the first node, there has been a path before und the goal changed
		if (currentNode->getPredecessor() == nullptr && p != nullptr && p->size() > 1 && lastGoal != nullptr
				&& lastGoal->distanceTo(goal) > 250)
		{
			//claculate agle between the first edge of the current path and the last path
			double a = startPos->x - currentNode->getVertex()->x;
			double b = startPos->y - currentNode->getVertex()->y;
			double c = p->at(0)->x - p->at(1)->x;
			double d = p->at(0)->y - p->at(1)->y;

			double mag_v1 = sqrt(a * a + b * b);
			double mag_v2 = sqrt(c * c + d * d);

			double cos_angle = (a * c + b * d) / (mag_v1 * mag_v2);
			double theta = acos(cos_angle);

			// if the angle is higher then M_PI / 2 use M_PI - angle
			if (theta > M_PI / 2)
			{
				theta = M_PI - theta;
			}
			//if there is a valid angle add weighted angle to return
			if (theta != NAN)
			{
				ret += pathDeviationWeight * theta;
			}
		}
		/*
		 * can be null so check it
		 */
		if (voronoi != nullptr)
		{
			//get sites next to voronoi edge
			pair<shared_ptr<geometry::CNPoint2D>, shared_ptr<geometry::CNPoint2D>> obs =
					voronoi->getSitesNextToHalfEdge(currentNode->getVertex(), nextNode->getVertex());

			if (obs.first != nullptr)
			{
				// calculate distance to one obstacle, you dont need to second one because dist is euqal by voronoi definition
				double distobs = geometry::GeometryCalculator::distancePointToLineSegment(obs.first->x, obs.first->y,
																							currentNode->getVertex(),
																							nextNode->getVertex());
				//calcualte weighted dist to both objects
				ret += (obstacleDistanceWeight * (1.0 / distobs)) * 2;

				//if the distance to the obstacles is too small return -1 to not expand this node
				if ((distobs * 2)
						< (this->planner->getRobotDiameter() * 2 + this->planner->getAdditionalCorridorWidth()))
				{
					return -1.0;
				}
			}
		}
		//if the path is longer then one vertex add cost for the angle
		if (currentNode->getPredecessor() != nullptr)
		{
			double dx21 = nextNode->getVertex()->x - currentNode->getVertex()->x;
			double dx31 = currentNode->getPredecessor()->getVertex()->x - currentNode->getVertex()->x;
			double dy21 = nextNode->getVertex()->y - currentNode->getVertex()->y;
			double dy31 = currentNode->getPredecessor()->getVertex()->y - currentNode->getVertex()->y;
			double m12 = sqrt(dx21 * dx21 + dy21 * dy21);
			double m13 = sqrt(dx31 * dx31 + dy31 * dy31);
			double theta = acos((dx21 * dx31 + dy21 * dy31) / (m12 * m13));
			// if the angle is higher then M_PI / 2 use M_PI - angle

			if (theta > M_PI / 2)
			{
				theta = M_PI - theta;
			}
			//if there is a valid angle add weighted angle to return
			if (theta != NAN)
			{
				ret += pathAngleWeight * theta;
			}
		}
		return ret;
	}

} /* namespace msl */

