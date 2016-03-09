/*
 * PathEvaluator.cpp
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#include <pathplanner/evaluator/PathEvaluator.h>
#include "GeometryCalculator.h"
#include "msl_msgs/VoronoiNetInfo.h"
#include "MSLEnums.h"
namespace msl
{

	PathEvaluator::PathEvaluator()
	{
		this->sc = SystemConfig::getInstance();
		voronoiPub = n.advertise<msl_msgs::VoronoiNetInfo>("/PathPlanner/VoronoiNet", 10);
		this->additionalCorridorWidth = (*this->sc)["PathPlanner"]->get<double>("PathPlanner",
																				"additionalCorridorWidth", NULL);
		this->robotDiameter = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL) * 2;
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
								VoronoiNet* voronoi, shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > lastPath,
								shared_ptr<geometry::CNPoint2D> lastTarget)
	{

		// add the cost of current node to return
		double ret = currentNode->getCost();
		// add weighted distance to return
		ret += pathLengthWeight * distanceTo(nextNode->getVertex(), currentNode->getVertex());
		//if we are in the first node, there has been a path before und the goal changed
		if (currentNode->getPredecessor() == nullptr && lastPath != nullptr && lastPath->size() > 1
				&& lastTarget != nullptr && lastTarget->distanceTo(goal) > 250)
		{
			//claculate agle between the first edge of the current path and the last path
			double a = startPos->x - currentNode->getVertex()->point().x();
			double b = startPos->y - currentNode->getVertex()->point().y();
			double c = lastPath->at(0)->x - lastPath->at(1)->x;
			double d = lastPath->at(0)->y - lastPath->at(1)->y;

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
			pair<pair<shared_ptr<geometry::CNPoint2D>, int>, pair<shared_ptr<geometry::CNPoint2D>, int>> obs =
					voronoi->getSitesNextToHalfEdge(currentNode->getVertex(), nextNode->getVertex());

			if (obs.first.first != nullptr && obs.second.first != nullptr)
			{
				// Both are artificial sites, so dont expand
				if (obs.first.second == EntityType::ArtificialObstacle
						&& obs.second.second == EntityType::ArtificialObstacle && MSLFootballField::isInsideField(startPos, MSLFootballField::Surrounding))
				{
					return -1.0;
				}
				// calculate distance to one obstacle, you dont need to second one because dist is euqal by voronoi definition
				double distobs = geometry::GeometryCalculator::distancePointToLineSegment(
						obs.first.first->x,
						obs.first.first->y,
						make_shared<geometry::CNPoint2D>(currentNode->getVertex()->point().x(),
															currentNode->getVertex()->point().y()),
						make_shared<geometry::CNPoint2D>(nextNode->getVertex()->point().x(),
															nextNode->getVertex()->point().y()));
				//calculate weighted dist to both objects

				//Both sites are teammates (relax costs) || Teammate & artificial (ignorable & not ignorable) (relax costs)
				if ((obs.first.second > 0 && obs.second.second > 0)
						|| (obs.first.second > 0 && obs.second.second != EntityType::ArtificialObstacle) // TODO war aus irgend einem grund -1
						|| (obs.second.second > 0 && obs.first.second != EntityType::ArtificialObstacle))
				{
					ret += (obstacleDistanceWeight * (1.0 / distobs));
				}
				//One of both sites is an opp (classic) || Both are opponents (classic) || Opp & artificial (classic)
				if (obs.first.second == -1 || obs.second.second == -1)
				{
					ret += (obstacleDistanceWeight * (1.0 / distobs)) * 2;
				}

				//if the distance to the obstacles is too small return -1 to not expand this node
				if ((distobs * 2) < (robotDiameter * 2 + additionalCorridorWidth) && MSLFootballField::isInsideField(startPos))
				{
					return -1.0;
				}
			}
		}
		//if the path is longer then one vertex add cost for the angle
		if (currentNode->getPredecessor() != nullptr)
		{
			double dx21 = nextNode->getVertex()->point().x() - currentNode->getVertex()->point().x();
			double dx31 = currentNode->getPredecessor()->getVertex()->point().x()
					- currentNode->getVertex()->point().x();
			double dy21 = nextNode->getVertex()->point().y() - currentNode->getVertex()->point().y();
			double dy31 = currentNode->getPredecessor()->getVertex()->point().y()
					- currentNode->getVertex()->point().y();
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

	double PathEvaluator::distanceTo(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2)
	{
		return sqrt(pow(v2->point().x() - v1->point().x(), 2) + pow(v2->point().y() - v1->point().y(), 2));
	}

} /* namespace msl */

