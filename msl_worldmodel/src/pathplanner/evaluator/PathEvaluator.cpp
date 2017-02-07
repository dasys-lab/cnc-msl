/*
 * PathEvaluator.cpp
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#include "GeometryCalculator.h"
#include "MSLEnums.h"
#include "msl_msgs/VoronoiNetInfo.h"
#include "pathplanner/SearchNode.h"
#include "pathplanner/VoronoiNet.h"
#include <SystemConfig.h>
#include <pathplanner/evaluator/PathEvaluator.h>
namespace msl
{

PathEvaluator::PathEvaluator()
{
    this->sc = supplementary::SystemConfig::getInstance();
    voronoiPub = n.advertise<msl_msgs::VoronoiNetInfo>("/PathPlanner/VoronoiNet", 10);
    this->additionalCorridorWidth = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "additionalCorridorWidth", NULL);
    this->robotDiameter = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL) * 2;
    this->obstacleDistanceWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "obstacleDistanceWeight", NULL);
    this->pathAngleWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "pathAngleWeight", NULL);
    this->pathLengthWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "pathLengthWeight", NULL);
    this->pathDeviationWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "pathDeviationWeight", NULL);
}

PathEvaluator::~PathEvaluator()
{
}

pair<double, double> PathEvaluator::evalInitial(shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<SearchNode> nextNode,
                                                VoronoiNet *voronoi, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> lastPath,
                                                shared_ptr<geometry::CNPoint2D> lastTarget)
{
    auto curPoint = nextNode->getPoint();

    double cost = pathLengthWeight * startPos->distanceTo(curPoint);
    double heuristic = pathLengthWeight * curPoint->distanceTo(goal);

    // if we are in the first node, there has been a path before and the goal changed
    if (lastPath != nullptr && lastPath->size() > 1 && lastTarget != nullptr && lastTarget->distanceTo(goal) < 250)
    {
        // calculate angle between the first edge of the current path and the last path
        double a = startPos->x - curPoint->x;
        double b = startPos->y - curPoint->y;
        double c = lastPath->at(0)->x - lastPath->at(1)->x;
        double d = lastPath->at(0)->y - lastPath->at(1)->y;

        double mag_v1 = sqrt(a * a + b * b);
        double mag_v2 = sqrt(c * c + d * d);

        double cos_angle = (a * c + b * d) / (mag_v1 * mag_v2);
        double theta = acos(cos_angle);

        if (theta != NAN)
        {
            cost += pathDeviationWeight * theta;
        }
    }
    return pair<double, double>(cost, heuristic);
}

/**
 * Calculates the cost for a voronoi vertex
 */
pair<double, double> PathEvaluator::eval(shared_ptr<geometry::CNPoint2D> goal, shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode,
                                         VoronoiNet *voronoi)
{
    auto curPoint = currentNode->getPoint();
    auto nextPoint = nextNode->getPoint();

    double cost = currentNode->getCost();
    double heuristic = pathLengthWeight * nextPoint->distanceTo(goal);

    cost += pathLengthWeight * nextPoint->distanceTo(curPoint);

    if (voronoi != nullptr)
    {
        // get sites next to voronoi edge
        int upType = voronoi->getTypeOfSite(nextNode->getEdge()->up()->point());
        int downType = voronoi->getTypeOfSite(nextNode->getEdge()->down()->point());

        if (upType == EntityType::UndefinedEntity || downType == EntityType::UndefinedEntity)
        {
            return pair<double, double>(-1.0, -1.0);
        }

        // calculate distance to one obstacle, you dont need to second one because dist is euqal by voronoi definition
        double distobs = geometry::distancePointToLineSegment(nextNode->getEdge()->up()->point().x(), nextNode->getEdge()->up()->point().y(), curPoint->x,
                                                              curPoint->y, nextPoint->x, nextPoint->y);
        // calculate weighted dist to both objects

        // Both sites are teammates (relax costs) || Teammate & artificial (ignorable & not ignorable) (relax costs)
        if ((upType > 0 && downType > 0) || (upType > 0 && downType != EntityType::ArtificialObstacle) // TODO war aus irgend einem grund -1
            || (downType > 0 && upType != EntityType::ArtificialObstacle))
        {
            cost += (obstacleDistanceWeight * (1.0 / distobs));
        }
        // One of both sites is an opp (classic) || Both are opponents (classic) || Opp & artificial (classic)
        if (upType == -1 || downType == -1)
        {
            cost += (obstacleDistanceWeight * (1.0 / distobs)) * 2;
        }
    }

    // if the path is longer then one vertex add cost for the angle
    if (currentNode->getPredecessor() != nullptr)
    {
        double dx21 = nextPoint->x - curPoint->x;
        double dx31 = currentNode->getPredecessor()->getPoint()->x - curPoint->x;
        double dy21 = nextPoint->y - curPoint->y;
        double dy31 = currentNode->getPredecessor()->getPoint()->y - curPoint->y;
        double m12 = sqrt(dx21 * dx21 + dy21 * dy21);
        double m13 = sqrt(dx31 * dx31 + dy31 * dy31);
        double theta = acos((dx21 * dx31 + dy21 * dy31) / (m12 * m13));

        // if there is a valid angle add weighted angle to return
        if (theta != NAN)
        {
            cost += pathAngleWeight * theta;
        }
    }
    return pair<double, double>(cost, heuristic);
}

//	double PathEvaluator::distanceTo(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2)
//	{
//		return sqrt(pow(v2->point().x() - v1->point().x(), 2) + pow(v2->point().y() - v1->point().y(), 2));
//	}

} /* namespace msl */
