/*
 * PathPlanner.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#include <pathplanner/PathPlanner.h>
#include "MSLWorldModel.h"
#include "GeometryCalculator.h"

namespace msl
{

	PathPlanner::PathPlanner(MSLWorldModel* wm, int count) :
			voronoiDiagrams(count)
	{
		this->wm = wm;
		this->artificialObjectNet = make_shared<VoronoiNet>(wm);
		sc = supplementary::SystemConfig::getInstance();
		for (int i = 0; i < count; i++)
		{
			this->voronoiDiagrams.at(i) = make_shared<VoronoiNet>(wm);
			this->voronoiDiagrams.at(i)->setVoronoi(
					make_shared<VoronoiDiagram>(
							(DelaunayTriangulation)this->artificialObjectNet->getVoronoi()->dual()));
		}
		this->robotDiameter = (*this->sc)["Globals"]->get<double>("Globals", "Dimensions", "DiameterRobot", NULL);
		this->pathDeviationWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "pathDeviationWeight", NULL);
		this->currentVoronoiPos = -1;
		this->corridorWidthDivisor = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "corridorWidthDivisor",
		NULL);
		this->pathPlannerDebug = (*this->sc)["PathPlanner"]->get<bool>("PathPlanner", "pathPlannerDebug",
		NULL);
		this->additionalCorridorWidth = (*this->sc)["PathPlanner"]->get<double>("PathPlanner",
																				"additionalCorridorWidth", NULL);
		this->snapDistance = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "snapDistance", NULL);
		lastPath = nullptr;
		corridorPub = n.advertise<msl_msgs::CorridorCheck>("/PathPlanner/CorridorCheck", 10);
		initializeArtificialObstacles();

	}

	PathPlanner::~PathPlanner()
	{
	}

	/**
	 * processes the WorldModel msg
	 * @param msg msl_sensor_msgs::WorldModelDataPtr
	 */
	void PathPlanner::processWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg)
	{
		lock_guard<mutex> lock(voronoiMutex);
		vector<geometry::CNPoint2D> points;
		auto ownPos = wm->rawSensorData.getOwnPositionVision();
		for (int i = 0; i < msg->obstacles.size(); i++)
		{
			points.push_back(*(geometry::CNPoint2D(msg->obstacles.at(i).x, msg->obstacles.at(i).y).egoToAllo(*ownPos)));
		}

		voronoiDiagrams.at((currentVoronoiPos + 1) % 10)->generateVoronoiDiagram(points);
		currentVoronoiPos++;

	}

	void PathPlanner::initializeArtificialObstacles()
	{
		MSLFootballField* field = MSLFootballField::getInstance();
		vector<VoronoiDiagram::Site_2> toInsert;
		int baseSize = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "artificialObjectBaseSize", NULL);
		if (field->FieldLength / baseSize > 20 || field->FieldWidth / baseSize > 20)
		{
			baseSize = (int)max(field->FieldLength / 20, field->FieldWidth / 20);
		}
		int lengthInterval = (int)(baseSize
				+ ((int)(field->FieldLength + 2000) % baseSize) / (int)((int)(field->FieldLength + 2000) / baseSize));
		int widthInterval = (int)(baseSize
				+ ((int)(field->FieldWidth + 2000) % baseSize) / (int)((int)(field->FieldWidth + 2000) / baseSize));
		int halfFieldLength = (int)field->FieldLength / 2 + 1000;
		int halfFieldWidth = (int)field->FieldWidth / 2 + 1000;

		//up right
		toInsert.push_back(VoronoiDiagram::Site_2(halfFieldLength, -halfFieldWidth));
		//down right
		toInsert.push_back(VoronoiDiagram::Site_2(-halfFieldLength, -halfFieldWidth));
		// down left
		toInsert.push_back(VoronoiDiagram::Site_2(-halfFieldLength, halfFieldWidth));
		// up left
		toInsert.push_back(VoronoiDiagram::Site_2(halfFieldLength, halfFieldWidth));

		int x = 0;
		int y = halfFieldWidth;
		for (x = -halfFieldLength + lengthInterval; x <= halfFieldLength - lengthInterval; x += lengthInterval)
		{ // side sides
			toInsert.push_back(VoronoiDiagram::Site_2(x, y));
			toInsert.push_back(VoronoiDiagram::Site_2(x, -y));
		}

		x = halfFieldLength;
		for (y = -halfFieldWidth + widthInterval; y <= halfFieldWidth - widthInterval; y += widthInterval)
		{ // goal sides
			toInsert.push_back(VoronoiDiagram::Site_2(x, y));
			toInsert.push_back(VoronoiDiagram::Site_2(-x, y));
		}
		this->artificialObjectNet->getVoronoi()->insert(toInsert.begin(), toInsert.end());
	}

	/**
	 * aStar search on a VoronoiDiagram
	 * @param voronoi shared_ptr<VoronoiNet>
	 * @param ownPos Point_2
	 * @param goal Point_2
	 * @return shared_ptr<vector<shared_ptr<Point_2>>>
	 */
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> PathPlanner::plan(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<PathEvaluator> eval)
	{
		//TODO think about it if it stays comments and docu
		if((goal - startPos)->length() < this->snapDistance)
		{
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
			ret->push_back(goal);
			lastPath = ret;
			return ret;
		}
		bool reachable = true;
		auto sites = voronoi->getSitePositions();
		for(int i = 0; i < sites->size(); i++)
		{
			if(corridorCheck(voronoi, startPos, goal, sites->at(i).first))
			{
				reachable = false;
				break;
			}
		}
		if(reachable)
		{
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
			ret->push_back(goal);
			lastPath = ret;
			return ret;
		}
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = aStarSearch(voronoi, startPos, goal, eval);
		if(ret != nullptr)
		{
			lastPath = ret;
			return ret;
		}
		// return nullptr if there is no way to goal
		lastPath = nullptr;
		return nullptr;
	}

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > msl::PathPlanner::aStarSearch(
			shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> startPos,
			shared_ptr<geometry::CNPoint2D> goal, shared_ptr<PathEvaluator> eval)
	{
		// return
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		// vector with open searchnodes
		shared_ptr<vector<shared_ptr<SearchNode>>> open = make_shared<vector<shared_ptr<SearchNode>>>();
		//vector with closed search nodes
		shared_ptr<vector<shared_ptr<SearchNode>>> closed = make_shared<vector<shared_ptr<SearchNode>>>();

		//get closest Vertices to ownPos => start point for a star serach
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> closestVerticesToOwnPos = voronoi->getVerticesOfFace(startPos);

		// get closest Vertex to goal => goal for a star serach
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> closestVerticesToGoal = voronoi->getVerticesOfFace(goal);

		// a star serach

		for(int i = 0; i < closestVerticesToOwnPos->size(); i++)
		{
			insert(open, make_shared<SearchNode>(SearchNode(closestVerticesToOwnPos->at(i),
									voronoi->calcDist(closestVerticesToOwnPos->at(i), goal), nullptr)));
		}

		while(open->size() != 0)
		{
			shared_ptr<SearchNode> currentNode = open->at(0);
			if(checkGoalReachable(voronoi, currentNode, closestVerticesToGoal, goal))
			{
				shared_ptr<SearchNode> temp = currentNode;
				ret->push_back(make_shared<geometry::CNPoint2D>(currentNode->getVertex()->x, currentNode->getVertex()->y));
				while(temp->getPredecessor() != nullptr)
				{
					ret->push_back(make_shared<geometry::CNPoint2D>(temp->getPredecessor()->getVertex()->x, temp->getPredecessor()->getVertex()->y));
					temp = temp->getPredecessor();
				}
				reverse(ret->begin(), ret->end());
				ret->push_back(goal);
				lastPath = ret;
				return ret;
			}
			closed->push_back(currentNode);
			open->erase(open->begin());
			voronoi->expandNode(currentNode, open, closed, startPos, goal, eval);
		}
		lastPath = nullptr;
		return nullptr;
	}

	void msl::PathPlanner::insert(shared_ptr<vector<shared_ptr<SearchNode> > > vect, shared_ptr<SearchNode> currentNode)
	{
		vector<shared_ptr<SearchNode> >::iterator it = std::upper_bound(vect->begin(), vect->end(), currentNode,
																		PathPlanner::compare); // find proper position in descending order
		vect->insert(it, currentNode); // insert before iterator it
	}

	bool msl::PathPlanner::compare(shared_ptr<SearchNode> first, shared_ptr<SearchNode> second)
	{
		if (first->getCost() < second->getCost())
		{
			return true;
		}
		return false;
	}

	bool PathPlanner::checkGoalVerticesReached(
			const shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > >& closestVerticesToGoal,
			const shared_ptr<SearchNode>& currentNode, bool found)
	{
		for (int i = 0; i < closestVerticesToGoal->size(); i++)
		{
			if (currentNode->getVertex()->x == closestVerticesToGoal->at(i)->x
					&& currentNode->getVertex()->y == closestVerticesToGoal->at(i)->y)
			{
				found = true;
				break;
			}
		}
		return found;
	}

	bool msl::PathPlanner::corridorCheck(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> currentPos,
											shared_ptr<geometry::CNPoint2D> goal,
											shared_ptr<geometry::CNPoint2D> obstaclePoint)
	{
		double length = voronoi->calcDist(currentPos, goal);
		double dx = currentPos->x - goal->x;
		double dy = currentPos->y - goal->y;
		double dist = std::sqrt(dx * dx + dy * dy);
		dx /= dist;
		dy /= dist;
		shared_ptr<geometry::CNPoint2D> p1 = make_shared<geometry::CNPoint2D>(
				currentPos->x + (this->robotDiameter / 2 + this->additionalCorridorWidth) * dy,
				currentPos->y - (this->robotDiameter / 2 + this->additionalCorridorWidth) * dx);
		shared_ptr<geometry::CNPoint2D> p2 = make_shared<geometry::CNPoint2D>(
				currentPos->x - (this->robotDiameter / 2 + this->additionalCorridorWidth) * dy,
				currentPos->y + (this->robotDiameter / 2 + this->additionalCorridorWidth) * dx);
		shared_ptr<geometry::CNPoint2D> p3 = make_shared<geometry::CNPoint2D>(
				goal->x
						+ std::max(this->robotDiameter / 2 + this->additionalCorridorWidth,
									length / this->corridorWidthDivisor + this->additionalCorridorWidth) * dy,
				goal->y
						- std::max(this->robotDiameter / 2 + this->additionalCorridorWidth,
									length / this->corridorWidthDivisor + this->additionalCorridorWidth) * dx);
		shared_ptr<geometry::CNPoint2D> p4 = make_shared<geometry::CNPoint2D>(
				goal->x
						- std::max(this->robotDiameter / 2 + this->additionalCorridorWidth,
									length / this->corridorWidthDivisor + this->additionalCorridorWidth) * dy,
				goal->y
						+ std::max(this->robotDiameter / 2 + this->additionalCorridorWidth,
									length / this->corridorWidthDivisor + this->additionalCorridorWidth) * dx);
		vector<shared_ptr<geometry::CNPoint2D>> points;
		points.push_back(p1);
		points.push_back(p3);
		points.push_back(p4);
		points.push_back(p2);
		if (pathPlannerDebug)
		{
			sendCorridorCheck(points);
		}
		return obstaclePoint != nullptr
				&& geometry::GeometryCalculator::isInsidePolygon(points, points.size(), obstaclePoint);
	}

	void msl::PathPlanner::sendCorridorCheck(vector<shared_ptr<geometry::CNPoint2D> > points)
	{
		msl_msgs::CorridorCheck cc;
		for (int i = 0; i < points.size(); i++)
		{
			msl_msgs::Point2dInfo info;
			info.x = points.at(i)->x;
			info.y = points.at(i)->y;
			cc.corridorPoints.push_back(info);
		}
		corridorPub.publish(cc);
	}

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> msl::PathPlanner::getLastPath()
	{
		return lastPath;
	}

	bool PathPlanner::checkGoalReachable(shared_ptr<VoronoiNet> voronoi, shared_ptr<SearchNode> currentNode,
											shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> closestVerticesToGoal, shared_ptr<geometry::CNPoint2D> goal)
	{
		bool found = false;
		found = checkGoalVerticesReached(closestVerticesToGoal, currentNode, found);
		if(found)
		{
			shared_ptr<VoronoiDiagram::Point_2> obstacle = voronoi->getSiteOfFace(Point_2(goal->x, goal->y));
			shared_ptr<geometry::CNPoint2D> obstaclePoint = make_shared<geometry::CNPoint2D>(obstacle->x(), obstacle->y());
			return !corridorCheck(voronoi, currentNode->getVertex(), goal, obstaclePoint);
		}
		return false;
	}

	shared_ptr<VoronoiNet> PathPlanner::getArtificialObjectNet()
	{
		return artificialObjectNet;
	}

	/**
	 * gets all saved VoronoiNets
	 * @return vector<shared_ptr<VoronoiNet>>
	 */
	vector<shared_ptr<VoronoiNet> > PathPlanner::getVoronoiNets()
	{
		lock_guard<mutex> lock(this->voronoiMutex);
		return this->voronoiDiagrams;
	}

	/**
	 * gets latest accesable VoronoiNet
	 * @return shared_ptr<VoronoiNet>
	 */
	shared_ptr<VoronoiNet> PathPlanner::getCurrentVoronoiNet()
	{
		lock_guard<mutex> lock(this->voronoiMutex);
		if (currentVoronoiPos == -1)
		{
			return nullptr;
		}
		shared_ptr<VoronoiNet> ret = make_shared<VoronoiNet>(voronoiDiagrams.at(currentVoronoiPos % 10));
		return ret;
	}

	double PathPlanner::getRobotDiameter()
	{
		return robotDiameter;
	}

	double PathPlanner::getPathDeviationWeight()
	{
		return pathDeviationWeight;
	}

	double PathPlanner::getDribbleAngleTolerance()
	{
		return dribble_angleTolerance;
	}

	double PathPlanner::getDribbleRotationWeight()
	{
		return dribble_rotationWeight;
	}

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > PathPlanner::getArtificialObstacles()
	{
		MSLFootballField* field = MSLFootballField::getInstance();
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > toInsert = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		int baseSize = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "artificialObjectBaseSize", NULL);
		if (field->FieldLength / baseSize > 20 || field->FieldWidth / baseSize > 20)
		{
			baseSize = (int)max(field->FieldLength / 20, field->FieldWidth / 20);
		}
		int lengthInterval = (int)(baseSize
				+ ((int)(field->FieldLength + 2000) % baseSize) / (int)((int)(field->FieldLength + 2000) / baseSize));
		int widthInterval = (int)(baseSize
				+ ((int)(field->FieldWidth + 2000) % baseSize) / (int)((int)(field->FieldWidth + 2000) / baseSize));
		int halfFieldLength = (int)field->FieldLength / 2 + 1000;
		int halfFieldWidth = (int)field->FieldWidth / 2 + 1000;

		//up right
		toInsert->push_back(make_shared<geometry::CNPoint2D>(halfFieldLength, -halfFieldWidth));
		//down right
		toInsert->push_back(make_shared<geometry::CNPoint2D>(-halfFieldLength, -halfFieldWidth));
		// down left
		toInsert->push_back(make_shared<geometry::CNPoint2D>(-halfFieldLength, halfFieldWidth));
		// up left
		toInsert->push_back(make_shared<geometry::CNPoint2D>(halfFieldLength, halfFieldWidth));

		int x = 0;
		int y = halfFieldWidth;
		for (x = -halfFieldLength + lengthInterval; x <= halfFieldLength - lengthInterval; x += lengthInterval)
		{ // side sides
			toInsert->push_back(make_shared<geometry::CNPoint2D>(x, y));
			toInsert->push_back(make_shared<geometry::CNPoint2D>(x, -y));
		}

		x = halfFieldLength;
		for (y = -halfFieldWidth + widthInterval; y <= halfFieldWidth - widthInterval; y += widthInterval)
		{ // goal sides
			toInsert->push_back(make_shared<geometry::CNPoint2D>(x, y));
			toInsert->push_back(make_shared<geometry::CNPoint2D>(-x, y));
		}
		return toInsert;
	}
} /* namespace alica */

