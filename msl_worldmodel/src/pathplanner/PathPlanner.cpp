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
		this->lastClosestNode = nullptr;
		this->lastClosestPointToBlock = nullptr;
		this->lastTarget == nullptr;
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
		this->corridorWidthDivisorBall = (*this->sc)["PathPlanner"]->get<double>("PathPlanner",
																					"corridorWidthDivisorBall",
																					NULL);
		this->pathPlannerDebug = (*this->sc)["PathPlanner"]->get<bool>("PathPlanner", "pathPlannerDebug",
		NULL);
		this->additionalCorridorWidth = (*this->sc)["PathPlanner"]->get<double>("PathPlanner",
																				"additionalCorridorWidth", NULL);
		this->additionalBallCorridorWidth = (*this->sc)["PathPlanner"]->get<double>("PathPlanner",
																					"additionalBallCorridorWidth",
																					NULL);
		this->snapDistance = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "snapDistance", NULL);
		this->marginToBlockedArea = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "marginToBlockedArea", NULL);
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
		vector<shared_ptr<geometry::CNPoint2D>> points;
		auto ownPos = wm->rawSensorData.getOwnPositionVision();
		for (int i = 0; i < msg->obstacles.size(); i++)
		{
			points.push_back(
					make_shared<geometry::CNPoint2D>(msg->obstacles.at(i).x, msg->obstacles.at(i).y)->egoToAllo(
							*ownPos));
		}

		voronoiDiagrams.at((currentVoronoiPos + 1) % 10)->generateVoronoiDiagram(points);
		currentVoronoiPos++;

	}

	/**
	 * initializes artificial objects and insert them into artificial object net
	 */
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
		//if there was no target before save if
		// else if there was a traget before it has to be difffenrent to be saved
		if(lastTarget == nullptr)
		{
			lastTarget = goal;
		}
		else
		{
			if(lastTarget->distanceTo(goal) > 50)
			{
				lastTarget = goal;
				lastClosestPointToBlock = nullptr;
			}
		}
		//TODO think about it if it stays comments and docu
//		if((goal - startPos)->length() < this->snapDistance)
//		{
//			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
//			ret->push_back(goal);
//			lastPath = ret;
//			return ret;
//		}
		//check if the goal is reachable directly by checking a corridor between the robot and the goal
		bool reachable = true;
		auto sites = voronoi->getSitePositions();

		for(int i = 0; i < sites->size(); i++)
		{
			//if there is an obstacle inside the corridor the goal is not reachable
			if(corridorCheck(voronoi, startPos, goal, sites->at(i).first))
			{
				reachable = false;
				break;
			}
		}
		//if the corridor is free the robot can drive towards the goal
		if(reachable)
		{
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
			ret->push_back(goal);
			lastPath = ret;
			return ret;
		}
		//there is no direct way so wo have to search a way to the goal
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = aStarSearch(voronoi, startPos, goal, eval);
		//if there is a path save it and return it
		if(ret != nullptr)
		{
			lastPath = ret;
			return ret;
		}
		else
		{
			//if there is no path to the goalpoint
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
			//if there is a closest node to the goal the robot can reach
			if(lastClosestNode != nullptr)
			{
				//get the way to reach this node
				shared_ptr<SearchNode> temp = lastClosestNode;
				ret->push_back(make_shared<geometry::CNPoint2D>(lastClosestNode->getVertex()->x, lastClosestNode->getVertex()->y));
				while(temp->getPredecessor() != nullptr)
				{
					ret->push_back(make_shared<geometry::CNPoint2D>(temp->getPredecessor()->getVertex()->x, temp->getPredecessor()->getVertex()->y));
					temp = temp->getPredecessor();
				}
				reverse(ret->begin(), ret->end());
				//if there is only one vertex in the path we have reach the blocked area save this point
				if(ret->size() == 1 && lastClosestPointToBlock == nullptr)
				{
					lastClosestPointToBlock = ret->at(0);
				}
				//if there is a saved point close to the blocked area and it is closer than the margin to this area the closest point is returned
				if(lastClosestPointToBlock != nullptr && ret->size() == 1 && (*ret->begin())->distanceTo(lastClosestPointToBlock) < this->marginToBlockedArea)
				{
					ret->clear();
					ret->push_back(lastClosestPointToBlock);
					lastPath = ret;
					lastClosestNode =nullptr;
					return ret;
				}
				//if the margin is not reached the closest point is set to the next one
				if(lastClosestPointToBlock != nullptr && ret->size() == 1 && (*ret->begin())->distanceTo(lastClosestPointToBlock) > this->marginToBlockedArea)
				{
					lastClosestPointToBlock = ret->at(0);
				}
				lastPath = ret;
				lastClosestNode = nullptr;
				return ret;
			}
			else
			{
				//TODO test
				//if there is no closestVertex to the blocked area there was no expanded node during a star search
				auto ownPos = this->wm->rawSensorData.getOwnPositionVision();
				auto ownPoint = make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y);
				auto vertices = voronoi->getVerticesOfFace(ownPoint);
				shared_ptr<geometry::CNPoint2D> bestVertex = nullptr;
				//find vertex of robots voronoi fac and which has maximum distance
				for(int i = 0; i < vertices->size(); i++)
				{
					if(bestVertex == nullptr)
					{
						bestVertex = vertices->at(i);
					}
					else
					{
						if(vertices->at(i)->distanceTo(ownPoint) > bestVertex->distanceTo(ownPoint))
						{
							bestVertex = vertices->at(i);
						}
					}
				}
				// drive to this direction because it should be the freest
				shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
				double dist = ownPoint->distanceTo(bestVertex);
				shared_ptr<geometry::CNPoint2D> retPoint = (bestVertex - ownPoint)->normalize() * (dist / 2);
				ret->push_back(retPoint);
				lastClosestNode = nullptr;
				lastPath = ret;
				return ret;
			}
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

// get closest Vertices to goal => goal for a star serach
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> closestVerticesToGoal = voronoi->getVerticesOfFace(goal);

// a star serach

//insert all vertices of startpos face
		for(int i = 0; i < closestVerticesToOwnPos->size(); i++)
		{
			insert(open, make_shared<SearchNode>(SearchNode(closestVerticesToOwnPos->at(i),
									voronoi->calcDist(closestVerticesToOwnPos->at(i), goal), nullptr)));
		}
// while there is still a node to expand
		while(open->size() != 0)
		{
			//get first node in open
			shared_ptr<SearchNode> currentNode = open->at(0);
			//check if the goal is reachable
			if(checkGoalReachable(voronoi, currentNode, closestVerticesToGoal, goal))
			{
				//if the goal is reachable construct the path to the goal
				shared_ptr<SearchNode> temp = currentNode;
				ret->push_back(goal);
				ret->push_back(make_shared<geometry::CNPoint2D>(currentNode->getVertex()->x, currentNode->getVertex()->y));
				while(temp->getPredecessor() != nullptr)
				{
					ret->push_back(make_shared<geometry::CNPoint2D>(temp->getPredecessor()->getVertex()->x, temp->getPredecessor()->getVertex()->y));
					temp = temp->getPredecessor();
				}
				reverse(ret->begin(), ret->end());
				lastPath = ret;
				lastClosestNode = nullptr;
				return ret;
			}
			// put current node into closed and delete it from open
			closed->push_back(currentNode);
			open->erase(open->begin());
			//expand the current node
			voronoi->expandNode(currentNode, open, closed, startPos, goal, eval);
			//if there is no closest node set one
			if(lastClosestNode == nullptr )
			{
				lastClosestNode = currentNode;
			}
			else
			{
				//if the distance of the current node is closer to the goal than the last one set it
				if(currentNode->getVertex()->distanceTo(goal) < lastClosestNode->getVertex()->distanceTo(goal))
				{
					lastClosestNode = currentNode;
				}
			}
		}
		return nullptr;
	}

	/**
	 * inserts a searchnode into a sorted vector
	 */
	void msl::PathPlanner::insert(shared_ptr<vector<shared_ptr<SearchNode> > > vect, shared_ptr<SearchNode> currentNode)
	{
		vector<shared_ptr<SearchNode> >::iterator it = std::upper_bound(vect->begin(), vect->end(), currentNode,
																		PathPlanner::compare); // find proper position in descending order
		vect->insert(it, currentNode); // insert before iterator it
	}

	/**
	 * compares two search nodes
	 */
	bool msl::PathPlanner::compare(shared_ptr<SearchNode> first, shared_ptr<SearchNode> second)
	{
		if (first->getCost() < second->getCost())
		{
			return true;
		}
		return false;
	}

	/**
	 * checks if vertices of goal face are reached
	 */
	bool PathPlanner::checkGoalVerticesReached(
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > closestVerticesToGoal,
			shared_ptr<SearchNode> currentNode)
	{
		bool found = false;
		for (int i = 0; i < closestVerticesToGoal->size(); i++)
		{
			//if distance is close we have reached a goal vertex
			if (abs(currentNode->getVertex()->x - closestVerticesToGoal->at(i)->x) < 10.0
					&& abs(currentNode->getVertex()->y - closestVerticesToGoal->at(i)->y) < 10.0)
			{
				found = true;
				break;
			}
		}
		return found;
	}

	/**
	 * checks if there is an obstacle inside the corridor
	 * @param voronoi shared_ptr<VoronoiNet>
	 * @param currentPos shared_ptr<geometry::CNPoint2D>
	 * @param goal shared_ptr<geometry::CNPoint2D>
	 * @param obstaclePoint shared_ptr<geometry::CNPoint2D>
	 * @return bool true if inside corridor false otherwise
	 */
	bool msl::PathPlanner::corridorCheck(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> currentPos,
											shared_ptr<geometry::CNPoint2D> goal,
											shared_ptr<geometry::CNPoint2D> obstaclePoint)
	{
		//calculate length x and y offset
		double length = voronoi->calcDist(currentPos, goal);
		double dx = currentPos->x - goal->x;
		double dy = currentPos->y - goal->y;
		double dist = std::sqrt(dx * dx + dy * dy);
		dx /= dist;
		dy /= dist;
		//calculate corridor corner points
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
		//send debug msg
		if (pathPlannerDebug)
		{
			sendCorridorCheck(points);
		}
		//return result
		return obstaclePoint != nullptr
				&& geometry::GeometryCalculator::isInsidePolygon(points, points.size(), obstaclePoint);
	}

	/**
	 * checks if there is an obstacle inside the corridor
	 * @param voronoi shared_ptr<VoronoiNet>
	 * @param currentPos shared_ptr<geometry::CNPoint2D>
	 * @param goal shared_ptr<geometry::CNPoint2D>
	 * @param obstaclePoint shared_ptr<geometry::CNPoint2D>
	 * @return bool true if inside corridor false otherwise
	 */
	bool msl::PathPlanner::corridorCheckBall(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> currentPos,
												shared_ptr<geometry::CNPoint2D> goal,
												shared_ptr<geometry::CNPoint2D> obstaclePoint)
	{
		//calculate length x and y offset
		double length = voronoi->calcDist(currentPos, goal);
		double dx = currentPos->x - goal->x;
		double dy = currentPos->y - goal->y;
		double dist = std::sqrt(dx * dx + dy * dy);
		dx /= dist;
		dy /= dist;
		//calculate corridor corner points
		shared_ptr<geometry::CNPoint2D> p1 = make_shared<geometry::CNPoint2D>(
				currentPos->x + (this->robotDiameter / 2) * dy, currentPos->y - (this->robotDiameter / 2) * dx);
		shared_ptr<geometry::CNPoint2D> p2 = make_shared<geometry::CNPoint2D>(
				currentPos->x - (this->robotDiameter / 2) * dy, currentPos->y + (this->robotDiameter / 2) * dx);
		shared_ptr<geometry::CNPoint2D> p3 = make_shared<geometry::CNPoint2D>(
				goal->x
						+ std::max(wm->ball.getBallDiameter() / 2 + this->additionalBallCorridorWidth,
									length / this->corridorWidthDivisorBall + this->additionalBallCorridorWidth) * dy,
				goal->y
						- std::max(wm->ball.getBallDiameter() / 2 + this->additionalBallCorridorWidth,
									length / this->corridorWidthDivisorBall + this->additionalBallCorridorWidth) * dx);
		shared_ptr<geometry::CNPoint2D> p4 = make_shared<geometry::CNPoint2D>(
				goal->x
						- std::max(wm->ball.getBallDiameter() / 2 + this->additionalBallCorridorWidth,
									length / this->corridorWidthDivisorBall + this->additionalBallCorridorWidth) * dy,
				goal->y
						+ std::max(wm->ball.getBallDiameter() / 2 + this->additionalBallCorridorWidth,
									length / this->corridorWidthDivisorBall + this->additionalBallCorridorWidth) * dx);
		vector<shared_ptr<geometry::CNPoint2D>> points;
		points.push_back(p1);
		points.push_back(p3);
		points.push_back(p4);
		points.push_back(p2);
		//send debug msg
		if (pathPlannerDebug)
		{
			sendCorridorCheck(points);
		}
		//return result
		return obstaclePoint != nullptr
				&& geometry::GeometryCalculator::isInsidePolygon(points, points.size(), obstaclePoint);
	}
	/**
	 * helping method to debug the corridor check
	 */
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
	/**
	 * gets last returned path
	 */
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> msl::PathPlanner::getLastPath()
	{
		return lastPath;
	}
	/**
	 * gets wm pointer
	 */
	MSLWorldModel* msl::PathPlanner::getWm()
	{
		return wm;
	}
	/**
	 * gets additionalCorridorWidth
	 */
	double msl::PathPlanner::getAdditionalCorridorWidth()
	{
		return additionalCorridorWidth;
	}

	/**
	 * checks if there is an obstacle inside the corridor
	 * @param voronoi VoronoiNet*
	 * @param currentPos shared_ptr<geometry::CNPoint2D>
	 * @param goal shared_ptr<geometry::CNPoint2D>
	 * @param obstaclePoint shared_ptr<geometry::CNPoint2D>
	 * @return bool true if inside corridor false otherwise
	 */
	bool msl::PathPlanner::corridorCheck(VoronoiNet* voronoi, shared_ptr<geometry::CNPoint2D> currentPos,
											shared_ptr<geometry::CNPoint2D> goal,
											shared_ptr<geometry::CNPoint2D> obstaclePoint)
	{
		//calculate length x and y offset
		double length = voronoi->calcDist(currentPos, goal);
		double dx = currentPos->x - goal->x;
		double dy = currentPos->y - goal->y;
		double dist = std::sqrt(dx * dx + dy * dy);
		dx /= dist;
		dy /= dist;
		//calculate corridor corner points
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
		//send debug msg
		if (pathPlannerDebug)
		{
			sendCorridorCheck(points);
		}
		// return result
		return obstaclePoint != nullptr
				&& geometry::GeometryCalculator::isInsidePolygon(points, points.size(), obstaclePoint);
	}
	/**
	 * gets last planning target
	 */
	shared_ptr<geometry::CNPoint2D> msl::PathPlanner::getLastTarget()
	{
		return lastTarget;
	}
	/**
	 * check if the goal vertces are reached and if there is a corridor leading to the goal
	 */
	bool PathPlanner::checkGoalReachable(shared_ptr<VoronoiNet> voronoi, shared_ptr<SearchNode> currentNode,
											shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> closestVerticesToGoal, shared_ptr<geometry::CNPoint2D> goal)
	{
		bool found = false;
		//we have to reach the goal vertices
		found = checkGoalVerticesReached(closestVerticesToGoal, currentNode);
		if(found)
		{
			//if the goal vertices are reached
			shared_ptr<VoronoiDiagram::Point_2> obstacle = voronoi->getSiteOfFace(Point_2(goal->x, goal->y));
			shared_ptr<geometry::CNPoint2D> obstaclePoint = make_shared<geometry::CNPoint2D>(obstacle->x(), obstacle->y());
			//check if there is an obstacle on the way to the goal
			return !corridorCheck(voronoi, currentNode->getVertex(), goal, obstaclePoint);
		}
		return false;
	}

	/**
	 * gets the base voronoi net with artificial obstacles
	 */
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

	/**
	 * gets robot diameter
	 */
	double PathPlanner::getRobotDiameter()
	{
		return robotDiameter;
	}

	/**
	 * gets pathDeviationWeight
	 */
	double PathPlanner::getPathDeviationWeight()
	{
		return pathDeviationWeight;
	}

	/**
	 * get dribbleAngleTolerance
	 */
	double PathPlanner::getDribbleAngleTolerance()
	{
		return dribble_angleTolerance;
	}

	/**
	 * gets dribbleRotationWeight
	 */
	double PathPlanner::getDribbleRotationWeight()
	{
		return dribble_rotationWeight;
	}

	/**
	 * gets artificial obstacles as goemtry::CNPonit2D
	 */
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > PathPlanner::getArtificialObstacles()
	{
		MSLFootballField* field = MSLFootballField::getInstance();
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > toInsert = make_shared<
				vector<shared_ptr<geometry::CNPoint2D>>>();
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

