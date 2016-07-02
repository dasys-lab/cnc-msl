/*
 * PathPlanner.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#include <pathplanner/PathPlanner.h>
#include "MSLWorldModel.h"
#include "GeometryCalculator.h"
#include "RawSensorData.h"
#include "Ball.h"

namespace msl
{

	PathPlanner::PathPlanner(MSLWorldModel* wm, int count)
	{
		this->wm = wm;
		this->artificialObjectNet = make_shared<VoronoiNet>(wm);
		this->lastClosestNode = nullptr;
		this->lastClosestPointToBlock = nullptr;
		this->lastTarget == nullptr;
		sc = supplementary::SystemConfig::getInstance();
		for (int i = 0; i < count; i++)
		this->voronoiDiagrams.reserve(10);
		{
		        auto voi = make_shared<VoronoiNet>(wm);
			// TODO maybe VoronoiNet::generateVoronoiDiagram can spare inserting these art. obstacles again, if we maintain them properly
			voi->setVoronoi(
					make_shared<VoronoiDiagram>(
							(DelaunayTriangulation)this->artificialObjectNet->getVoronoi()->dual()));

                        this->voronoiDiagrams.push_back(voi);
		}
		this->robotRadius = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL);
		this->minEdgeWidth = (*sc)["PathPlanner"]->get<double>("PathPlanner", "minEdgeWidth", NULL);
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
	void PathPlanner::prepareVoronoiDiagram()
	{
		lock_guard<mutex> lock(voronoiMutex);
		auto ownPos = wm->rawSensorData->getOwnPositionVision();
		currentVoronoiPos = (currentVoronoiPos + 1) % voronoiDiagrams.size();
		if (ownPos != nullptr)
		{
			voronoiDiagrams.at(currentVoronoiPos)->generateVoronoiDiagram(true);
		}
		else
		{
			voronoiDiagrams.at(currentVoronoiPos)->generateVoronoiDiagram(false);
		}
	}

	/**
	 * initializes artificial objects and insert them into artificial object net
	 */
	void PathPlanner::initializeArtificialObstacles()
	{
		//TODO before 2 Meters now field->surrounding * 2
		vector<VoronoiDiagram::Site_2> toInsert;
		int baseSize = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "artificialObjectBaseSize", NULL);
		if (wm->field->getFieldLength() / baseSize > 20 || wm->field->getFieldWidth() / baseSize > 20)
		{
			baseSize = (int)max(wm->field->getFieldLength() / 20, wm->field->getFieldWidth() / 20);
		}
		//TODO was time 2 before
		int lengthInterval = (int)(baseSize
				+ ((int)(wm->field->getFieldLength() + wm->field->getSurrounding() * 1.5) % baseSize)
						/ (int)((int)(wm->field->getFieldLength() + wm->field->getSurrounding() * 1.5) / baseSize));
		int widthInterval = (int)(baseSize
				+ ((int)(wm->field->getFieldWidth() + wm->field->getSurrounding() * 1.5) % baseSize)
						/ (int)((int)(wm->field->getFieldWidth() + wm->field->getSurrounding() * 1.5) / baseSize));
		int halfFieldLength = (int)wm->field->getFieldLength() / 2 + wm->field->getSurrounding();
		int halfFieldWidth = (int)wm->field->getFieldWidth() / 2 + wm->field->getSurrounding();

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
	 * @param ownPos shared_ptr<geometry::CNPoint2D>
	 * @param goal shared_ptr<geometry::CNPoint2D>
	 * @return PATH shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
	 */
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> PathPlanner::plan(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<IPathEvaluator> eval)
	{
		if(lastTarget == nullptr || lastTarget->distanceTo(goal) > 50)
		{ //if there was no target before || there was a target before, it has to be different to be saved
			lastTarget = goal;
		}

		//check if the goal is reachable directly by checking a corridor between the robot and the goal
		bool reachable = true;

		auto vec = voronoi->getAlloClusteredObsWithMe();
		double shortestDist = 100000000.0;
		shared_ptr<geometry::CNPoint2D> closestOpp;
		for(auto cluster : *vec)
		{
			if(cluster->id == wm->getOwnId())
			{
				continue;
			}

			//if there is an obstacle inside the corridor the goal is not reachable
			auto point = cluster->getPoint();
			if(corridorCheck(startPos, goal, point, this->robotRadius))
			{
				reachable = false;
				break;
			}

			double tmpDist = goal->distanceTo(point);
			if (tmpDist < shortestDist)
			{
				closestOpp = point;
				shortestDist = tmpDist;
			}
		}


		// this helps to avoid obstacles which are very close to the goal, but behind it -
		// in such cases the corridor check say, you can drive directly to the goal, although an opponent robots corner can reach into the corridor...
		if (closestOpp && shortestDist < 400 && closeOppToBallCheck(voronoi, startPos, goal, closestOpp))
		{
			reachable = false;
		}

		if (reachable)
		{
		        auto vec = voronoi->getArtificialObstacles();
			for(auto obs : *vec)
			{
				//if there is an obstacle inside the corridor the goal is not reachable
				if(corridorCheck(startPos, goal, obs))
				{
					reachable = false;
					break;
				}
			}
		}

		if (reachable)
		{
				auto vec = voronoi->getAdditionalObstacles();
				for(auto obs : *vec)
				{
						//if there is an obstacle inside the corridor the goal is not reachable
						if(corridorCheck(startPos, goal, obs))
						{
								reachable = false;
								break;
						}
				}
		}

		//if the corridor is free the robot can drive towards the goal
		if(reachable)
		{
			auto ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
			ret->push_back(goal);
			lastPath = ret;
			return ret;
		}

		//there is no direct way so we have to search a way to the goal
		auto ret = aStarSearch(voronoi, startPos, goal, eval);
		//if there is a path save and return it
		if(ret != nullptr)
		{
			lastPath = ret;
			return ret;
		}

		/**
		 * In this case, we should be surrounded and therefore, we try to drive in the direction which gives us the most space according to
		 * our surrounding voronoi vertices.
		 */

		ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		shared_ptr<Vertex> bestVertex = nullptr;
		double bestDist = 0;
		//find vertex of robots voronoi face and which has maximum distance
		for(auto vertice : *voronoi->getVerticesOfFace(startPos))
		{
			double curDist = distanceTo(startPos, vertice);
			if(curDist > bestDist)
			{
				bestVertex = vertice;
				bestDist = curDist;
			}
		}

		ret->push_back(make_shared<geometry::CNPoint2D>(bestVertex->point().x(), bestVertex->point().y()));

		lastPath = ret;
		return ret;
	}

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > PathPlanner::aStarSearch(
			shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> startPos,
			shared_ptr<geometry::CNPoint2D> goal, shared_ptr<IPathEvaluator> pathEvaluator)
	{
		// return
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();

		// vector with open searchnodes
		shared_ptr<vector<shared_ptr<SearchNode>>> open = make_shared<vector<shared_ptr<SearchNode>>>();

		//vector with closed search nodes
		shared_ptr<vector<shared_ptr<SearchNode>>> closed = make_shared<vector<shared_ptr<SearchNode>>>();

		//get closest Vertices to ownPos => start point for a star serach
		shared_ptr<vector<shared_ptr<Vertex>>> closestVerticesToOwnPos = voronoi->getVerticesOfFace(startPos);

		// get closest Vertices to goal => goal for a star serach
		shared_ptr<vector<shared_ptr<Vertex>>> closestVerticesToGoal = voronoi->getVerticesOfFace(goal);

		// a star serach

		//insert all vertices of startpos face
		for(auto vertice : *closestVerticesToOwnPos)
		{
			// TODO create the right initial costs
			auto node = make_shared<SearchNode>(vertice, 0,0, nullptr);
			pair<double,double> costHeuristicPair = pathEvaluator->evalInitial(startPos, goal, node, voronoi.get(), this->lastPath, this->lastTarget);
			node->setCost(costHeuristicPair.first);
			node->setHeuristic(costHeuristicPair.second);
			insert(open, node);
		}

		shared_ptr<SearchNode> currentNode, stop;

		// while there is still a node to expand
		while(open->size() != 0)
		{
			//get first node in open
			currentNode = open->at(0);
			open->erase(open->begin());
			closed->push_back(currentNode);

			//check if the goal is reachable
			if(checkGoalReachable(voronoi, currentNode, closestVerticesToGoal, goal))
			{
				stop = currentNode;
				ret->push_back(goal);
				break;
			}

			//expand the current node
			this->expandNode(currentNode, open, closed, startPos, goal, pathEvaluator, voronoi);

			//if there is no closest node || the distance of the current node is closer to the goal than the last one
			if(stop == nullptr || goal->distanceTo(currentNode->getPoint()) < goal->distanceTo(stop->getPoint()))
			{
				stop = currentNode;
			}
		}

		if (stop == nullptr)
		return nullptr;

		//get the way to reach this node
		ret->push_back(stop->getPoint());
		currentNode = stop;
		while((currentNode = currentNode->getPredecessor()) != nullptr)
		{
			ret->push_back(currentNode->getPoint());
		}
		reverse(ret->begin(), ret->end());

//		lastPath = ret;
//		lastClosestNode = nullptr;
		return ret;
	}

	bool PathPlanner::isAdmissableEdge(VoronoiDiagram::Halfedge_around_vertex_circulator incidentHalfEdge,
											shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<VoronoiNet> voronoi)
		{
			// dont follow edges to infinity
			if (!incidentHalfEdge->has_source())
			{
				return false;
			}

			// dont follow edges through art obstacles
			if (voronoi->getTypeOfSite(incidentHalfEdge->up()->point()) == EntityType::ArtificialObstacle
					&& voronoi->getTypeOfSite(incidentHalfEdge->down()->point()) == EntityType::ArtificialObstacle)
			{
				return !wm->field->isInsideField(startPos);
			}

			double distToEdge = geometry::distancePointToLineSegment(incidentHalfEdge->up()->point().x(),
																		incidentHalfEdge->up()->point().y(),
																		incidentHalfEdge->source()->point().x(),
																		incidentHalfEdge->source()->point().y(),
																		incidentHalfEdge->target()->point().x(),
																		incidentHalfEdge->target()->point().y());

			if (distToEdge < this->minEdgeWidth)
			{
				return false;
			}

			return true;

		}

	/**
	 * expands a SearchNode
	 * @param currentNode shared_ptr<SearchNode>
	 * @param open shared_ptr<vector<shared_ptr<SearchNode>>>
	 * @param closed shared_ptr<vector<shared_ptr<SearchNode>>>
	 * @param startPos shared_ptr<geometry::CNPoint2D>
	 * @param goal shared_ptr<geometry::CNPoint2D>
	 * @param eval shared_ptr<PathEvaluator>
	 */
	void PathPlanner::expandNode(shared_ptr<SearchNode> currentNode, shared_ptr<vector<shared_ptr<SearchNode>>> open,
	shared_ptr<vector<shared_ptr<SearchNode>>> closed, shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<IPathEvaluator> pathEvaluator,shared_ptr<VoronoiNet> voronoi )
	{
		// get neighbored nodes
		vector<VoronoiDiagram::Halfedge_around_vertex_circulator> neighborEdges;// = getNeighboredVertices(currentNode);
		VoronoiDiagram::Halfedge_around_vertex_circulator incidentHalfEdge = currentNode->getIncidentEdges();
		VoronoiDiagram::Halfedge_around_vertex_circulator begin = incidentHalfEdge;
		do
		{
			if (this->isAdmissableEdge(incidentHalfEdge, startPos, voronoi))
			{
				// if node is already closed || if node has still to be expanded but there is a cheaper way
				if (contains(closed, incidentHalfEdge) || contains(open, incidentHalfEdge))
				{
					continue;
				}

				// TODO init correct costs + heuristic
				neighborEdges.push_back(incidentHalfEdge);
			}
		}while(begin != ++incidentHalfEdge);

		for (auto edge : neighborEdges)
		{
			auto nextNode = make_shared<SearchNode>(edge, 0, 0, nullptr);
			pair<double, double> costHeuristicPair = pathEvaluator->eval(goal, currentNode, nextNode, voronoi.get());
			if(costHeuristicPair.first > 0)
			{
				nextNode->setPredecessor(currentNode);
				nextNode->setCost(costHeuristicPair.first);
				nextNode->setHeuristic(costHeuristicPair.second);
				PathPlanner::insert(open, nextNode);
			}
		}
	}

	/**
	 * checks if a SearchNode is part of a vector
	 * @param vector shared_ptr<vector<shared_ptr<SearchNode> > >
	 * @param vertex shared_ptr<SearchNode>
	 * @return bool
	 */
	bool PathPlanner::contains(shared_ptr<vector<shared_ptr<SearchNode> > > vector, VoronoiDiagram::Halfedge_around_vertex_circulator edge )
	{
		for (auto node : *vector)
		{
			if (edge == (*node->getEdge()))
			{
				return true;
			}
		}
		return false;
	}

	/**
	 * inserts a searchnode into a sorted vector
	 */
	void PathPlanner::insert(shared_ptr<vector<shared_ptr<SearchNode> > > vect, shared_ptr<SearchNode> currentNode)
	{
		vector<shared_ptr<SearchNode> >::iterator it = std::upper_bound(vect->begin(), vect->end(), currentNode,
																		PathPlanner::compare); // find proper position in descending order
		vect->insert(it, currentNode); // insert before iterator it
	}

	/**
	 * compares two search nodes
	 */
	bool PathPlanner::compare(shared_ptr<SearchNode> first, shared_ptr<SearchNode> second)
	{
		if (first->getCost()+first->getHeuristic() < second->getCost() + second->getHeuristic())
		{
			return true;
		}
		return false;
	}

	/**
	 * checks if vertices of goal face are reached
	 */
	bool PathPlanner::checkGoalVerticesReached(shared_ptr<vector<shared_ptr<Vertex> > > closestVerticesToGoal,
												shared_ptr<SearchNode> currentNode)
	{
		for (auto vertice : *closestVerticesToGoal)
		{
			if (currentNode->matches(vertice))
			{
				return true;
			}
		}
		return false;
	}

	bool PathPlanner::closeOppToBallCheck(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> currentPos,
												shared_ptr<geometry::CNPoint2D> goal,
												shared_ptr<geometry::CNPoint2D> obstaclePoint)
	{
		auto g2opp = obstaclePoint-goal;
		double angle2BallOppLine = abs(geometry::deltaAngle(currentPos->angleTo(),g2opp->angleTo()));
		//cout << "PathPlanner: angle2BallOppLine: \t" << angle2BallOppLine << endl;
		if (angle2BallOppLine > M_PI/2.0 && angle2BallOppLine < M_PI * 2.0/3.0)
		{
			return true;
		}
		return false;
	}

	/**
	 * checks if there is an obstacle inside the corridor
	 * @param voronoi shared_ptr<VoronoiNet>
	 * @param currentPos shared_ptr<geometry::CNPoint2D>
	 * @param goal shared_ptr<geometry::CNPoint2D>
	 * @param obstaclePoint shared_ptr<geometry::CNPoint2D>
	 * @return bool true if inside corridor false otherwise
	 */
	bool PathPlanner::corridorCheck(shared_ptr<geometry::CNPoint2D> currentPos,
											shared_ptr<geometry::CNPoint2D> goal,
											shared_ptr<geometry::CNPoint2D> obstaclePoint, double obstacleRadius)
	{
		//calculate length x and y offset
		double length = currentPos->distanceTo(goal);
		double dx = currentPos->x - goal->x;
		double dy = currentPos->y - goal->y;
		dx /= length;
		dy /= length;
		//calculate corridor corner points
		shared_ptr<geometry::CNPoint2D> p1 = make_shared<geometry::CNPoint2D>(
				currentPos->x + (this->robotRadius + this->additionalCorridorWidth) * dy,
				currentPos->y - (this->robotRadius + this->additionalCorridorWidth) * dx);
		shared_ptr<geometry::CNPoint2D> p2 = make_shared<geometry::CNPoint2D>(
				currentPos->x - (this->robotRadius + this->additionalCorridorWidth) * dy,
				currentPos->y + (this->robotRadius + this->additionalCorridorWidth) * dx);
		shared_ptr<geometry::CNPoint2D> p3 = make_shared<geometry::CNPoint2D>(
				goal->x
						+ std::max(this->robotRadius + this->additionalCorridorWidth,
									length / this->corridorWidthDivisor + this->additionalCorridorWidth) * dy,
				goal->y
						- std::max(this->robotRadius + this->additionalCorridorWidth,
									length / this->corridorWidthDivisor + this->additionalCorridorWidth) * dx);
		shared_ptr<geometry::CNPoint2D> p4 = make_shared<geometry::CNPoint2D>(
				goal->x
						- std::max(this->robotRadius + this->additionalCorridorWidth,
									length / this->corridorWidthDivisor + this->additionalCorridorWidth) * dy,
				goal->y
						+ std::max(this->robotRadius + this->additionalCorridorWidth,
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

//		cout << "----------------------" << endl;
		return obstaclePoint != nullptr && (geometry::distancePointToLine(p2, p4, obstaclePoint) < this->robotRadius
				&& geometry::distancePointToLine(p4, p3, obstaclePoint) < 0.0
				&& geometry::distancePointToLine(p3, p1, obstaclePoint) < this->robotRadius
				&& geometry::distancePointToLine(p1, p2, obstaclePoint) < 0.0);
		//return result
		//return obstaclePoint != nullptr && geometry::isInsidePolygon(points, obstaclePoint);
	}

	/**
	 * checks if there is an obstacle inside the corridor
	 * @param voronoi shared_ptr<VoronoiNet>
	 * @param currentPos shared_ptr<geometry::CNPoint2D>
	 * @param goal shared_ptr<geometry::CNPoint2D>
	 * @param obstaclePoint shared_ptr<geometry::CNPoint2D>
	 * @return bool true if inside corridor false otherwise
	 */
	bool PathPlanner::corridorCheckBall(shared_ptr<geometry::CNPoint2D> currentPos,
												shared_ptr<geometry::CNPoint2D> goal,
												shared_ptr<geometry::CNPoint2D> obstaclePoint,double obstacleRadius)
	{
		//calculate length x and y offset
		double length = currentPos->distanceTo(goal);
		double dx = currentPos->x - goal->x;
		double dy = currentPos->y - goal->y;
		double dist = length;
		dx /= dist;
		dy /= dist;
		//calculate corridor corner points
		shared_ptr<geometry::CNPoint2D> p1 = make_shared<geometry::CNPoint2D>(
				currentPos->x + (this->robotRadius) * dy, currentPos->y - (this->robotRadius) * dx);
		shared_ptr<geometry::CNPoint2D> p2 = make_shared<geometry::CNPoint2D>(
				currentPos->x - (this->robotRadius) * dy, currentPos->y + (this->robotRadius) * dx);
		shared_ptr<geometry::CNPoint2D> p3 = make_shared<geometry::CNPoint2D>(
				goal->x
						+ std::max(wm->ball->getBallDiameter() / 2 + this->additionalBallCorridorWidth,
									length / this->corridorWidthDivisorBall + this->additionalBallCorridorWidth) * dy,
				goal->y
						- std::max(wm->ball->getBallDiameter() / 2 + this->additionalBallCorridorWidth,
									length / this->corridorWidthDivisorBall + this->additionalBallCorridorWidth) * dx);
		shared_ptr<geometry::CNPoint2D> p4 = make_shared<geometry::CNPoint2D>(
				goal->x
						- std::max(wm->ball->getBallDiameter() / 2 + this->additionalBallCorridorWidth,
									length / this->corridorWidthDivisorBall + this->additionalBallCorridorWidth) * dy,
				goal->y
						+ std::max(wm->ball->getBallDiameter() / 2 + this->additionalBallCorridorWidth,
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
		return obstaclePoint != nullptr && (geometry::distancePointToLine(p2, p4, obstaclePoint) < this->robotRadius
						&& geometry::distancePointToLine(p4, p3, obstaclePoint) < this->robotRadius
						&& geometry::distancePointToLine(p3, p1, obstaclePoint) < this->robotRadius
						&& geometry::distancePointToLine(p1, p2, obstaclePoint) < this->robotRadius);
		//return result
		//return obstaclePoint != nullptr && geometry::isInsidePolygon(points, obstaclePoint);
	}
	/**
	 * helping method to debug the corridor check
	 */
	void PathPlanner::sendCorridorCheck(vector<shared_ptr<geometry::CNPoint2D> > points)
	{
		msl_msgs::CorridorCheck cc;
		cc.senderId = this->wm->getOwnId();
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
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> PathPlanner::getLastPath()
	{
		return lastPath;
	}

	/**
	 * gets additionalCorridorWidth
	 */
	double PathPlanner::getAdditionalCorridorWidth()
	{
		return additionalCorridorWidth;
	}


	/**
	 * gets last planning target
	 */
	shared_ptr<geometry::CNPoint2D> PathPlanner::getLastTarget()
	{
		return lastTarget;
	}

	double PathPlanner::getRobotRadius() {
		return this->robotRadius;
	}

	/**
	 * check if the goal vertices are reached and if there is a corridor leading to the goal
	 */
	bool PathPlanner::checkGoalReachable(shared_ptr<VoronoiNet> voronoi, shared_ptr<SearchNode> currentNode,
											shared_ptr<vector<shared_ptr<Vertex>>> closestVerticesToGoal, shared_ptr<geometry::CNPoint2D> goal)
	{
		//we have to reach the goal vertices
		if(checkGoalVerticesReached(closestVerticesToGoal, currentNode))
		{
			//if the goal vertices are reached
			shared_ptr<VoronoiDiagram::Point_2> obstacle = voronoi->getSiteOfFace(Point_2(goal->x, goal->y));
			if (voronoi->getTypeOfSite(*obstacle) == this->wm->getOwnId())
				return true;
			shared_ptr<geometry::CNPoint2D> obstaclePoint = make_shared<geometry::CNPoint2D>(obstacle->x(), obstacle->y());
			//check if there is an obstacle on the way to the goal
			shared_ptr<geometry::CNPoint2D> vertexPoint = currentNode->getPoint();
			return !corridorCheck(vertexPoint, goal, obstaclePoint, this->robotRadius);
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
	 * gets latest accessible VoronoiNet
	 * @return shared_ptr<VoronoiNet>
	 */
	shared_ptr<VoronoiNet> PathPlanner::getCurrentVoronoiNet()
	{
		lock_guard<mutex> lock(this->voronoiMutex);
		if (currentVoronoiPos == -1)
		{
			return nullptr;
		}
		shared_ptr<VoronoiNet> ret = make_shared<VoronoiNet>(voronoiDiagrams.at(currentVoronoiPos));
		return ret;
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
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > PathPlanner::getArtificialFieldSurroundingObs()
	{
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > toInsert = make_shared<
				vector<shared_ptr<geometry::CNPoint2D>>>();
		int baseSize = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "artificialObjectBaseSize", NULL);
		if (wm->field->getFieldLength() / baseSize > 20 || wm->field->getFieldWidth() / baseSize > 20)
		{
			baseSize = (int)max(wm->field->getFieldLength() / 20, wm->field->getFieldWidth() / 20);
		}
		int lengthInterval = (int)(baseSize
				+ ((int)(wm->field->getFieldLength() + 2000) % baseSize) / (int)((int)(wm->field->getFieldLength() + 2000) / baseSize));
		int widthInterval = (int)(baseSize
				+ ((int)(wm->field->getFieldWidth() + 2000) % baseSize) / (int)((int)(wm->field->getFieldWidth() + 2000) / baseSize));
		int halfFieldLength = (int)wm->field->getFieldLength() / 2 + 1000;
		int halfFieldWidth = (int)wm->field->getFieldWidth() / 2 + 1000;

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

	double PathPlanner::distanceTo(shared_ptr<geometry::CNPoint2D> v1, shared_ptr<Vertex> v2)
	{
		return sqrt(pow(v2->point().x() - v1->x, 2) + pow(v2->point().y() - v1->y, 2));
	}

} /* namespace alica */


