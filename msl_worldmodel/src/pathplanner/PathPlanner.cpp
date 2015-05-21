/*
 * PathPlanner.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#include <pathplanner/PathPlanner.h>
#include "MSLWorldModel.h"

namespace msl
{

	PathPlanner::PathPlanner(MSLWorldModel* wm, int count) :
			voronoiDiagrams(count), artificialObjectNet(wm)
	{
		this->wm = wm;
		sc = supplementary::SystemConfig::getInstance();
		for (int i = 0; i < count; i++)
		{
			this->voronoiDiagrams.at(i) = make_shared<VoronoiNet>(wm);
			this->voronoiDiagrams.at(i)->setVoronoi(
					make_shared<VoronoiDiagram>((DelaunayTriangulation)this->artificialObjectNet.getVoronoi()->dual()));
		}
		this->robotDiameter = (*this->sc)["Globals"]->get<double>("Globals", "Dimensions", "DiameterRobot", NULL);
		initializeArtificialObstacles();
		this->pathDeviationWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "pathDeviationWeight", NULL);
		this->currentVoronoiPos = -1;

	}

	PathPlanner::~PathPlanner()
	{
	}

	/**
	 * aStar search on a VoronoiDiagram
	 * @param voronoi shared_ptr<VoronoiNet>
	 * @param ownPos Point_2
	 * @param goal Point_2
	 * @return shared_ptr<vector<shared_ptr<Point_2>>>
	 */
	shared_ptr<vector<shared_ptr<CNPoint2D>>> PathPlanner::aStarSearch(shared_ptr<VoronoiNet> voronoi, CNPoint2D startPos, CNPoint2D goal)
	{
		bool reachable = true;
		for(auto it = voronoi->getVoronoi()->sites_begin(); it != voronoi->getVoronoi()->sites_end(); it++)
		{
			shared_ptr<VoronoiDiagram::Point_2> obstaclePoint = voronoi->getSiteOfFace(Point_2(goal.x, goal.y));
			double length = voronoi->calcDist(Point_2(startPos.x, startPos.y), Point_2(goal.x, goal.y));
			double dx = startPos.x - goal.x;
			double dy = startPos.y - goal.y;
			double dist = std::sqrt(dx*dx + dy*dy);
			dx /= dist;
			dy /= dist;
			VoronoiDiagram::Point_2 p1 = Point_2(startPos.x + (this->robotDiameter / 2) * dy,
					startPos.y - (this->robotDiameter / 2) * dx);
			VoronoiDiagram::Point_2 p2 = Point_2(startPos.x - (this->robotDiameter / 2) * dy,
					startPos.y + (this->robotDiameter / 2) * dx);
			VoronoiDiagram::Point_2 p3 = Point_2(goal.x + std::max(this->robotDiameter / 2,length / 4) * dy,
					goal.y - std::max(this->robotDiameter / 2,length / 4) * dx);
			VoronoiDiagram::Point_2 p4 = Point_2(goal.x - std::max(this->robotDiameter / 2,length / 4) * dy,
					goal.y + std::max(this->robotDiameter / 2,length / 4) * dx);
			vector<VoronoiDiagram::Point_2> points;
			points.push_back(p1);
			points.push_back(p3);
			points.push_back(p4);
			points.push_back(p2);
			if(isInside(points, points.size(), *obstaclePoint))
			{
				reachable = false;
				break;
			}
		}
		if(reachable)
		{
			shared_ptr<vector<shared_ptr<CNPoint2D>>> ret = make_shared<vector<shared_ptr<CNPoint2D>>>();
			ret->push_back(make_shared<CNPoint2D>(goal));
			return ret;
		}
		// return
		shared_ptr<vector<shared_ptr<CNPoint2D>>> ret = make_shared<vector<shared_ptr<CNPoint2D>>>();
		// vector with open searchnodes
		shared_ptr<vector<shared_ptr<SearchNode>>> open = make_shared<vector<shared_ptr<SearchNode>>>();
		//vector with closed search nodes
		shared_ptr<vector<shared_ptr<SearchNode>>> closed = make_shared<vector<shared_ptr<SearchNode>>>();

		//get closest Vertices to ownPos => start point for a star serach
		shared_ptr<vector<shared_ptr<VoronoiDiagram::Vertex>>> closestVerticesToOwnPos = voronoi->getVerticesOfFace(Point_2(startPos.x, startPos.y));

		// get closest Vertex to goal => goal for a star serach
		shared_ptr<vector<shared_ptr<VoronoiDiagram::Vertex>>> closestVerticesToGoal = voronoi->getVerticesOfFace(Point_2(goal.x, goal.y));

		// a star serach
		for(int i = 0; i < closestVerticesToOwnPos->size(); i++)
		{
			insert(open, make_shared<SearchNode>(SearchNode(closestVerticesToOwnPos->at(i),
									voronoi->calcDist(closestVerticesToOwnPos->at(i)->point(), Point_2(goal.x, goal.y)), nullptr)));
		}

		while(open->size() != 0)
		{
			shared_ptr<SearchNode> currentNode = open->at(0);

			if(checkGoalReachable(voronoi, currentNode, closestVerticesToGoal, goal))
			{
				shared_ptr<SearchNode> temp = currentNode;
				ret->push_back(make_shared<CNPoint2D>(currentNode->getVertex()->point().x(), currentNode->getVertex()->point().y()));
				while(temp->getPredecessor() != nullptr)
				{
					ret->push_back(make_shared<CNPoint2D>(temp->getPredecessor()->getVertex()->point().x(), temp->getPredecessor()->getVertex()->point().y()));
					temp = temp->getPredecessor();
				}
				reverse(ret->begin(), ret->end());
				return ret;
			}
			closed->push_back(currentNode);

			voronoi->expandNode(currentNode, open, closed, Point_2(goal.x, goal.y));
		}

		// return nullptr if there is no way to goal
		return nullptr;
	}

	/**
	 * processes the WorldModel msg
	 * @param msg msl_sensor_msgs::WorldModelDataPtr
	 */
	void PathPlanner::processWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg)
	{
		vector<CNPoint2D> points;
		if (wm->rawSensorData.getOwnPositionVision() != nullptr)
		{
			points.push_back(
					CNPoint2D(wm->rawSensorData.getOwnPositionVision()->x,
								wm->rawSensorData.getOwnPositionVision()->y));
		}
		for (int i = 0; i < msg->obstacles.size(); i++)
		{
			points.push_back(CNPoint2D(msg->obstacles.at(i).x, msg->obstacles.at(i).y));
		}
		lock_guard<mutex> lock(voronoiMutex);
		//TODO buffer size 10 + einfügen an index
		// status entfernen

		voronoiDiagrams.at((currentVoronoiPos + 1) % 10)->generateVoronoiDiagram(points);
		currentVoronoiPos++;

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
		if(currentVoronoiPos == -1)
		{
			return nullptr;
		}
		return voronoiDiagrams.at(currentVoronoiPos % 10);
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
		this->artificialObjectNet.getVoronoi()->insert(toInsert.begin(), toInsert.end());
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

	void msl::PathPlanner::insert(shared_ptr<vector<shared_ptr<SearchNode> > > vect, shared_ptr<SearchNode> currentNode)
	{
		vector<shared_ptr<SearchNode> >::iterator it = std::lower_bound(vect->begin(), vect->end(), currentNode,
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

	//TODO vielleicht int da 3 mgl besthen
	bool PathPlanner::checkGoalReachable(shared_ptr<VoronoiNet> voronoi, shared_ptr<SearchNode> currentNode,
											shared_ptr<vector<shared_ptr<VoronoiDiagram::Vertex>>> closestVerticesToGoal, CNPoint2D goal)
	{
		bool found = false;
		for(int i = 0; i < closestVerticesToGoal->size(); i++)
		{
			if(currentNode->getVertex()->point().x() == closestVerticesToGoal->at(i)->point().x()
			&& currentNode->getVertex()->point().y() == closestVerticesToGoal->at(i)->point().y())
			{
				found = true;
				break;
			}
		}
		if(found)
		{
			shared_ptr<VoronoiDiagram::Point_2> obstaclePoint = voronoi->getSiteOfFace(Point_2(goal.x, goal.y));
			double length = voronoi->calcDist(Point_2(currentNode->getVertex()->point().x(), currentNode->getVertex()->point().y()), Point_2(goal.x, goal.y));
			double dx = currentNode->getVertex()->point().x() - goal.x;
			double dy = currentNode->getVertex()->point().y() - goal.y;
			double dist = std::sqrt(dx*dx + dy*dy);
			dx /= dist;
			dy /= dist;
			VoronoiDiagram::Point_2 p1 = Point_2(currentNode->getVertex()->point().x() + (this->robotDiameter / 2) * dy,
			currentNode->getVertex()->point().y() - (this->robotDiameter / 2) * dx);
			VoronoiDiagram::Point_2 p2 = Point_2(currentNode->getVertex()->point().x() - (this->robotDiameter / 2) * dy,
			currentNode->getVertex()->point().y() + (this->robotDiameter / 2) * dx);
			VoronoiDiagram::Point_2 p3 = Point_2(goal.x + std::max(this->robotDiameter / 2,length / 4) * dy,
			goal.y - std::max(this->robotDiameter / 2,length / 4) * dx);
			VoronoiDiagram::Point_2 p4 = Point_2(goal.x - std::max(this->robotDiameter / 2,length / 4) * dy,
			goal.y + std::max(this->robotDiameter / 2,length / 4) * dx);
			vector<VoronoiDiagram::Point_2> points;
			points.push_back(p1);
			points.push_back(p3);
			points.push_back(p4);
			points.push_back(p2);
			return isInside(points, points.size(), *obstaclePoint);
		}
		return false;
	}

	// Given three colinear points p, q, r, the function checks if
	// point q lies on line segment 'pr'
	bool PathPlanner::onSegment(VoronoiDiagram::Point_2 p, VoronoiDiagram::Point_2 q, VoronoiDiagram::Point_2 r)
	{
		if (q.x() <= max(p.x(), r.x()) && q.x() >= min(p.x(), r.x()) && q.y() <= max(p.y(), r.y())
				&& q.y() >= min(p.y(), r.y()))
		{
			return true;
		}
		return false;
	}

	// To find orientation of ordered triplet (p, q, r).
	// The function returns following values
	// 0 --> p, q and r are colinear
	// 1 --> Clockwise
	// 2 --> Counterclockwise
	int PathPlanner::orientation(VoronoiDiagram::Point_2 p, VoronoiDiagram::Point_2 q, VoronoiDiagram::Point_2 r)
	{
		int val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());

		if (val == 0)
			return 0; // colinear
		return (val > 0) ? 1 : 2; // clock or counterclock wise
	}

	// The function that returns true if line segment 'p1q1'
	// and 'p2q2' intersect.
	bool PathPlanner::doIntersect(VoronoiDiagram::Point_2 p1, VoronoiDiagram::Point_2 q1, VoronoiDiagram::Point_2 p2,
									VoronoiDiagram::Point_2 q2)
	{
		// Find the four orientations needed for general and
		// special cases
		int o1 = orientation(p1, q1, p2);
		int o2 = orientation(p1, q1, q2);
		int o3 = orientation(p2, q2, p1);
		int o4 = orientation(p2, q2, q1);

		// General case
		if (o1 != o2 && o3 != o4)
			return true;

		// Special Cases
		// p1, q1 and p2 are colinear and p2 lies on segment p1q1
		if (o1 == 0 && onSegment(p1, p2, q1))
			return true;

		// p1, q1 and p2 are colinear and q2 lies on segment p1q1
		if (o2 == 0 && onSegment(p1, q2, q1))
			return true;

		// p2, q2 and p1 are colinear and p1 lies on segment p2q2
		if (o3 == 0 && onSegment(p2, p1, q2))
			return true;

		// p2, q2 and q1 are colinear and q1 lies on segment p2q2
		if (o4 == 0 && onSegment(p2, q1, q2))
			return true;

		return false; // Doesn't fall in any of the above cases
	}

	// Returns true if the point p lies inside the polygon[] with n vertices
	bool PathPlanner::isInside(vector<VoronoiDiagram::Point_2> polygon, int n, VoronoiDiagram::Point_2 p)
	{
		// There must be at least 3 vertices in polygon[]
		if (n < 3)
			return false;

		// Create a point for line segment from p to infinite
		VoronoiDiagram::Point_2 extreme = VoronoiDiagram::Point_2(
				(*this->sc)["PathPlanner"]->get<double>("PathPlanner", "pointAtInfinityX", NULL), p.y());

		// Count intersections of the above line with sides of polygon
		int count = 0, i = 0;
		do
		{
			int next = (i + 1) % n;

			// Check if the line segment from 'p' to 'extreme' intersects
			// with the line segment from 'polygon[i]' to 'polygon[next]'
			if (doIntersect(polygon[i], polygon[next], p, extreme))
			{
				// If the point 'p' is colinear with line segment 'i-next',
				// then check if it lies on segment. If it lies, return true,
				// otherwise false
				if (orientation(polygon[i], p, polygon[next]) == 0)
					return onSegment(polygon[i], p, polygon[next]);

				count++;
			}
			i = next;
		} while (i != 0);

		// Return true if count is odd, false otherwise
		return count & 1; // Same as (count%2 == 1)
	}

} /* namespace alica */

