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

	PathPlanner::PathPlanner(MSLWorldModel* wm, int count) : voronoiDiagrams(count)
	{
		this->wm = wm;
		sc = supplementary::SystemConfig::getInstance();
		for(int i = 0; i < count; i++)
		{
			this->voronoiDiagrams.at(i) = make_shared<VoronoiNet>(wm);
		}

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
	shared_ptr<vector<shared_ptr<Point_2>>> PathPlanner::aStarSearch(shared_ptr<VoronoiNet>  voronoi, Point_2 ownPos, Point_2 goal)
	{
		// return
		shared_ptr<vector<shared_ptr<Point_2>>> ret = make_shared<vector<shared_ptr<Point_2>>>();
		// vector with open searchnodes
		shared_ptr<vector<shared_ptr<SearchNode>>> open = make_shared<vector<shared_ptr<SearchNode>>>();
		//vector with closed search nodes
		shared_ptr<vector<shared_ptr<SearchNode>>> closed = make_shared<vector<shared_ptr<SearchNode>>>();

		//get closest Vertex to ownPos => start point for a star serach
		shared_ptr<VoronoiDiagram::Vertex> closestVertexToOwnPos = voronoi->findClosestVertexToOwnPos(ownPos);

		// get closest Vertex to goal => goal for a star serach
		shared_ptr<VoronoiDiagram::Vertex> closestVertexToGoal = voronoi->findClosestVertexToOwnPos(goal);

		//if ownPos == goal we dont have to plan a route
		if(ownPos.x() == goal.x() && ownPos.y() == goal.y())
		{
			shared_ptr<vector<shared_ptr<Point_2>>> temp = make_shared<vector<shared_ptr<Point_2>>>();
			temp->push_back(make_shared<Point_2>(goal));
			return temp;
		}

		// a star serach

		open->push_back(make_shared<SearchNode>(SearchNode(closestVertexToOwnPos, 0, nullptr)));

		while(open->size() != 0)
		{
			shared_ptr<SearchNode> currentNode = voronoi->getMin(open);

			if(currentNode->getVertex()->point().x() == closestVertexToGoal->point().x()
					&& currentNode->getVertex()->point().x() == closestVertexToGoal->point().x())
			{
				shared_ptr<SearchNode> temp = currentNode;
				ret->push_back(make_shared<VoronoiDiagram::Point_2>(currentNode->getVertex()->point()));
				while(temp->getPredecessor() != nullptr)
				{
					ret->push_back(make_shared<VoronoiDiagram::Point_2>(temp->getPredecessor()->getVertex()->point()));
					temp = temp->getPredecessor();
				}
				reverse(ret->begin(), ret->end());
				return ret;
			}
			closed->push_back(currentNode);

			voronoi->expandNode(currentNode, open, closed, goal);
		}

		// return nullptr if there is no way to goal
		return nullptr;
	}

	/**
	 * processes the WorldModel msg
	 * @param msg msl_sensor_msgs::WorldModelDataPtr
	 */
	void PathPlanner::processWolrdModelData(msl_sensor_msgs::WorldModelDataPtr msg)
	{
		vector<CNPoint2D> points;
		for(int i = 0; i < msg->obstacles.size(); i++)
		{
			CNPoint2D point = CNPoint2D(msg->obstacles.at(i).x,msg->obstacles.at(i).y);
			points.push_back(point);
		}
		lock_guard<mutex> lock(voronoiMutex);
		for(int i = 0; i < voronoiDiagrams.size(); i++)
		{
			if(voronoiDiagrams.at(i)->getStatus() == VoronoiStatus::Latest)
			{
				voronoiDiagrams.at(i)->setStatus(VoronoiStatus::Old);
				break;
			}
		}
		for(int i = 0; voronoiDiagrams.size(); i++)
		{
			if(voronoiDiagrams.at(i)->getStatus() == VoronoiStatus::New ||
					voronoiDiagrams.at(i)->getStatus() == VoronoiStatus::Old)
			{
				voronoiDiagrams.at(i)->generateVoronoiDiagram(points);
				voronoiDiagrams.at(i)->setStatus(VoronoiStatus::Latest);
				break;
			}
		}

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
		for(int i = 0; i < this->voronoiDiagrams.size(); i++)
		{
			if(voronoiDiagrams.at(i)->getStatus() == VoronoiStatus::Latest)
			{
				return voronoiDiagrams.at(i);
			}
		}
		return nullptr;
	}

} /* namespace alica */
