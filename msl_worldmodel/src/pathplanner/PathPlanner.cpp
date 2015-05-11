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
			voronoiDiagrams(count)
	{
		this->wm = wm;
		sc = supplementary::SystemConfig::getInstance();
		for (int i = 0; i < count; i++)
		{
			this->voronoiDiagrams.at(i) = make_shared<VoronoiNet>(wm);
		}
		this->robotDiameter = (*this->sc)["Globals"]->get<double>("Globals", "Dimensions", "DiameterRobot", NULL);

	}

	PathPlanner::~PathPlanner()
	{
	}

	//TODO eigene position in voronoi einfügen
	//TODO gerade line zu ziel + korridor der breiter wird
	// anfrage mit start ziel voronoi wenn korridor fehlschlägt a stern
	/**
	 * aStar search on a VoronoiDiagram
	 * @param voronoi shared_ptr<VoronoiNet>
	 * @param ownPos Point_2
	 * @param goal Point_2
	 * @return shared_ptr<vector<shared_ptr<Point_2>>>
	 */
	shared_ptr<vector<shared_ptr<Point_2>>> PathPlanner::aStarSearch(shared_ptr<VoronoiNet> voronoi, Point_2 startPos, Point_2 goal)
	{
		// return
		shared_ptr<vector<shared_ptr<Point_2>>> ret = make_shared<vector<shared_ptr<Point_2>>>();
		// vector with open searchnodes
		shared_ptr<vector<shared_ptr<SearchNode>>> open = make_shared<vector<shared_ptr<SearchNode>>>();
		//vector with closed search nodes
		shared_ptr<vector<shared_ptr<SearchNode>>> closed = make_shared<vector<shared_ptr<SearchNode>>>();

		//get closest Vertex to ownPos => start point for a star serach
		shared_ptr<VoronoiDiagram::Vertex> closestVertexToOwnPos = voronoi->findClosestVertexToOwnPos(startPos);

		// get closest Vertex to goal => goal for a star serach
		shared_ptr<VoronoiDiagram::Vertex> closestVertexToGoal = voronoi->findClosestVertexToOwnPos(goal);

		// a star serach
		//TODO alle knoten in der nähe von ownpos einfügen
		open->push_back(make_shared<SearchNode>(SearchNode(closestVertexToOwnPos, 0, nullptr)));

		while(open->size() != 0)
		{
			//TODO einfügen in sortierte liste
			shared_ptr<SearchNode> currentNode = voronoi->getMin(open);

			//TODO == probieren
			//TODO ziel alle knoten der zelle mit goal und es gibt korridor zu goal
			if(currentNode->getVertex()->point().x() == closestVertexToGoal->point().x()
					&& currentNode->getVertex()->point().y() == closestVertexToGoal->point().y())
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
	 * aStar search on a VoronoiDiagram considering robot diameter and ballpossetion
	 * @param voronoi shared_ptr<VoronoiNet>
	 * @param ownPos Point_2
	 * @param goal Point_2
	 * @param haveBall bool
	 * @return shared_ptr<vector<shared_ptr<Point_2>>>
	 */
	shared_ptr<vector<shared_ptr<Point_2>>> PathPlanner::carefullAStarSearch(shared_ptr<VoronoiNet> voronoi, Point_2 ownPos, Point_2 goal, bool haveBall)
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

			voronoi->expandNodeCarefully(currentNode, open, closed, goal, this->robotDiameter, haveBall);
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
		for (int i = 0; i < msg->obstacles.size(); i++)
		{
			CNPoint2D point = CNPoint2D(msg->obstacles.at(i).x, msg->obstacles.at(i).y);
			points.push_back(point);
		}
		lock_guard<mutex> lock(voronoiMutex);
		for (int i = 0; i < voronoiDiagrams.size(); i++)
		{
			if (voronoiDiagrams.at(i)->getStatus() == VoronoiStatus::Latest)
			{
				voronoiDiagrams.at(i)->setStatus(VoronoiStatus::Old);
				break;
			}
		}
		for (int i = 0; voronoiDiagrams.size(); i++)
		{
			if (voronoiDiagrams.at(i)->getStatus() == VoronoiStatus::New
					|| voronoiDiagrams.at(i)->getStatus() == VoronoiStatus::Old)
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
		lock_guard<mutex> lock(this->voronoiMutex);
		for (int i = 0; i < this->voronoiDiagrams.size(); i++)
		{
			if (voronoiDiagrams.at(i)->getStatus() == VoronoiStatus::Latest)
			{
				return voronoiDiagrams.at(i);
			}
		}
		return nullptr;
	}

	//TODO wie ersten knoten anfahren
	//TODO vielleicht sinnvoll aus pfadplaner mgl. pathproxi
	//TODO verschiedene mgl zum anfahren des ersten punktes
	shared_ptr<CNPoint2D> PathPlanner::getEgoDirection(CNPoint2D egoTarget, bool stayInField)
	{
		lastPathTarget = egoTarget;
		shared_ptr<VoronoiNet> net = getCurrentVoronoiNet();
		shared_ptr<CNPoint2D> retPoint = nullptr;
		shared_ptr<CNPosition> ownPos = this->wm->rawSensorData.getOwnPositionVision();
		if (ownPos != nullptr)
		{
			shared_ptr<CNPoint2D> alloTarget = egoTarget.egoToAllo(*ownPos);
			shared_ptr<vector<shared_ptr<Point_2>>> path = carefullAStarSearch(this->getCurrentVoronoiNet(),
					Point_2(ownPos->x, ownPos->y)
					, Point_2(alloTarget->x, alloTarget->y), this->wm->ball.haveBall());
			if(path != nullptr)
			{
				retPoint = make_shared<CNPoint2D>(path->at(0)->x(), path->at(0)->y());
			}
		}

		return retPoint->alloToEgo(*ownPos);

	}

	//TODO pfad als cnpoint2d zurück

} /* namespace alica */
