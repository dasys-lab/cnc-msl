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

	PathPlanner::PathPlanner(MSLWorldModel* wm)
	{
		this->wm = wm;
		sc = SystemConfig::getInstance();
	}

	PathPlanner::~PathPlanner()
	{
	}

	VoronoiDiagram* PathPlanner::generateVoronoiDiagram()
	{
		return nullptr;
	}

	void PathPlanner::insertPoints(vector<Site_2> points)
	{
		voronoi.insert(points.begin(), points.end());
	}

	/**
	 * a star search
	 * @param ownPos own position Point_2
	 * @param goal position of goal Point_2
	 * @return shared_ptr<vector<shared_ptr<VoronoiDiagram::Point_2>>> path from ownPos to goal
	 */
	shared_ptr<vector<shared_ptr<Point_2>>> PathPlanner::aStarSearch(Point_2 ownPos, Point_2 goal)
	{
		// return
		shared_ptr<vector<shared_ptr<Point_2>>> ret = make_shared<vector<shared_ptr<Point_2>>>();
		// vector with open searchnodes
		shared_ptr<vector<shared_ptr<SearchNode>>> open = make_shared<vector<shared_ptr<SearchNode>>>();
		//vector with closed search nodes
		shared_ptr<vector<shared_ptr<SearchNode>>> closed = make_shared<vector<shared_ptr<SearchNode>>>();

		//get closest Vertex to ownPos => start point for a star serach
		shared_ptr<VoronoiDiagram::Vertex> closestVertexToOwnPos = findClosestVertexToOwnPos(ownPos);

		// get closest Vertex to goal => goal for a star serach
		shared_ptr<VoronoiDiagram::Vertex> closestVertexToGoal = findClosestVertexToOwnPos(goal);

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
			shared_ptr<SearchNode> currentNode = getMin(open);

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

			expandNode(currentNode, open, closed, goal);
		}

		// return nullptr if there is no way to goal
		return nullptr;
	}

	/**
	 * gets the closest Vertex to a given position
	 * @param pos position
	 */
	shared_ptr<VoronoiDiagram::Vertex> PathPlanner::findClosestVertexToOwnPos(Point_2 pos)
	{
		shared_ptr<VoronoiDiagram::Vertex> ret = nullptr;
		VoronoiDiagram::Vertex_iterator iter = voronoi.vertices_begin();
		int minDist = std::numeric_limits<int>::max();
		while (iter != voronoi.vertices_end())
		{
			if (ret == nullptr)
			{
				ret = make_shared<VoronoiDiagram::Vertex>(*iter);
				iter++;
			}
			else
			{
				int dist = calcDist(pos, iter->point());
				if (dist < minDist)
				{
					ret = make_shared<VoronoiDiagram::Vertex>(*iter);
					minDist = dist;
					iter++;
				}
			}
		}
		return ret;
	}

	int PathPlanner::calcDist(Point_2 ownPos, Point_2 vertexPoint)
	{
		int ret = sqrt(pow((vertexPoint.x() - ownPos.x()), 2) + pow((vertexPoint.y() - ownPos.y()), 2));
		return ret;
	}

	//TODO
	bool PathPlanner::checkGoal(shared_ptr<VoronoiDiagram::Vertex> vertex, Point_2 goal)
	{
		bool ret = false;

		return ret;
	}

	//TODO find way to get from face to vertices
	shared_ptr<vector<shared_ptr<VoronoiDiagram::Vertex> > > PathPlanner::getVerticesNearPoint(Point_2 point)
	{
		shared_ptr<vector<shared_ptr<VoronoiDiagram::Vertex>>> ret = make_shared<vector<shared_ptr<VoronoiDiagram::Vertex>>>();
		auto face = this->voronoi.locate(point);
		return ret;
	}

	/**
	 * gets SearchNode with minimal distance to goal
	 * @param open shared_ptr<vector<shared_ptr<SearchNode> > > vector with minimal searchnode
	 */
	shared_ptr<SearchNode> PathPlanner::getMin(shared_ptr<vector<shared_ptr<SearchNode> > > open)
	{
		if (open->size() > 0)
		{
			sort(open->begin(), open->end(), SearchNode::compare);
			return open->at(0);
		}
		else
		{
			return nullptr;
		}

	}

	vector<shared_ptr<SearchNode>> PathPlanner::getNeighboredVertices(shared_ptr<SearchNode> currentNode)
	{
		vector<VoronoiDiagram::Vertex> neighbors;
		for (VoronoiDiagram::Edge_iterator it = this->voronoi.edges_begin(); it != this->voronoi.edges_end(); it++)
		{
			if (it->source()->point().x() == currentNode->getVertex()->point().x()
					&& it->source()->point().y() == currentNode->getVertex()->point().y())
			{
				if (find(neighbors.begin(), neighbors.end(), *it->target()) == neighbors.end())
				{
					neighbors.push_back(*it->target());
				}
			}
		}
		vector<shared_ptr<SearchNode>> ret;
		for(int i = 0; i < neighbors.size(); i++)
		{
			ret.push_back(make_shared<SearchNode>(SearchNode(make_shared<VoronoiDiagram::Vertex>(neighbors.at(i)), 0, nullptr)));
		}
		return ret;
	}

	/**
	 * expands Nodes given in current node
	 * @param currentNode shared_ptr<SearchNode>
	 */
	void PathPlanner::expandNode(shared_ptr<SearchNode> currentNode, shared_ptr<vector<shared_ptr<SearchNode>>> open,
								 shared_ptr<vector<shared_ptr<SearchNode>>> closed, Point_2 goal)
	{
		vector<shared_ptr<SearchNode>> neighbors = getNeighboredVertices(currentNode);
		for(int i = 0; i < neighbors.size(); i++)
		{
			if(contains(closed, neighbors.at(i)))
			{
				continue;
			}
			double cost = currentNode->getCost() + calcDist(currentNode->getVertex()->point(), neighbors.at(i)->getVertex()->point());
			if(contains(open, neighbors.at(i)) && cost >= neighbors.at(i)->getCost())
			{
				continue;
			}
			neighbors.at(i)->setPredecessor(currentNode);
			neighbors.at(i)->setCost(cost);
			cost += calcDist(neighbors.at(i)->getVertex()->point(), goal);
			if(contains(open, neighbors.at(i)))
			{
				for(int j = 0; j < open->size(); j++)
				{
					if(open->at(i)->getVertex()->point().x() == neighbors.at(i)->getVertex()->point().x()
							&& open->at(i)->getVertex()->point().y() == neighbors.at(i)->getVertex()->point().y())
					{
						open->at(j)->setCost(cost);
					}
				}
			}
			else
			{
				open->push_back(neighbors.at(i));
			}
		}
	}

	bool PathPlanner::contains(shared_ptr<vector<shared_ptr<SearchNode> > > vector, shared_ptr<SearchNode> vertex)
	{
		for(int i = 0; i < vector->size(); i++)
		{
			if(vector->at(i)->getVertex()->point().x() == vertex->getVertex()->point().x()
					&& vector->at(i)->getVertex()->point().y() == vertex->getVertex()->point().y())
			{
				return true;
			}
		}
		return false;
	}

} /* namespace alica */
