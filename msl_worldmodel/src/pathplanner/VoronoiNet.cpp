/*
 * VoronoiNet.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: Stefan Jakob
 */

#include "pathplanner/VoronoiNet.h"

namespace msl
{

	VoronoiNet::VoronoiNet(MSLWorldModel* wm)
	{
		this->wm = wm;
		sc = supplementary::SystemConfig::getInstance();
		status = VoronoiStatus::New;

	}

	VoronoiNet::~VoronoiNet()
	{
	}

	/**
	 * inserts sites into the VoronoiDiagram
	 * @param points vector<Site_2>
	 */
	void VoronoiNet::insertPoints(vector<Site_2> points)
	{
		this->voronoi->insert(points.begin(), points.end());
	}

	/**
	 * gets the closest Vertex to a given position
	 * @param pos position
	 */
	shared_ptr<VoronoiDiagram::Vertex> VoronoiNet::findClosestVertexToOwnPos(Point_2 pos)
	{
		shared_ptr<VoronoiDiagram::Vertex> ret = nullptr;
		VoronoiDiagram::Vertex_iterator iter = voronoi->vertices_begin();
		int minDist = std::numeric_limits<int>::max();
		while (iter != voronoi->vertices_end())
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

	/**
	 * calculates distance between two points
	 * @param ownPos Point_2
	 * @param vertexPoint Point_2
	 */
	int VoronoiNet::calcDist(Point_2 ownPos, Point_2 vertexPoint)
	{
		int ret = sqrt(pow((vertexPoint.x() - ownPos.x()), 2) + pow((vertexPoint.y() - ownPos.y()), 2));
		return ret;
	}

	/**
	 * gets SearchNode with minimal distance to goal
	 * @param open shared_ptr<vector<shared_ptr<SearchNode> > > vector with minimal searchnode
	 */
	shared_ptr<SearchNode> VoronoiNet::getMin(shared_ptr<vector<shared_ptr<SearchNode> > > open)
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

	/**
	 * gets Vertices connected to SeachNode vertex
	 * @param currentNode shared_ptr<SearchNode>
	 * @return vector<shared_ptr<SearchNode>>
	 */
	vector<shared_ptr<SearchNode>> VoronoiNet::getNeighboredVertices(shared_ptr<SearchNode> currentNode)
	{
		vector<VoronoiDiagram::Vertex> neighbors;
		for (VoronoiDiagram::Edge_iterator it = this->voronoi->edges_begin(); it != this->voronoi->edges_end(); it++)
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
		for (int i = 0; i < neighbors.size(); i++)
		{
			ret.push_back(
					make_shared<SearchNode>(
							SearchNode(make_shared<VoronoiDiagram::Vertex>(neighbors.at(i)), 0, nullptr)));
		}
		return ret;
	}

	/**
	 * expands Nodes given in current node
	 * @param currentNode shared_ptr<SearchNode>
	 */
	void VoronoiNet::expandNode(shared_ptr<SearchNode> currentNode, shared_ptr<vector<shared_ptr<SearchNode>>> open,
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

	/**
	 * generates a VoronoiDiagram and inserts given points
	 * @param points vector<CNPoint2D>
	 * @return shared_ptr<VoronoiDiagram>
	 */
	shared_ptr<VoronoiDiagram> VoronoiNet::generateVoronoiDiagram(vector<CNPoint2D> points)
	{
		vector<Site_2> sites;
		for(int i = 0; i < points.size(); i++)
		{
			Site_2 site = Site_2(points.at(i).x,points.at(i).y);
			sites.push_back(site);
		}
		insertPoints(sites);
		return this->voronoi;
	}

	/**
	 * checks if a SearchNode is part of a vector
	 * @param vector shared_ptr<vector<shared_ptr<SearchNode> > >
	 * @param vertex shared_ptr<SearchNode>
	 * @return bool
	 */
	bool VoronoiNet::contains(shared_ptr<vector<shared_ptr<SearchNode> > > vector, shared_ptr<SearchNode> vertex)
	{
		for (int i = 0; i < vector->size(); i++)
		{
			if (vector->at(i)->getVertex()->point().x() == vertex->getVertex()->point().x()
					&& vector->at(i)->getVertex()->point().y() == vertex->getVertex()->point().y())
			{
				return true;
			}
		}
		return false;
	}

	/**
	 * gets the status of the VoronoiDiagram
	 * @return VoronoiStatus
	 */
	VoronoiStatus VoronoiNet::getStatus()
	{
		return status;
	}

	/**
	 * sets the status of the VoronoiDiagram
	 * @param status VoronoiStatus
	 */
	void VoronoiNet::setStatus(VoronoiStatus status)
	{
		this->status = status;
	}

} /* namespace msl */
