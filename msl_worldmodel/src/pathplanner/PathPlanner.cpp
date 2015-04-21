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

	shared_ptr<vector<shared_ptr<VoronoiDiagram::Vertex>>> PathPlanner::aStarSearch(Point_2 ownPos)
	{
		shared_ptr<vector<shared_ptr<VoronoiDiagram::Vertex>>> ret = make_shared<vector<shared_ptr<VoronoiDiagram::Vertex>>>();
		ret->push_back(findClosestVertexToOwnPos(ownPos));

		return ret;
	}

	shared_ptr<VoronoiDiagram::Vertex> PathPlanner::findClosestVertexToOwnPos(Point_2 ownPos)
	{
		shared_ptr<VoronoiDiagram::Vertex> ret = nullptr;
		VoronoiDiagram::Vertex_iterator iter = voronoi.vertices_begin();
		int minDist = std::numeric_limits<int>::max();
		while(iter != voronoi.vertices_end())
		{
			if(ret == nullptr)
			{
				ret = make_shared<VoronoiDiagram::Vertex>(*iter);
				iter++;
			}
			else
			{
				int dist = calcDist(ownPos, iter->point());
				if(dist < minDist)
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
		int ret = sqrt(pow((vertexPoint.x()-ownPos.x()), 2) + pow((vertexPoint.y() - ownPos.y()), 2));
		return ret;
	}

} /* namespace alica */
