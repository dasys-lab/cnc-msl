/*
 * VoronoiNet.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: Stefan Jakob
 */

#include "pathplanner/VoronoiNet.h"
#include "MSLWorldModel.h"

namespace msl
{

	//TODO  c# vornoinet => initialize artficial obstacles
	VoronoiNet::VoronoiNet(MSLWorldModel* wm)
	{
		this->wm = wm;
		sc = supplementary::SystemConfig::getInstance();
		this->voronoi = make_shared<VoronoiDiagram>();
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
	shared_ptr<VoronoiDiagram::Vertex> VoronoiNet::findClosestVertexToOwnPos(shared_ptr<CNPoint2D> pos)
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
				int dist = calcDist(pos, make_shared<CNPoint2D>(iter->point().x(), iter->point().y()));
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
	int VoronoiNet::calcDist(shared_ptr<CNPoint2D> ownPos, shared_ptr<CNPoint2D> vertexPoint)
	{
		int ret = sqrt(pow((vertexPoint->x - ownPos->x), 2) + pow((vertexPoint->y - ownPos->y), 2));
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
			if (it->has_source() && it->source()->point().x() == currentNode->getVertex()->x
					&& it->source()->point().y() == currentNode->getVertex()->y)
			{
				if (find(neighbors.begin(), neighbors.end(), *it->target()) == neighbors.end())
				{
					if((*it).has_target())
					{
						neighbors.push_back(*it->target());
					}
				}
			}
			if (it->has_target() && it->target()->point().x() == currentNode->getVertex()->x
					&& it->target()->point().y() == currentNode->getVertex()->y)
			{
				if (find(neighbors.begin(), neighbors.end(), *it->source()) == neighbors.end())
				{
					if((*it).has_source())
					{
						neighbors.push_back(*it->source());
					}
				}
			}
		}
		vector<shared_ptr<SearchNode>> ret;
		for (int i = 0; i < neighbors.size(); i++)
		{
			ret.push_back(
					make_shared<SearchNode>(
							SearchNode(make_shared<CNPoint2D>(neighbors.at(i).point().x(), neighbors.at(i).point().y()), 0, nullptr)));
		}
		return ret;
	}

	/**
	 * expands Nodes given in current node
	 * @param currentNode shared_ptr<SearchNode>
	 */
	void VoronoiNet::expandNode(shared_ptr<SearchNode> currentNode, shared_ptr<vector<shared_ptr<SearchNode>>> open,
	shared_ptr<vector<shared_ptr<SearchNode>>> closed, CNPoint2D startPos, CNPoint2D goal, PathEvaluator* eval)
	{
		// get neighbored nodes
		vector<shared_ptr<SearchNode>> neighbors = getNeighboredVertices(currentNode);
		for(int i = 0; i < neighbors.size(); i++)
		{
			// if node is already closed skip it
			if(contains(closed, neighbors.at(i)))
			{
				continue;
			}
			//calculate cost with current cost and way to next vertex
			double cost = currentNode->getCost() + calcDist(currentNode->getVertex(), neighbors.at(i)->getVertex());
			// if node has still to be expaned but there is a cheaper way skip it
			if(contains(open, neighbors.at(i)) && cost >= neighbors.at(i)->getCost())
			{
				continue;
			}
			//set predecessor and cost
			neighbors.at(i)->setPredecessor(currentNode);
			// add heuristic cost
			cost += eval->eval(cost, startPos, goal, currentNode, neighbors.at(i));//calcDist(neighbors.at(i)->getVertex()->point(), goal);
			//if node is already in open change cost else add node
			if(contains(open, neighbors.at(i)))
			{
				for(int j = 0; j < open->size(); j++)
				{
					if(open->at(i)->getVertex()->x == neighbors.at(i)->getVertex()->x
					&& open->at(i)->getVertex()->y == neighbors.at(i)->getVertex()->y)
					{
						open->at(j)->setCost(cost);
						break;
					}
				}
			}
			else
			{
				neighbors.at(i)->setCost(cost);
				PathPlanner::insert(open, neighbors.at(i));
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
		lock_guard<mutex> lock(netMutex);
		vector<Site_2> sites;
		this->voronoi->clear();
		for (int i = 0; i < points.size(); i++)
		{
			Site_2 site(points.at(i).x, points.at(i).y);
			sites.push_back(site);
		}
		insertPoints(sites);
		this->voronoi->insert(wm->pathPlanner.getArtificialObjectNet()->getVoronoi()->sites_begin(), wm->pathPlanner.getArtificialObjectNet()->getVoronoi()->sites_end());
		return this->voronoi;
	}

	void msl::VoronoiNet::printSites()
	{
		cout << "Voronoi Diagram Sites: " << endl;
		for (VoronoiDiagram::Face_iterator it = this->voronoi->faces_begin(); it != this->voronoi->faces_end(); it++)
		{
			cout << it->dual()->point() << endl;
		}
	}

	void msl::VoronoiNet::printVertices()
	{
		cout << "Voronoi Diagram Vertices: " << endl;
		for (VoronoiDiagram::Vertex_iterator it = this->voronoi->vertices_begin(); it != this->voronoi->vertices_end();
				it++)
		{
			cout << it->point() << endl;
		}
	}

	string msl::VoronoiNet::toString()
	{
		stringstream ss;
		ss << "Voronoi Diagram Vertices: " << endl;
		for (VoronoiDiagram::Vertex_iterator it = this->voronoi->vertices_begin(); it != this->voronoi->vertices_end();
				it++)
		{
			ss << it->point() << endl;
		}
		ss << "Voronoi Diagram Sites: " << endl;
		for (VoronoiDiagram::Face_iterator it = this->voronoi->faces_begin(); it != this->voronoi->faces_end(); it++)
		{
			ss << it->dual()->point() << endl;
		}
		return ss.str();
	}

	bool msl::VoronoiNet::isOwnCellEdge(CNPoint2D startPos, shared_ptr<SearchNode> currentNode,
										shared_ptr<SearchNode> nextNode)
	{
		VoronoiDiagram::Locate_result loc = this->voronoi->locate(Point_2(startPos.x, startPos.y));
		//boost::variant<Face_handle,Halfedge_handle,Vertex_handle>
		if (loc.which() == 0)
		{
			VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
			VoronoiDiagram::Halfedge_handle begin = handle->halfedge()->opposite();
			VoronoiDiagram::Halfedge_handle edge = begin;
			do
			{
				if((edge->source()->point().x() == currentNode->getVertex()->x && edge->source()->point().y() == currentNode->getVertex()->y
						&& edge->target()->point().x() == nextNode->getVertex()->x && edge->target()->point().y() == nextNode->getVertex()->y)
						||
						(edge->source()->point().x() == nextNode->getVertex()->x && edge->source()->point().y() == nextNode->getVertex()->y
						&& edge->target()->point().x() == currentNode->getVertex()->x && edge->target()->point().y() == currentNode->getVertex()->y))
				{
					return true;
				}
				edge = edge->previous();
			} while (edge != begin);
		}
		return false;
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
			if (vector->at(i)->getVertex()->x == vertex->getVertex()->x
					&& vector->at(i)->getVertex()->y == vertex->getVertex()->y)
			{
				return true;
			}
		}
		return false;
	}

	/**
	 * return the sites near an egde defined by 2 points
	 * @param v1 VoronoiDiagram::Vertex
	 * @param v2 VoronoiDiagram::Vertex
	 * @returnpair<shared_ptr<Point_2>, shared_ptr<Point_2>>
	 */
	pair<shared_ptr<Point_2>, shared_ptr<Point_2>> VoronoiNet::getSitesNextToHalfEdge(
			shared_ptr<VoronoiDiagram::Vertex> v1, shared_ptr<VoronoiDiagram::Vertex> v2)
	{
		pair<shared_ptr<Site_2>, shared_ptr<Site_2>> ret;
		ret.first = nullptr;
		ret.second = nullptr;
		for (VoronoiDiagram::Face_iterator fit = this->voronoi->faces_begin(); fit != this->voronoi->faces_end(); ++fit)
		{
			bool foundFirst = false;
			bool foundSecond = false;
			VoronoiDiagram::Halfedge_handle begin = fit->halfedge()->opposite();
			VoronoiDiagram::Halfedge_handle edge = begin;
			do
			{
				if (edge->source()->point().x() == v1->point().x() && edge->source()->point().y() == v1->point().y())
				{
					foundFirst = true;
				}
				if (edge->source()->point().x() == v2->point().x() && edge->source()->point().y() == v2->point().y())
				{
					foundSecond = true;
				}
				edge = edge->previous();
			} while (edge != begin);
			if (ret.first == nullptr)
			{
				ret.first = make_shared<Point_2>(fit->dual()->point());
				continue;
			}
			if (ret.second == nullptr && ret.first->x() != fit->dual()->point().x()
					&& ret.first->y() != fit->dual()->point().y())
			{
				ret.second = make_shared<Point_2>(fit->dual()->point());
				break;
			}
		}
		return ret;
	}

	shared_ptr<vector<shared_ptr<CNPoint2D>>> VoronoiNet::getVerticesOfFace(VoronoiDiagram::Point_2 point)
	{
		shared_ptr<vector<shared_ptr<CNPoint2D>>> ret = make_shared<vector<shared_ptr<CNPoint2D>>>();
		VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
		//boost::variant<Face_handle,Halfedge_handle,Vertex_handle>
		if(loc.which() == 0)
		{
			VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
			VoronoiDiagram::Halfedge_handle begin = handle->halfedge();
			VoronoiDiagram::Halfedge_handle edge = begin;
			do
			{
				if(edge->has_source())
				{
					ret->push_back(make_shared<CNPoint2D>(edge->source()->point().x(),edge->source()->point().y()));
				}
				edge = edge->next();
			}while (edge != begin);
		}
		return ret;
	}

	shared_ptr<VoronoiDiagram> VoronoiNet::getVoronoi()
	{
		return voronoi;
	}
	void VoronoiNet::setVoronoi(shared_ptr<VoronoiDiagram> voronoi)
	{
		this->voronoi = voronoi;
	}

	shared_ptr<VoronoiDiagram::Site_2> VoronoiNet::getSiteOfFace(VoronoiDiagram::Point_2 point)
	{
		VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
		//boost::variant<Face_handle,Halfedge_handle,Vertex_handle>
		if (loc.which() == 0)
		{
			VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
			return make_shared<VoronoiDiagram::Site_2>(handle->dual()->point());
		}
		return nullptr;
	}
}
/* namespace msl */

