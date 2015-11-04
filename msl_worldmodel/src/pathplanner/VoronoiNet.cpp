/*
 * VoronoiNet.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: Stefan Jakob
 */

#include "pathplanner/VoronoiNet.h"
#include "MSLWorldModel.h"
#include <SystemConfig.h>

namespace msl
{

	VoronoiNet::VoronoiNet(MSLWorldModel* wm)
	{
		this->wm = wm;
		sc = supplementary::SystemConfig::getInstance();
		this->voronoi = make_shared<VoronoiDiagram>();
		field = MSLFootballField::getInstance();
	}
	VoronoiNet::VoronoiNet(shared_ptr<VoronoiNet> net)
	{
		this->wm = net->wm;
		this->sc = net->sc;
		this->voronoi = make_shared<VoronoiDiagram>();
		this->field = net->field;
		auto opponents = net->getOpponentPositions();
		vector<shared_ptr<geometry::CNPoint2D>> points;
		for (int i = 0; i < opponents->size(); i++)
		{
			points.push_back(make_shared<geometry::CNPoint2D>(opponents->at(i).first->x, opponents->at(i).first->y));
		}
		this->generateVoronoiDiagram(points);
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
	 * gets the closest vertex to a given point
	 * @param ownPos shared_ptr<geometry::CNPoint2D>
	 * @return shared_ptr<VoronoiDiagram::Vertex>
	 */
	shared_ptr<VoronoiDiagram::Vertex> VoronoiNet::findClosestVertexToOwnPos(shared_ptr<geometry::CNPoint2D> pos)
	{
		shared_ptr<VoronoiDiagram::Vertex> ret = nullptr;
		// get all vertices
		VoronoiDiagram::Vertex_iterator iter = voronoi->vertices_begin();
		int minDist = std::numeric_limits<int>::max();
		//iterate over them and find closest
		while (iter != voronoi->vertices_end())
		{
			//if there has been no closest so far
			if (ret == nullptr)
			{
				ret = make_shared<VoronoiDiagram::Vertex>(*iter);
				iter++;
			}
			else
			{
				//change if current vertex is closer
				int dist = calcDist(pos, make_shared<geometry::CNPoint2D>(iter->point().x(), iter->point().y()));
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
	 * @param ownPos shared_ptr<geometry::CNPoint2D>
	 * @param vertexPoint shared_ptr<geometry::CNPoint2D>
	 * @return double
	 */
	double VoronoiNet::calcDist(shared_ptr<geometry::CNPoint2D> ownPos, shared_ptr<geometry::CNPoint2D> vertexPoint)
	{
		int ret = sqrt(pow((vertexPoint->x - ownPos->x), 2) + pow((vertexPoint->y - ownPos->y), 2));
		return ret;
	}

	/**
	 * gets the SearchNode with lowest dist to goal
	 * @param open shared_ptr<vector<shared_ptr<SearchNode>>>
	 * @return shared_ptr<SearchNode>
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
	 * TODO: check whether it is possible to iterate through edges by using forward-edge.next and twin-edge.previous!?
	 */
	vector<shared_ptr<SearchNode>> VoronoiNet::getNeighboredVertices(shared_ptr<SearchNode> currentNode)
	{
		vector<VoronoiDiagram::Vertex> neighbors;
		//iterate over edges
		for (VoronoiDiagram::Edge_iterator it = this->voronoi->edges_begin(); it != this->voronoi->edges_end(); it++)
		{
			//if there is a scource and ist fits to current node point
			if (it->has_source() && it->source()->point().x() == currentNode->getVertex()->x
					&& it->source()->point().y() == currentNode->getVertex()->y)
			{
				//if the edge has a traget and its not found already add it
				if (it->has_target() && find(neighbors.begin(), neighbors.end(), *it->target()) == neighbors.end())
				{
					neighbors.push_back(*it->target());
				}
			}
			//if there is a target and ist fits to current node point
			if (it->has_target() && it->target()->point().x() == currentNode->getVertex()->x
					&& it->target()->point().y() == currentNode->getVertex()->y)
			{
				//if the edge has a source and its not found already add it
				if (it->has_source() && find(neighbors.begin(), neighbors.end(), *it->source()) == neighbors.end())
				{
					neighbors.push_back(*it->source());
				}
			}
		}
		//convert them to geometry::CNPoint2D
		vector<shared_ptr<SearchNode>> ret;
		for (int i = 0; i < neighbors.size(); i++)
		{
			ret.push_back(
					make_shared<SearchNode>(
							SearchNode(
									make_shared<geometry::CNPoint2D>(neighbors.at(i).point().x(),
																		neighbors.at(i).point().y()),
									0, nullptr)));
		}
		return ret;
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
	void VoronoiNet::expandNode(shared_ptr<SearchNode> currentNode, shared_ptr<vector<shared_ptr<SearchNode>>> open,
	shared_ptr<vector<shared_ptr<SearchNode>>> closed, shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<PathEvaluator> eval)
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
			// if node has still to be expaned but there is a cheaper way skip it
			if(contains(open, neighbors.at(i)) /*&& cost >= neighbors.at(i)->getCost()*/)
			{
				continue;
				//calculate cost with current cost and way to next vertex
			}
			//set predecessor and cost
			// add heuristic cost
			double cost = eval->eval(startPos, goal, currentNode, neighbors.at(i), this, wm->pathPlanner.getLastPath(), wm->pathPlanner.getLastTarget());
			if(cost > 0)
			{
				//if node is already in open change cost else add node
//				if(contains(open, neighbors.at(i)))
//				{
//					for(int j = 0; j < open->size(); j++)
//					{
//						if(open->at(i)->getVertex()->x == neighbors.at(i)->getVertex()->x
//						&& open->at(i)->getVertex()->y == neighbors.at(i)->getVertex()->y)
//						{
//							open->at(j)->setCost(cost);
//							break;
//						}
//					}
//				}
//				else
//				{
				neighbors.at(i)->setPredecessor(currentNode);
				neighbors.at(i)->setCost(cost);
				PathPlanner::insert(open, neighbors.at(i));
//					cout << "node " << neighbors.at(i)->getVertex()->toString() << " " << neighbors.at(i)->getCost() << endl;
//				}
			}
		}
	}

	/**
	 * generates a VoronoiDiagram and inserts given points
	 * @param points vector<shared_ptr<geometry::CNPoint2D>>
	 * @return shared_ptr<VoronoiDiagram>
	 */
	shared_ptr<VoronoiDiagram> VoronoiNet::generateVoronoiDiagram(vector<shared_ptr<geometry::CNPoint2D>> points)
	{
		lock_guard<mutex> lock(netMutex);
		vector<Site_2> sites;
		//clear data
		this->voronoi->clear();
		this->pointRobotKindMapping.clear();
		// get teammate positions
		shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>> >> ownTeamMatesPositions = wm->robots.getPositionsOfTeamMates();
		bool alreadyIn = false;
		//get ownPos
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision();
		if (ownPos != nullptr)
		{
			//add own pos to voronoi
			sites.push_back(Site_2(ownPos->x, ownPos->y));
			pointRobotKindMapping.insert(
					pair<shared_ptr<geometry::CNPoint2D>, int>(make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y),
																wm->getOwnId()));
		}
		//if there are teammates add the positions
		if (ownTeamMatesPositions != nullptr)
		{
			//check if the position is already in if not insert
			for (auto iter = ownTeamMatesPositions->begin(); iter != ownTeamMatesPositions->end(); iter++)
			{
				if ((*iter)->first == wm->getOwnId())
				{
					continue;
				}
				pointRobotKindMapping.insert(
						pair<shared_ptr<geometry::CNPoint2D>, int>(
								make_shared<geometry::CNPoint2D>((*iter)->second->x, (*iter)->second->y),
								(*iter)->first));
				Site_2 site((*iter)->second->x, (*iter)->second->y);
				sites.push_back(site);
			}
		}
		//obstacle points
		for (int i = 0; i < points.size(); i++)
		{
			//check if the point is already in
			for (int j = 0; j < sites.size(); j++)
			{
				if (abs(sites.at(j).x() - points.at(i)->x) < 10 && abs(sites.at(j).y() - points.at(i)->y) < 10)
				{
					alreadyIn = true;
					break;
				}
			}
			//add if not in
			if (!alreadyIn)
			{
				pointRobotKindMapping.insert(
						pair<shared_ptr<geometry::CNPoint2D>, int>(
								make_shared<geometry::CNPoint2D>(points.at(i)->x, points.at(i)->y), -1));
				Site_2 site(points.at(i)->x, points.at(i)->y);
				sites.push_back(site);
			}
			alreadyIn = false;
		}
		//insert points into site
		insertPoints(sites);
		//insert artificial obstacles
		auto artObs = wm->pathPlanner.getArtificialObstacles();
		for (int i = 0; i < artObs->size(); i++)
		{
			pointRobotKindMapping.insert(pair<shared_ptr<geometry::CNPoint2D>, int>(artObs->at(i), -2));
		}
		this->voronoi->insert(wm->pathPlanner.getArtificialObjectNet()->getVoronoi()->sites_begin(),
								wm->pathPlanner.getArtificialObjectNet()->getVoronoi()->sites_end());
		return this->voronoi;
	}

	/**
	 * print the voronoi diagrams sites
	 */
	void msl::VoronoiNet::printSites()
	{
		cout << "Voronoi Diagram Sites: " << endl;
		for (VoronoiDiagram::Face_iterator it = this->voronoi->faces_begin(); it != this->voronoi->faces_end(); it++)
		{
			cout << it->dual()->point() << endl;
		}
	}

	/**
	 * print the voronoi diagrams vertices
	 */
	void msl::VoronoiNet::printVertices()
	{
		cout << "Voronoi Diagram Vertices: " << endl;
		for (VoronoiDiagram::Vertex_iterator it = this->voronoi->vertices_begin(); it != this->voronoi->vertices_end();
				it++)
		{
			cout << it->point() << endl;
		}
	}

	/**
	 * to string
	 */
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

	/**
	 * check if an edge belongs to face of given point
	 * @param pos shared_ptr<geometry::CNPoint2D>
	 * @param currentNode shared_ptr<SearchNode>
	 * @param nextNode shared_ptr<SearchNode>
	 * @return bool
	 */
	bool msl::VoronoiNet::isOwnCellEdge(shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<SearchNode> currentNode,
										shared_ptr<SearchNode> nextNode)
	{
		//locate point
		VoronoiDiagram::Locate_result loc = this->voronoi->locate(Point_2(startPos->x, startPos->y));
		//if location == face
		if (loc.which() == 0)
		{
			VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
			//iterate over halfedges of face
			VoronoiDiagram::Halfedge_handle begin = handle->halfedge();
			VoronoiDiagram::Halfedge_handle edge = begin;
			do
			{
				//finite edge
				if (edge->has_source() && edge->has_target()
						/* edge points fit*/
						&& ((edge->source()->point().x() == currentNode->getVertex()->x
								&& edge->source()->point().y() == currentNode->getVertex()->y
								&& edge->target()->point().x() == nextNode->getVertex()->x
								&& edge->target()->point().y() == nextNode->getVertex()->y)
								|| (edge->source()->point().x() == nextNode->getVertex()->x
										&& edge->source()->point().y() == nextNode->getVertex()->y
										&& edge->target()->point().x() == currentNode->getVertex()->x
										&& edge->target()->point().y() == currentNode->getVertex()->y)))
				{
					//part of own edge
					return true;
				}
				//get next edge
				edge = edge->previous();
			} while (edge != begin);
		}
		return false;
	}

	/**
	 * insert additional points into the voronoi diagram
	 * @param points shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
	 */
	void msl::VoronoiNet::insertAdditionalPoints(shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > points)
	{
		lock_guard<mutex> lock(netMutex);
		vector<Site_2> sites;
		bool alreadyIn = false;
		for (auto iter = points->begin(); iter != points->end(); iter++)
		{
			//check if point is already in
			for (auto it = pointRobotKindMapping.begin(); it != pointRobotKindMapping.end(); it++)
			{
				if (abs(it->first->x - (*iter)->x) < 10 && abs(it->first->y - (*iter)->y) < 10)
				{
					alreadyIn = true;
					break;
				}
			}
			//insert
			if (!alreadyIn)
			{
				pointRobotKindMapping.insert(pair<shared_ptr<geometry::CNPoint2D>, int>(*iter, -1));
				Site_2 site((*iter)->x, (*iter)->y);
				sites.push_back(site);
			}
			alreadyIn = false;
		}
		insertPoints(sites);
	}

	/**
	 * return vertices teammates voronoi face
	 * @param teamMateId int
	 * @return shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
	 */
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > msl::VoronoiNet::getTeamMateVertices(int teamMateId)
	{
		//locate teammate
		shared_ptr<geometry::CNPosition> teamMatePos = wm->robots.getTeamMatePosition(teamMateId);
		//get vertices
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > ret = this->getVerticesOfFace(
				make_shared<geometry::CNPoint2D>(teamMatePos->x, teamMatePos->y));
		return ret;

	}

	/**
	 * removes given sites from voronoi net
	 * @param sites shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
	 */
	void msl::VoronoiNet::removeSites(shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > sites)
	{
		for (int i = 0; i < sites->size(); i++)
		{
			VoronoiDiagram::Point_2 point(sites->at(i).first->x, sites->at(i).first->y);
			//locate site
			VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
			if (loc.which() == 0)
			{
				//get handle for face
				VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
				//remove site from delaunay to remove face
				((DelaunayTriangulation)this->voronoi->dual()).remove(handle->dual());
			}
		}
	}

	/**
	 * deletes sites from voronoi net and clears pointRobotKindMapping
	 */
	void msl::VoronoiNet::clearVoronoiNet()
	{
		this->getVoronoi()->clear();
		this->pointRobotKindMapping.clear();
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
			if (abs(vector->at(i)->getVertex()->x - vertex->getVertex()->x) < 10.0
					&& abs(vector->at(i)->getVertex()->y - vertex->getVertex()->y) < 10.0)
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
	pair<pair<shared_ptr<geometry::CNPoint2D>, int>, pair<shared_ptr<geometry::CNPoint2D>, int>> VoronoiNet::getSitesNextToHalfEdge(
			shared_ptr<geometry::CNPoint2D> v1, shared_ptr<geometry::CNPoint2D> v2)
	{
		pair<pair<shared_ptr<geometry::CNPoint2D>, int>, pair<shared_ptr<geometry::CNPoint2D>, int>> ret;
		ret.first.first = nullptr;
		ret.second.first = nullptr;
		//iterate over faces
		for (VoronoiDiagram::Face_iterator fit = this->voronoi->faces_begin(); fit != this->voronoi->faces_end(); ++fit)
		{
			bool foundFirst = false;
			bool foundSecond = false;
			//iterate over halfedges
			VoronoiDiagram::Halfedge_handle begin = fit->halfedge();
			VoronoiDiagram::Halfedge_handle edge = begin;
			do
			{
				//look for fitting halfedge with right source
				if (edge->has_source() && abs(edge->source()->point().x() - v1->x) < 10
						&& abs(edge->source()->point().y() - v1->y) < 10)
				{
					foundFirst = true;
				}
				if (edge->has_target() && abs(edge->target()->point().x() - v2->x) < 10
						&& abs(edge->target()->point().y() - v2->y) < 10)
				{
					foundSecond = true;
				}
				if(foundFirst && foundSecond)
				{
					break;
				}
				edge = edge->previous();
			} while (edge != begin);
			foundFirst = false;
			foundSecond = false;
			//get face next to halfedge => get dual Point in delaunay
			auto firstSite = edge->face()->dual()->point();
			//get opposite halfedge => get face next to halfedge => get dual Point in delaunay
			auto secondSite = edge->opposite()->face()->dual()->point();
			for (auto current = pointRobotKindMapping.begin(); current != pointRobotKindMapping.end(); current++)
			{
				if (abs(current->first->x - firstSite.x()) < 0.01
						&& abs(current->first->y - firstSite.y()) < 0.01)
				{
					ret.first = *current;
					foundFirst = true;
					continue;
				}
				if (abs(current->first->x - secondSite.x()) < 0.01
						&& abs(current->first->y - secondSite.y()) < 0.01)
				{
					ret.second = *current;
					foundSecond = true;
					continue;
				}
				if(foundFirst && foundSecond)
				{
					break;
				}
			}
//			//if both points are found insert them into ret
//			if (foundFirst && foundSecond)
//			{
//				if (ret.first.first == nullptr)
//				{

////					ret.first = make_shared<geometry::CNPoint2D>(fit->dual()->point().x(), fit->dual()->point().y());
////					continue;
//				}
//				if (ret.second.first == nullptr && abs(ret.first.first->x - fit->dual()->point().x()) > 0.001
//						&& abs(ret.first.first->y - fit->dual()->point().y()) > 0.001)
//				{
//					for(auto current = pointRobotKindMapping.begin(); current != pointRobotKindMapping.end(); current++)
//					{
//						if(abs(current->first->x - fit->dual()->point().x()) < 0.01 && abs(current->first->y - fit->dual()->point().y()) < 0.01)
//						{
//							ret.second = *current;
//							break;
//						}
//					}
////					ret.second = make_shared<geometry::CNPoint2D>(fit->dual()->point().x(), fit->dual()->point().y());
////
////					break;
//				}
//
//			}
		}
		return ret;
	}

	/**
	 * locates face of point and returns vertices
	 * @param point shared_ptr<geometry::CNPoint2D>
	 * @return shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
	 */
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> VoronoiNet::getVerticesOfFace(shared_ptr<geometry::CNPoint2D> point)
	{
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		//locate point
		VoronoiDiagram::Point_2 p(point->x, point->y);
		VoronoiDiagram::Locate_result loc = this->voronoi->locate(p);
		//if its a face
		if(loc.which() == 0)
		{
			VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
			//iterate over halfedges
			VoronoiDiagram::Halfedge_handle begin = handle->ccb();
			VoronoiDiagram::Halfedge_handle edge = begin;
			do
			{
				//if the edge has a sourcesave the vertex
				if(edge->has_source())
				{
					ret->push_back(make_shared<geometry::CNPoint2D>(edge->source()->point().x(),edge->source()->point().y()));
				}
				edge = edge->next();
			}while (edge != begin);
		}
		return ret;
	}

	/**
	 * gets the voronoi net
	 */
	shared_ptr<VoronoiDiagram> VoronoiNet::getVoronoi()
	{
		return voronoi;
	}

	/**
	 * sets the voronoi net
	 * @param voronoi shared_ptr<VoronoiDiagram>
	 */
	void VoronoiNet::setVoronoi(shared_ptr<VoronoiDiagram> voronoi)
	{
		this->voronoi = voronoi;
	}

	/**
	 * find the face in which the point is situated
	 * @param point VoronoiDiagram::Point_2
	 * @return shared_ptr<VoronoiDiagram::Site_2>
	 */
	shared_ptr<VoronoiDiagram::Site_2> VoronoiNet::getSiteOfFace(VoronoiDiagram::Point_2 point)
	{
		//locate point
		VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
		//result is face
		if (loc.which() == 0)
		{
			//get site from delaunay
			VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
			return make_shared<VoronoiDiagram::Site_2>(handle->dual()->point());
		}
		return nullptr;
	}

	/**
	 * return the teammate positions
	 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > >
	 */
	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > VoronoiNet::getTeamMatePositions()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
				vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
		for (auto iter = pointRobotKindMapping.begin(); iter != pointRobotKindMapping.end(); iter++)
		{
			//teammates have positive ids
			if (iter->second > 0)
			{
				ret->push_back(*iter);
			}
		}
		return ret;
	}

	/**
	 * return the obstacle positions
	 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > >
	 */
	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > VoronoiNet::getObstaclePositions()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
				vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
		for (auto iter = pointRobotKindMapping.begin(); iter != pointRobotKindMapping.end(); iter++)
		{
			//obstacles have negative ids
			if (iter->second < 0)
			{
				ret->push_back(*iter);
			}
		}
		return ret;
	}

	/**
	 * return the site positions
	 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > >
	 */
	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > VoronoiNet::getSitePositions()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
				vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
		for (auto iter = pointRobotKindMapping.begin(); iter != pointRobotKindMapping.end(); iter++)
		{
			ret->push_back(*iter);
		}
		return ret;
	}

	/**
	 * return the site positions
	 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > >
	 */
	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > VoronoiNet::getOpponentPositions()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
				vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
		for (auto iter = pointRobotKindMapping.begin(); iter != pointRobotKindMapping.end(); iter++)
		{
			if (iter->second == -1)
			{
				ret->push_back(*iter);
			}
		}
		return ret;
	}

	/**
	 * removes given sites from voronoi net
	 * @param sites shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
	 */
	void VoronoiNet::removeSites(shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > sites)
	{
		for (int i = 0; i < sites->size(); i++)
		{
			//locate point
			VoronoiDiagram::Point_2 point(sites->at(i)->x, sites->at(i)->y);
			VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
			//if location is face
			if (loc.which() == 0)
			{
				//delete if form delaunay
				VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
				((DelaunayTriangulation)this->voronoi->dual()).remove(handle->dual());
			}
		}
	}

	/**
	 * bolck opponent penalty area
	 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
	 */
	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockOppPenaltyArea()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();

		//get cornerpoints of opp penalty area
		auto upLeftCorner = field->posULOppPenaltyArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upLeftCorner, -2));
		auto lowRightCorner = field->posLROppPenaltyArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowRightCorner, -2));
		//get field length and width
		int penaltyWidth = field->GoalAreaWidth;
		int penaltyLength = field->GoalAreaLength;
		//calculate missing points
		auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upRightCorner, -2));
		auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowLeftCorner, -2));
		//calculate point count to block space in width
		int pointCount = penaltyWidth / 500;
		double rest = penaltyWidth % 500;
		double pointDist = 500 + rest / 500;
		//calculate points to block area
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x + i * pointDist, lowRightCorner->y);
			auto temp2 = make_shared<geometry::CNPoint2D>(lowLeftCorner->x + i * pointDist, lowLeftCorner->y);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp2, -2));
		}
		//calculate point count to block space in length
		pointCount = penaltyLength / 500;
		rest = penaltyLength % 500;
		pointDist = 500 + rest / 500;
		//calculate points to block area
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x, lowRightCorner->y + i * pointDist);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
		}
		//insert them into the voronoi diagram
		insertAdditionalPoints(ret);
		return ret;
	}

	/**
	 * bolck opponent goal area
	 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
	 */
	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockOppGoalArea()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();

		//get cornerpoints of opp goal area
		auto upLeftCorner = field->posULOppGoalArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upLeftCorner, 2));
		auto lowRightCorner = field->posLROppGoalArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowRightCorner, 2));
		//get field length and width
		int penaltyWidth = field->PenaltyAreaWidth;
		int penaltyLength = field->PenaltyAreaLength;
		//calculate missing points
		auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upRightCorner, 2));
		auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowLeftCorner, 2));
		//calculate point count to block space in width
		int pointCount = penaltyWidth / 500;
		double rest = penaltyWidth % 500;
		double pointDist = 500 + rest / 500;
		//calculate points to block area
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x + i * pointDist, lowRightCorner->y);
			auto temp2 = make_shared<geometry::CNPoint2D>(lowLeftCorner->x + i * pointDist, lowLeftCorner->y);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp2, -2));
		}
		//calculate point count to block space in length
		pointCount = penaltyLength / 500;
		rest = penaltyLength % 500;
		pointDist = 500 + rest / 500;
		//calculate points to block area
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x, lowRightCorner->y + i * pointDist);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
		}
		//insert them into the voronoi diagram
		insertAdditionalPoints(ret);
		return ret;
	}

	/**
	 * bolck own penalty area
	 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
	 */
	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockOwnPenaltyArea()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();

		//get cornerpoints of own penalty area
		auto upLeftCorner = field->posULOwnPenaltyArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upLeftCorner, 2));
		auto lowRightCorner = field->posLROwnPenaltyArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowRightCorner, 2));
		//get field length and width
		int penaltyWidth = field->GoalAreaWidth;
		int penaltyLength = field->GoalAreaLength;
		//calculate missing points
		auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upRightCorner, 2));
		auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowLeftCorner, 2));
		//calculate point count to block space in width
		int pointCount = penaltyWidth / 500;
		double rest = penaltyWidth % 500;
		double pointDist = 500 + rest / 500;
		//calculate points to block area
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(upRightCorner->x - i * pointDist, lowRightCorner->y);
			auto temp2 = make_shared<geometry::CNPoint2D>(upLeftCorner->x - i * pointDist, lowLeftCorner->y);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp2, -2));
		}
		//calculate point count to block space in length
		pointCount = penaltyLength / 500;
		rest = penaltyLength % 500;
		pointDist = 500 + rest / 500;
		//calculate points to block area
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y + i * pointDist);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
		}
		//insert them into the voronoi diagram
		insertAdditionalPoints(ret);
		return ret;
	}

	/**
	 * bolck own goal area
	 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
	 */
	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockOwnGoalArea()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();

		//get cornerpoints of own goal area
		auto upLeftCorner = field->posULOwnGoalArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upLeftCorner, 2));
		auto lowRightCorner = field->posLROwnGoalArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowRightCorner, 2));
		//get field length and width
		int penaltyWidth = field->PenaltyAreaWidth;
		int penaltyLength = field->PenaltyAreaLength;
		//calculate missing points
		auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upRightCorner, 2));
		auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowLeftCorner, 2));
		//calculate point count to block space in width
		int pointCount = penaltyWidth / 500;
		double rest = penaltyWidth % 500;
		double pointDist = 500 + rest / 500;
		//calculate points to block area
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(upRightCorner->x - i * pointDist, lowRightCorner->y);
			auto temp2 = make_shared<geometry::CNPoint2D>(upLeftCorner->x - i * pointDist, lowLeftCorner->y);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp2, -2));
		}
		//calculate point count to block space in length
		pointCount = penaltyLength / 500;
		rest = penaltyLength % 500;
		pointDist = 500 + rest / 500;
		//calculate points to block area
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(upRightCorner->x, lowRightCorner->y + i * pointDist);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
		}
		//insert them into the voronoi diagram
		insertAdditionalPoints(ret);
		return ret;
	}

	/**
	 * bolck 3 meters around the ball
	 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
	 */
	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockThreeMeterAroundBall()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
		//get ball pos
		auto alloBall = wm->ball.getAlloBallPosition();
		if(alloBall == nullptr)
		{
			return ret;
		}
		// 3m radius
		int radius = 3000;
		//calculate perimeter
		double perimeter = 2 * M_PI * radius;
		//calculate point count on perimeter
		int pointCount = perimeter / 500 + 0.5;
		//calculate agle between slices of the circle
		double slice = 2 * M_PI / pointCount;
		//calculate points to block area
		for (int i = 0; i < pointCount; i++)
		{
			//calculate point
			double angle = slice * i;
			int newX = (int)(alloBall->x + radius * cos(angle));
			int newY = (int)(alloBall->y + radius * sin(angle));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(make_shared<geometry::CNPoint2D>
							(newX, newY), -2));
		}
		//insert them into the voronoi diagram
		insertAdditionalPoints(ret);
		return ret;
	}

	/**
	 * bolck circle shaped area
	 * @param centerPoint shared_ptr<geometry::CNPoint2D>
	 * @param radious double
	 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
	 */
	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockCircle(
			shared_ptr<geometry::CNPoint2D> centerPoint, double radius)
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
		//calculate perimeter
		double perimeter = 2 * M_PI * radius;
		//calculate point count on perimeter
		int pointCount = perimeter / 500 + 0.5;
		//calculate agle between slices of the circle
		double slice = 2 * M_PI / pointCount;
		//calculate points to block area
		cout << pointCount << endl;
		for (int i = 0; i < pointCount; i++)
		{
			//calculate point
			double angle = slice * i;
			int newX = (int)(centerPoint->x + radius * cos(angle));
			int newY = (int)(centerPoint->y + radius * sin(angle));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(make_shared<geometry::CNPoint2D>
							(newX, newY), -2));
		}
		//insert them into the voronoi diagram
		insertAdditionalPoints(ret);
		return ret;
	}

	/**
	 * bolck opponent penalty area
	 * @param upLeftCorner shared_ptr<geometry::CNPoint2D>
	 * @param lowRightCorner shared_ptr<geometry::CNPoint2D>
	 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
	 */
	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockRectangle(
			shared_ptr<geometry::CNPoint2D> upLeftCorner, shared_ptr<geometry::CNPoint2D> lowRightCorner)
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();

		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upLeftCorner, 2));
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowRightCorner, 2));
		//calculate missing points
		auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upRightCorner, 2));
		auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowLeftCorner, 2));
		int width = upLeftCorner->distanceTo(lowLeftCorner);
		int length = upLeftCorner->distanceTo(upRightCorner);
		//calculate point count to block space in width
		int pointCount = width / 500;
		double rest = width % 500;
		double pointDist = 500 + rest / 500;
		//calculate points to block area
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x - i * pointDist, lowRightCorner->y);
			auto temp2 = make_shared<geometry::CNPoint2D>(lowLeftCorner->x - i * pointDist, lowLeftCorner->y);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp2, -2));
		}
		//calculate point count to block space in length
		pointCount = length / 500;
		rest = length % 500;
		pointDist = 500 + rest / 500;
		//calculate points to block area
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(lowLeftCorner->x, upLeftCorner->y + i * pointDist);
			auto temp2 = make_shared<geometry::CNPoint2D>(upLeftCorner->x, upRightCorner->y - i * pointDist);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp2, -2));
		}
		insertAdditionalPoints(ret);
		return ret;
	}

	/**
	 * insert additional points into the voronoi diagram
	 * @param points shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
	 */
	void VoronoiNet::insertAdditionalPoints(shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > points)
	{
		lock_guard<mutex> lock(netMutex);
		vector<Site_2> sites;
		bool alreadyIn = false;
		for (auto iter = points->begin(); iter != points->end(); iter++)
		{
			//check if point is already in
			for (auto it = pointRobotKindMapping.begin(); it != pointRobotKindMapping.end(); it++)
			{
				if (abs(it->first->x - iter->first->x) < 10 && abs(it->first->y - iter->first->y) < 10)
				{
					alreadyIn = true;
					break;
				}
			}
			//if not insert
			if (!alreadyIn)
			{
				pointRobotKindMapping.insert(pair<shared_ptr<geometry::CNPoint2D>, int>(iter->first, iter->second));
				Site_2 site(iter->first->x, iter->first->y);
				sites.push_back(site);
			}
			alreadyIn = false;
		}
		//insert points
		insertPoints(sites);
	}

}

/* namespace msl */

