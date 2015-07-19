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
	shared_ptr<VoronoiDiagram::Vertex> VoronoiNet::findClosestVertexToOwnPos(shared_ptr<geometry::CNPoint2D> pos)
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
	 * @param ownPos Point_2
	 * @param vertexPoint Point_2
	 */
	int VoronoiNet::calcDist(shared_ptr<geometry::CNPoint2D> ownPos, shared_ptr<geometry::CNPoint2D> vertexPoint)
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
				if (it->has_target() && find(neighbors.begin(), neighbors.end(), *it->target()) == neighbors.end())
				{
					neighbors.push_back(*it->target());
				}
			}
			if (it->has_target() && it->target()->point().x() == currentNode->getVertex()->x
					&& it->target()->point().y() == currentNode->getVertex()->y)
			{
				if (it->has_source() && find(neighbors.begin(), neighbors.end(), *it->source()) == neighbors.end())
				{
					neighbors.push_back(*it->source());
				}
			}
		}
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
	 * expands Nodes given in current node
	 * @param currentNode shared_ptr<SearchNode>
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
			cost += eval->eval(cost, startPos, goal, currentNode, neighbors.at(i), this);//calcDist(neighbors.at(i)->getVertex()->point(), goal);
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
	shared_ptr<VoronoiDiagram> VoronoiNet::generateVoronoiDiagram(vector<geometry::CNPoint2D> points)
	{
		lock_guard<mutex> lock(netMutex);
		vector<Site_2> sites;
		this->voronoi->clear();
		this->pointRobotKindMapping.clear();
		shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>> >> ownTeamMatesPositions = wm->robots.getPositionsOfTeamMates();
		bool alreadyIn = false;
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision();
		if (ownPos != nullptr)
		{
			sites.push_back(Site_2(ownPos->x, ownPos->y));
			pointRobotKindMapping.insert(
					pair<shared_ptr<geometry::CNPoint2D>, int>(make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y),
																wm->getOwnId()));
		}
		if (ownTeamMatesPositions != nullptr)
		{
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
		for (int i = 0; i < points.size(); i++)
		{
			for (int j = 0; j < sites.size(); j++)
			{
				//TODO check
				if (abs(sites.at(j).x() - points.at(i).x) < 250 && abs(sites.at(j).y() - points.at(i).y) < 250)
				{
					alreadyIn = true;
					break;
				}
			}
			if (!alreadyIn)
			{
				pointRobotKindMapping.insert(
						pair<shared_ptr<geometry::CNPoint2D>, int>(
								make_shared<geometry::CNPoint2D>(points.at(i).x, points.at(i).y), -1));
				Site_2 site(points.at(i).x, points.at(i).y);
				sites.push_back(site);
			}
			alreadyIn = false;
		}
		insertPoints(sites);
		auto artObs = wm->pathPlanner.getArtificialObstacles();
		for (int i = 0; i < artObs->size(); i++)
		{
			pointRobotKindMapping.insert(pair<shared_ptr<geometry::CNPoint2D>, int>(artObs->at(i), -2));
		}
		this->voronoi->insert(wm->pathPlanner.getArtificialObjectNet()->getVoronoi()->sites_begin(),
								wm->pathPlanner.getArtificialObjectNet()->getVoronoi()->sites_end());
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

	bool msl::VoronoiNet::isOwnCellEdge(geometry::CNPoint2D startPos, shared_ptr<SearchNode> currentNode,
										shared_ptr<SearchNode> nextNode)
	{
		VoronoiDiagram::Locate_result loc = this->voronoi->locate(Point_2(startPos.x, startPos.y));
		//boost::variant<Face_handle,Halfedge_handle,Vertex_handle>
		if (loc.which() == 0)
		{
			VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
			//removed opposite
			VoronoiDiagram::Halfedge_handle begin = handle->halfedge();
			VoronoiDiagram::Halfedge_handle edge = begin;
			do
			{
				if (edge->has_source() && edge->has_target()
						&& ((edge->source()->point().x() == currentNode->getVertex()->x
								&& edge->source()->point().y() == currentNode->getVertex()->y
								&& edge->target()->point().x() == nextNode->getVertex()->x
								&& edge->target()->point().y() == nextNode->getVertex()->y)
								|| (edge->source()->point().x() == nextNode->getVertex()->x
										&& edge->source()->point().y() == nextNode->getVertex()->y
										&& edge->target()->point().x() == currentNode->getVertex()->x
										&& edge->target()->point().y() == currentNode->getVertex()->y)))
				{
					return true;
				}
				edge = edge->previous();
			} while (edge != begin);
		}
		return false;
	}

	void msl::VoronoiNet::insertAdditionalPoints(shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > points)
	{
		lock_guard<mutex> lock(netMutex);
		vector<Site_2> sites;
		bool alreadyIn = false;
		for (auto iter = points->begin(); iter != points->end(); iter++)
		{
			for (auto it = pointRobotKindMapping.begin(); it != pointRobotKindMapping.end(); it++)
			{
				//TODO needs to be checked
				if (abs(it->first->x - (*iter)->x) < 250 && abs(it->first->y - (*iter)->y) < 250)
				{
					alreadyIn = true;
					break;
				}
			}
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

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > msl::VoronoiNet::getTeamMateVertices(int teamMateId)
	{
		shared_ptr<geometry::CNPosition> teamMatePos = wm->robots.getTeamMatePosition(teamMateId);
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > ret = this->getVerticesOfFace(
				make_shared<geometry::CNPoint2D>(teamMatePos->x, teamMatePos->y));
		return ret;

	}

	void msl::VoronoiNet::removeSites(shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > sites)
	{
		for (int i = 0; i < sites->size(); i++)
		{
			VoronoiDiagram::Point_2 point(sites->at(i).first->x, sites->at(i).first->y);
			VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
			if (loc.which() == 0)
			{
				VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
				((DelaunayTriangulation)this->voronoi->dual()).remove(handle->dual());
			}
		}
	}

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
	pair<shared_ptr<geometry::CNPoint2D>, shared_ptr<geometry::CNPoint2D>> VoronoiNet::getSitesNextToHalfEdge(
			shared_ptr<geometry::CNPoint2D> v1, shared_ptr<geometry::CNPoint2D> v2)
	{
		pair<shared_ptr<geometry::CNPoint2D>, shared_ptr<geometry::CNPoint2D>> ret;
		ret.first = nullptr;
		ret.second = nullptr;
		for (VoronoiDiagram::Face_iterator fit = this->voronoi->faces_begin(); fit != this->voronoi->faces_end(); ++fit)
		{
			bool foundFirst = false;
			bool foundSecond = false;
			//removed opposite
			VoronoiDiagram::Halfedge_handle begin = fit->halfedge();
			VoronoiDiagram::Halfedge_handle edge = begin;
			do
			{
				//TODO needs to be tested
				if (edge->has_source() && abs(edge->source()->point().x() - v1->x) < 50
						&& abs(edge->source()->point().y() - v1->y) < 50)
				{
					foundFirst = true;
				}
				if (edge->has_source() && abs(edge->source()->point().x() - v2->x) < 50
						&& abs(edge->source()->point().y() - v2->y) < 50)
				{
					foundSecond = true;
				}
				edge = edge->previous();
			} while (edge != begin);
			if (foundFirst && foundSecond)
			{
				if (ret.first == nullptr)
				{
					ret.first = make_shared<geometry::CNPoint2D>(fit->dual()->point().x(), fit->dual()->point().y());
					continue;
				}
				//TODO needs to be tested
				if (ret.second == nullptr && abs(ret.first->x - fit->dual()->point().x()) > 0.001
						&& abs(ret.first->y - fit->dual()->point().y()) > 0.001)
				{
					ret.second = make_shared<geometry::CNPoint2D>(fit->dual()->point().x(), fit->dual()->point().y());
					break;
				}
			}
		}
		return ret;
	}

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> VoronoiNet::getVerticesOfFace(shared_ptr<geometry::CNPoint2D> point)
	{
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		VoronoiDiagram::Point_2 p(point->x, point->y);
		VoronoiDiagram::Locate_result loc = this->voronoi->locate(p);
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
					ret->push_back(make_shared<geometry::CNPoint2D>(edge->source()->point().x(),edge->source()->point().y()));
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

	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > VoronoiNet::getTeamMatePositions()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
				vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
		for (auto iter = pointRobotKindMapping.begin(); iter != pointRobotKindMapping.end(); iter++)
		{
			if (iter->second > 0)
			{
				ret->push_back(*iter);
			}
		}
		return ret;
	}

	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > VoronoiNet::getObstaclePositions()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
				vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
		for (auto iter = pointRobotKindMapping.begin(); iter != pointRobotKindMapping.end(); iter++)
		{
			if (iter->second < 0)
			{
				ret->push_back(*iter);
			}
		}
		return ret;
	}

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

	void VoronoiNet::removeSites(shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > sites)
	{
		for (int i = 0; i < sites->size(); i++)
		{
			VoronoiDiagram::Point_2 point(sites->at(i)->x, sites->at(i)->y);
			VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
			if (loc.which() == 0)
			{
				VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
				((DelaunayTriangulation)this->voronoi->dual()).remove(handle->dual());
			}
		}
	}

	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockOppPenaltyArea()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();

		auto upLeftCorner = field->posULOppPenaltyArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upLeftCorner, -2));
		auto lowRightCorner = field->posLROppPenaltyArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowRightCorner, -2));
		int penaltyWidth = field->GoalAreaWidth;
		int penaltyLength = field->GoalAreaLength;
		auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upRightCorner, -2));
		auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowLeftCorner, -2));
		int pointCount = penaltyWidth / 500;
		double rest = penaltyWidth % 500;
		double pointDist = 500 + rest / 500;
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x + i * pointDist, lowRightCorner->y);
			auto temp2 = make_shared<geometry::CNPoint2D>(lowLeftCorner->x + i * pointDist, lowLeftCorner->y);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp2, -2));
		}
		pointCount = penaltyLength / 500;
		rest = penaltyLength % 500;
		pointDist = 500 + rest / 500;
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x, lowRightCorner->y + i * pointDist);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
		}
		insertAdditionalPoints(ret);
		return ret;
	}

	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockOppGoalArea()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();

		auto upLeftCorner = field->posULOppGoalArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upLeftCorner, 2));
		auto lowRightCorner = field->posLROppGoalArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowRightCorner, 2));
		int penaltyWidth = field->GoalInnerAreaWidth;
		int penaltyLength = field->GoalInnerAreaLength;
		auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upRightCorner, 2));
		auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowLeftCorner, 2));
		int pointCount = penaltyWidth / 500;
		double rest = penaltyWidth % 500;
		double pointDist = 500 + rest / 500;
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x + i * pointDist, lowRightCorner->y);
			auto temp2 = make_shared<geometry::CNPoint2D>(lowLeftCorner->x + i * pointDist, lowLeftCorner->y);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp2, -2));
		}
		pointCount = penaltyLength / 500;
		rest = penaltyLength % 500;
		pointDist = 500 + rest / 500;
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x, lowRightCorner->y + i * pointDist);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
		}
		insertAdditionalPoints(ret);
		return ret;
	}

	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockOwnPenaltyArea()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();

		auto upLeftCorner = field->posULOwnPenaltyArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upLeftCorner, 2));
		auto lowRightCorner = field->posLROwnPenaltyArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowRightCorner, 2));
		int penaltyWidth = field->GoalAreaWidth;
		int penaltyLength = field->GoalAreaLength;
		auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upRightCorner, 2));
		auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowLeftCorner, 2));
		int pointCount = penaltyWidth / 500;
		double rest = penaltyWidth % 500;
		double pointDist = 500 + rest / 500;
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(upRightCorner->x - i * pointDist, lowRightCorner->y);
			auto temp2 = make_shared<geometry::CNPoint2D>(upLeftCorner->x - i * pointDist, lowLeftCorner->y);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp2, -2));
		}
		pointCount = penaltyLength / 500;
		rest = penaltyLength % 500;
		pointDist = 500 + rest / 500;
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y + i * pointDist);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
		}
		insertAdditionalPoints(ret);
		return ret;
		return ret;
	}

	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockOwnGoalArea()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();

		auto upLeftCorner = field->posULOwnGoalArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upLeftCorner, 2));
		auto lowRightCorner = field->posLROwnGoalArea();
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowRightCorner, 2));
		int penaltyWidth = field->GoalInnerAreaWidth;
		int penaltyLength = field->GoalInnerAreaLength;
		auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upRightCorner, 2));
		auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowLeftCorner, 2));

		int pointCount = penaltyWidth / 500;
		double rest = penaltyWidth % 500;
		double pointDist = 500 + rest / 500;
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(upRightCorner->x - i * pointDist, lowRightCorner->y);
			auto temp2 = make_shared<geometry::CNPoint2D>(upLeftCorner->x - i * pointDist, lowLeftCorner->y);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp2, -2));
		}
		pointCount = penaltyLength / 500;
		rest = penaltyLength % 500;
		pointDist = 500 + rest / 500;
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(upRightCorner->x, lowRightCorner->y + i * pointDist);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
		}
		insertAdditionalPoints(ret);
		return ret; return ret;
	}

	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockThreeMeterAroundBall()
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
		auto alloBall = wm->ball.getAlloBallPosition();
		if(alloBall == nullptr)
		{
			return ret;
		}
		int radius = 3000;
		double perimeter = 2 * M_PI * radius;
		int pointCount = perimeter / 700 + 0.5;
		double slice = 2 * M_PI / pointCount;
		for (int i = 0; i < pointCount; i++)
		{
			double angle = slice * i;
			int newX = (int)(alloBall->x + radius * cos(angle));
			int newY = (int)(alloBall->y + radius * sin(angle));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(make_shared<geometry::CNPoint2D>
							(newX, newY), -2));
		}
		insertAdditionalPoints(ret);
		return ret;
	}

	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockCircle(
			shared_ptr<geometry::CNPoint2D> centerPoint, double radius)
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
		double perimeter = 2 * M_PI * radius;
		int pointCount = perimeter / 700 + 0.5;
		double slice = 2 * M_PI / pointCount;
		for (int i = 0; i < pointCount; i++)
		{
			double angle = slice * i;
			int newX = (int)(centerPoint->x + radius * cos(angle));
			int newY = (int)(centerPoint->y + radius * sin(angle));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(make_shared<geometry::CNPoint2D>
							(newX, newY), -2));
		}
		insertAdditionalPoints(ret);
		return ret;
	}

	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> VoronoiNet::blockRectangle(
			shared_ptr<geometry::CNPoint2D> upLeftCorner, shared_ptr<geometry::CNPoint2D> lowRightCorner)
	{
		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
		vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();

		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upLeftCorner, 2));
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowRightCorner, 2));
		auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(upRightCorner, 2));
		auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
		ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(lowLeftCorner, 2));
		int width = upLeftCorner->distanceTo(lowLeftCorner);
		int length = upLeftCorner->distanceTo(upRightCorner);

		int pointCount = width / 500;
		double rest = width % 500;
		double pointDist = 500 + rest / 500;
		for(int i = 1; i < pointCount; i++)
		{
			auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x - i * pointDist, lowRightCorner->y);
			auto temp2 = make_shared<geometry::CNPoint2D>(lowLeftCorner->x - i * pointDist, lowLeftCorner->y);
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp, -2));
			ret->push_back(pair<shared_ptr<geometry::CNPoint2D>, int>(temp2, -2));
		}
		pointCount = length / 500;
		rest = length % 500;
		pointDist = 500 + rest / 500;
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

	void VoronoiNet::insertAdditionalPoints(shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > points)
	{
		lock_guard<mutex> lock(netMutex);
		vector<Site_2> sites;
		bool alreadyIn = false;
		for (auto iter = points->begin(); iter != points->end(); iter++)
		{

			for (auto it = pointRobotKindMapping.begin(); it != pointRobotKindMapping.end(); it++)
			{
				//TODO needs to be checked
				if (abs(it->first->x - iter->first->x) < 250 && abs(it->first->y - iter->first->y) < 250)
				{
					alreadyIn = true;
					break;
				}
			}
			if (!alreadyIn)
			{
				pointRobotKindMapping.insert(pair<shared_ptr<geometry::CNPoint2D>, int>(iter->first, iter->second));
				Site_2 site(iter->first->x, iter->first->y);
				sites.push_back(site);
			}
			alreadyIn = false;
		}
		insertPoints(sites);
	}

}

/* namespace msl */

