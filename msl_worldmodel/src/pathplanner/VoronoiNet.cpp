/*
 * VoronoiNet.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: Stefan Jakob
 */

#include "pathplanner/VoronoiNet.h"
#include "Ball.h"
#include "MSLWorldModel.h"
#include "Robots.h"
#include "obstaclehandler/Obstacles.h"
#include "pathplanner/PathPlanner.h"
#include <SystemConfig.h>

using std::vector;
using std::shared_ptr;
using std::pair;
using nonstd::optional;
using nonstd::nullopt;

namespace msl
{

VoronoiNet::VoronoiNet(MSLWorldModel *wm)
{
    this->wm = wm;
    this->sc = supplementary::SystemConfig::getInstance();
    this->voronoi = make_shared<VoronoiDiagram>();
    this->ownPosAvail = false;
    this->alloClusteredObsWithMe = make_shared<vector<CNRobotAllo>>();
    this->artificialObstacles = make_shared<vector<geometry::CNPointAllo>>();
    this->additionalObstacles = make_shared<vector<geometry::CNPointAllo>>();
}

VoronoiNet::VoronoiNet(shared_ptr<VoronoiNet> net)
{
    this->wm = net->wm;
    this->sc = net->sc;
    this->voronoi = make_shared<VoronoiDiagram>();
    this->ownPosAvail = net->ownPosAvail;

    this->alloClusteredObsWithMe = make_shared<vector<CNRobotAllo>>();
    for (auto cluster : *net->getAlloClusteredObsWithMe())
    {
        this->alloClusteredObsWithMe->push_back(cluster);
    }

    this->artificialObstacles = make_shared<vector<geometry::CNPointAllo>>();
    for (auto obs : *net->getArtificialObstacles())
    {
        this->artificialObstacles->push_back(obs);
    }

    this->additionalObstacles = make_shared<vector<geometry::CNPointAllo>>();
    for (auto obs : *net->getAdditionalObstacles())
    {
        this->additionalObstacles->push_back(obs);
    }

    // this must be last!!!!!!
    this->generateVoronoiDiagram(this->ownPosAvail);
}

VoronoiNet::~VoronoiNet()
{
}

int VoronoiNet::getTypeOfSite(Site_2 site) const
{
    auto result = this->pointRobotKindMapping.find(site);
    if (result != this->pointRobotKindMapping.end())
    {
        return result->second;
    }
    return EntityType::UndefinedEntity;
}

//	shared_ptr<VoronoiDiagram::Vertex> VoronoiNet::findClosestVertexToOwnPos(shared_ptr<geometry::CNPoint2D> pos)
//	{
//		shared_ptr<VoronoiDiagram::Vertex> ret = nullptr;
//		// get all vertices
//		VoronoiDiagram::Vertex_iterator iter = voronoi->vertices_begin();
//		int minDist = std::numeric_limits<int>::max();
//		//iterate over them and find closest
//		while (iter != voronoi->vertices_end())
//		{
//			//if there has been no closest so far
//			if (ret == nullptr)
//			{
//				ret = make_shared<VoronoiDiagram::Vertex>(*iter);
//				iter++;
//			}
//			else
//			{
//				//change if current vertex is closer
//				int dist = pos->distanceTo(make_shared<geometry::CNPoint2D>(iter->point().x(), iter->point().y()));
//				if (dist < minDist)
//				{
//					ret = make_shared<VoronoiDiagram::Vertex>(*iter);
//					minDist = dist;
//					iter++;
//				}
//			}
//		}
//		return ret;
//	}

//	shared_ptr<SearchNode> VoronoiNet::getMin(shared_ptr<vector<shared_ptr<SearchNode> > > open)
//	{
//		if (open->size() > 0)
//		{
//			sort(open->begin(), open->end(), SearchNode::compare);
//			return open->at(0);
//		}
//		else
//		{
//			return nullptr;
//		}
//
//	}

void VoronoiNet::generateVoronoiDiagram(bool ownPosAvail)
{
    lock_guard<mutex> lock(netMutex);
    // clear data
    this->clearVoronoiNet();

    this->ownPosAvail = ownPosAvail;

    // insert allo obstacles (including me) into voronoi diagram
    vector<Site_2> sites;
    this->alloClusteredObsWithMe = make_shared<vector<CNRobotAllo>>();

    auto alloObs = wm->obstacles->getClusteredObstaclesAlloWithMeBuffer().getLastValidContent();

    if (alloObs)
    {
        for (auto cluster : *(*alloObs))
        {
            Site_2 site(cluster.position.x, cluster.position.y);
            this->pointRobotKindMapping[site] = cluster.id;
            sites.push_back(site);
            this->alloClusteredObsWithMe->push_back(cluster);
        }
    }
    this->voronoi->insert(sites.begin(), sites.end());

    // insert artificial obstacles
    this->artificialObstacles = wm->pathPlanner->getArtificialFieldSurroundingObs();

    auto iter = wm->pathPlanner->getArtificialObjectNet()->getVoronoi()->sites_begin();
    do
    {

        this->pointRobotKindMapping[*iter] = EntityType::ArtificialObstacle;
    } while (++iter != wm->pathPlanner->getArtificialObjectNet()->getVoronoi()->sites_end());

    this->voronoi->insert(wm->pathPlanner->getArtificialObjectNet()->getVoronoi()->sites_begin(),
                          wm->pathPlanner->getArtificialObjectNet()->getVoronoi()->sites_end());

    //		cout << "VoronoiNet: obstWithMe " << alloClusteredObs->size() << " : sites " << sites.size() << " : artObs " << artObs->size() << endl;
}

//	bool VoronoiNet::isOwnCellEdge(shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<SearchNode> currentNode,
//									shared_ptr<SearchNode> nextNode)
//	{
//		//locate point
//		VoronoiDiagram::Locate_result loc = this->voronoi->locate(Point_2(startPos->x, startPos->y));
//		//if location == face
//		if (loc.which() == 0)
//		{
//			VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
//			//iterate over halfedges of face
//			VoronoiDiagram::Halfedge_handle begin = handle->halfedge();
//			VoronoiDiagram::Halfedge_handle edge = begin;
//			do
//			{
//				//finite edge
//				if (edge->has_source() && edge->has_target()
//						/* edge points fit*/
//						&& ((edge->source()->point().x() == currentNode->getVertex()->point().x()
//								&& edge->source()->point().y() == currentNode->getVertex()->point().y()
//								&& edge->target()->point().x() == nextNode->getVertex()->point().x()
//								&& edge->target()->point().y() == nextNode->getVertex()->point().y())
//								|| (edge->source()->point().x() == nextNode->getVertex()->point().x()
//										&& edge->source()->point().y() == nextNode->getVertex()->point().y()
//										&& edge->target()->point().x() == currentNode->getVertex()->point().x()
//										&& edge->target()->point().y() == currentNode->getVertex()->point().y())))
//				{
//					//part of own edge
//					return true;
//				}
//				//get next edge
//				edge = edge->previous();
//			} while (edge != begin);
//		}
//		return false;
//	}

void VoronoiNet::insertAdditionalPoints(const vector<geometry::CNPointAllo> &points, EntityType type)
{
    // TODO: does not delete points outside the field from param points like before (points was shared_ptr)
    lock_guard<mutex> lock(netMutex);
    vector<Site_2> sites;

    for (auto point : points)
    {
        if(this->wm->field->isInsideField(point))
        {
            Site_2 site(point.x, point.y);
            this->pointRobotKindMapping[site] = type;
            sites.push_back(site);

            if (type == EntityType::Obstacle)
                this->additionalObstacles->push_back(point);
            else if (type == EntityType::ArtificialObstacle)
                this->artificialObstacles->push_back(point);
        }
    }

    this->voronoi->insert(sites.begin(), sites.end());
}

//	shared_ptr<vector<shared_ptr<Vertex> > > VoronoiNet::getTeamMateVertices(int teamMateId)
//	{
//		//locate teammate
//		shared_ptr<geometry::CNPosition> teamMatePos = wm->robots.teammates.getTeamMatePosition(teamMateId);
//		//get vertices
//		shared_ptr<vector<shared_ptr<Vertex> > > ret = this->getVerticesOfFace(
//				make_shared<geometry::CNPoint2D>(teamMatePos->x, teamMatePos->y));
//		return ret;
//
//	}

shared_ptr<vector<geometry::CNPointAllo>> VoronoiNet::getTeamMateVerticesCNPoint2D(int teamMateId)
{
    // locate teammate
    auto teamMatePosInfo = wm->robots->teammates.getTeammatePositionBuffer(teamMateId).getLastValid();

    if (teamMatePosInfo == nullptr)
        return nullptr;

    auto teamMatePos = teamMatePosInfo->getInformation();

    // get vertices
    auto ret = make_shared<vector<geometry::CNPointAllo>>();
    auto vertices = this->getVerticesOfFace(teamMatePos.getPoint());
    for (int i = 0; i < vertices->size(); i++)
    {
        ret->push_back(geometry::CNPointAllo(vertices->at(i).point().x(), vertices->at(i).point().y()));
    }
    return ret;
}

void VoronoiNet::removeSites(shared_ptr<const vector<pair<geometry::CNPointAllo, int>>> sites)
{
    for (int i = 0; i < sites->size(); i++)
    {
        VoronoiDiagram::Point_2 point(sites->at(i).first.x, sites->at(i).first.y);
        // locate site
        VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
        if (loc.which() == 0)
        {
            // get handle for face
            VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
            // remove site from delaunay to remove face
            ((DelaunayTriangulation) this->voronoi->dual()).remove(handle->dual());
        }
    }
}

void VoronoiNet::removeSites(shared_ptr<const vector<geometry::CNPointAllo>> sites)
{
    for (int i = 0; i < sites->size(); i++)
    {
        // locate point
        VoronoiDiagram::Point_2 point(sites->at(i).x, sites->at(i).y);
        VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
        // if location is face
        if (VoronoiDiagram::Face_handle *handle = boost::get<VoronoiDiagram::Face_handle>(&loc))
        {
            // delete it from delaunay graph
            ((DelaunayTriangulation) this->voronoi->dual()).remove((*handle)->dual());
        }
    }
}


void VoronoiNet::clearVoronoiNet()
{
    this->voronoi->clear();
    this->pointRobotKindMapping.clear();
    this->alloClusteredObsWithMe->clear();
    this->artificialObstacles->clear();
    this->additionalObstacles->clear();
}

//	/**
//	 * return the sites near an egde defined by 2 points
//	 * @param v1 VoronoiDiagram::Vertex
//	 * @param v2 VoronoiDiagram::Vertex
//	 * @returnpair<shared_ptr<Point_2>, shared_ptr<Point_2>>
//	 */
//	pair<pair<shared_ptr<geometry::CNPoint2D>, int>, pair<shared_ptr<geometry::CNPoint2D>, int>> VoronoiNet::getSitesNextToHalfEdge(
//			shared_ptr<Vertex> v1, shared_ptr<Vertex> v2)
//	{
//		pair<pair<shared_ptr<geometry::CNPoint2D>, int>, pair<shared_ptr<geometry::CNPoint2D>, int>> ret;
//		ret.first.first = nullptr;
//		ret.second.first = nullptr;
//		//iterate over faces
//		bool foundFirst = false;
//		bool foundSecond = false;
//		for (VoronoiDiagram::Face_iterator fit = this->voronoi->faces_begin(); fit != this->voronoi->faces_end(); ++fit)
//		{
//			//iterate over halfedges
//			VoronoiDiagram::Halfedge_handle begin = fit->halfedge();
//			VoronoiDiagram::Halfedge_handle edge = begin;
//			do
//			{
//				//look for fitting halfedge with right source
//				if (edge->has_source() && abs(edge->source()->point().x() - v1->point().x()) < 10
//						&& abs(edge->source()->point().y() - v1->point().y()) < 10)
//				{
//					foundFirst = true;
//				}
//				if (edge->has_target() && abs(edge->target()->point().x() - v2->point().x()) < 10
//						&& abs(edge->target()->point().y() - v2->point().y()) < 10)
//				{
//					foundSecond = true;
//				}
//				if (foundFirst && foundSecond)
//				{
//					break;
//				}
//				edge = edge->previous();
//			} while (edge != begin);
//			foundFirst = false;
//			foundSecond = false;
//			//get face next to halfedge => get dual Point in delaunay
//			auto firstSite = edge->face()->dual()->point();
//			//get opposite halfedge => get face next to halfedge => get dual Point in delaunay
//			auto secondSite = edge->opposite()->face()->dual()->point();
//			for (auto current = pointRobotKindMapping.begin(); current != pointRobotKindMapping.end(); current++)
//			{
//				if (abs(current->first->x - firstSite.x()) < 0.01 && abs(current->first->y - firstSite.y()) < 0.01)
//				{
//					ret.first = *current;
//					foundFirst = true;
//					continue;
//				}
//				if (abs(current->first->x - secondSite.x()) < 0.01 && abs(current->first->y - secondSite.y()) < 0.01)
//				{
//					ret.second = *current;
//					foundSecond = true;
//					continue;
//				}
//				if (foundFirst && foundSecond)
//				{
//					return ret;
//				}
//			}
//		}
//		return ret;
//	}


shared_ptr<vector<Vertex>> VoronoiNet::getVerticesOfFace(geometry::CNPointAllo point) const
{
    auto ret = make_shared<vector<Vertex>>();
    // locate point
    VoronoiDiagram::Point_2 p(point.x, point.y);
    VoronoiDiagram::Locate_result loc = this->voronoi->locate(p);
    // if its a face
    if (loc.which() == 0)
    {
        VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
        // iterate over halfedges
        VoronoiDiagram::Halfedge_handle begin = handle->ccb();
        VoronoiDiagram::Halfedge_handle edge = begin;
        do
        {
            // if the edge has a sourcesave the vertex
            if (edge->has_source())
            {
                ret->push_back(VoronoiDiagram::Vertex(*(edge->source())));
            }
            edge = edge->next();
        } while (edge != begin);
    }
    return ret;
}


shared_ptr<VoronoiDiagram> VoronoiNet::getVoronoi()
{
    return voronoi;
}

shared_ptr<const VoronoiDiagram> VoronoiNet::getVoronoi() const
{
    return voronoi;
}

void VoronoiNet::setVoronoi(shared_ptr<VoronoiDiagram> voronoi)
{
    this->voronoi = voronoi;
}

optional<VoronoiDiagram::Site_2> VoronoiNet::getSiteOfFace(VoronoiDiagram::Point_2 point) const
{
    // locate point
    VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
    // result is face
    if (loc.which() == 0)
    {
        // get site from delaunay
        VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
        return VoronoiDiagram::Site_2(handle->dual()->point());
    }
    return nullopt;
}


//	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > VoronoiNet::getTeamMatePositions()
//	{
//		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
//				vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
//		for (auto iter = pointRobotKindMapping.begin(); iter != pointRobotKindMapping.end(); iter++)
//		{
//			//teammates have positive ids
//			if (iter->second > 0 && iter->second != SystemConfig::getOwnRobotID())
//			{
//				ret->push_back(*iter);
//			}
//		}
//		return ret;
//	}


shared_ptr<vector<geometry::CNPointAllo>> VoronoiNet::getObstaclePositions()
{
    auto ret = make_shared<vector<geometry::CNPointAllo>>();
    for (auto cluster : *this->alloClusteredObsWithMe)
    {
        if (cluster.id == EntityType::Obstacle)
        {
            ret->push_back(cluster.position.getPoint());
        }
    }
    return ret;
}

//	shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > VoronoiNet::getSitePositions()
//	{
//		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > ret = make_shared<
//				vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>();
//		for (auto iter = pointRobotKindMapping.begin(); iter != pointRobotKindMapping.end(); iter++)
//		{
//			ret->push_back(*iter);
//		}
//		return ret;
//	}

shared_ptr<vector<geometry::CNPointAllo>> VoronoiNet::getOpponentPositions()
{
    auto ret = make_shared<vector<geometry::CNPointAllo>>();
    for (auto cluster : *alloClusteredObsWithMe)
    {
        if (cluster.id == EntityType::Opponent)
        {
            ret->push_back(cluster.position.getPoint());
        }
    }
    return ret;
}

shared_ptr<const vector<geometry::CNPointAllo>> VoronoiNet::getArtificialObstacles() const
{
    return this->artificialObstacles;
}

shared_ptr<const vector<CNRobotAllo>> VoronoiNet::getAlloClusteredObsWithMe() const
{
    return this->alloClusteredObsWithMe;
}

shared_ptr<const vector<geometry::CNPointAllo>> VoronoiNet::getAdditionalObstacles() const
{
    return this->additionalObstacles;
}

void VoronoiNet::blockOppPenaltyArea()
{
    auto ret = vector<geometry::CNPointAllo>();

    // get cornerpoints of opp penalty area
    auto upLeftCorner = wm->field->posULOppPenaltyArea();
    ret.push_back(upLeftCorner);
    auto lowRightCorner = wm->field->posLROppPenaltyArea();
    ret.push_back(lowRightCorner);
    // get field length and width
    int penaltyWidth = (int)wm->field->getGoalAreaWidth();
    int penaltyLength = (int)wm->field->getGoalAreaLength();
    // calculate missing points
    auto upRightCorner = geometry::CNPointAllo(upLeftCorner.x, lowRightCorner.y);
    ret.push_back(upRightCorner);
    auto lowLeftCorner = geometry::CNPointAllo(lowRightCorner.x, upLeftCorner.y);
    ret.push_back(lowLeftCorner);
    // calculate point count to block space in width
    int pointCount = penaltyWidth / 500;
    double rest = penaltyWidth % 500;
    double pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = geometry::CNPointAllo(lowRightCorner.x + i * pointDist, lowRightCorner.y);
        auto temp2 = geometry::CNPointAllo(lowLeftCorner.x + i * pointDist, lowLeftCorner.y);
        ret.push_back(temp);
        ret.push_back(temp2);
    }
    // calculate point count to block space in length
    pointCount = penaltyLength / 500;
    rest = penaltyLength % 500;
    pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = geometry::CNPointAllo(lowRightCorner.x, lowRightCorner.y + i * pointDist);
        ret.push_back(temp);
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

void VoronoiNet::blockOppGoalArea()
{
    auto ret = vector<geometry::CNPointAllo>();

    // get cornerpoints of opp goal area
    auto upLeftCorner = wm->field->posULOppGoalArea();
    ret.push_back(upLeftCorner);
    auto lowRightCorner = wm->field->posLROppGoalArea();
    ret.push_back(lowRightCorner);
    // get field length and width
    int penaltyWidth = (int)wm->field->getPenaltyAreaWidth();
    int penaltyLength = (int)wm->field->getPenaltyAreaLength();
    // calculate missing points
    auto upRightCorner = geometry::CNPointAllo(upLeftCorner.x, lowRightCorner.y);
    ret.push_back(upRightCorner);
    auto lowLeftCorner = geometry::CNPointAllo(lowRightCorner.x, upLeftCorner.y);
    ret.push_back(lowLeftCorner);
    // calculate point count to block space in width
    int pointCount = penaltyWidth / 500;
    double rest = penaltyWidth % 500;
    double pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = geometry::CNPointAllo(lowRightCorner.x + i * pointDist, lowRightCorner.y);
        auto temp2 = geometry::CNPointAllo(lowLeftCorner.x + i * pointDist, lowLeftCorner.y);
        ret.push_back(temp);
        ret.push_back(temp2);
    }
    // calculate point count to block space in length
    pointCount = penaltyLength / 500;
    rest = penaltyLength % 500;
    pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = geometry::CNPointAllo(lowRightCorner.x, lowRightCorner.y + i * pointDist);
        ret.push_back(temp);
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

void VoronoiNet::blockOwnPenaltyArea()
{
    auto ret = vector<geometry::CNPointAllo>();

    // get cornerpoints of own penalty area
    auto upLeftCorner = wm->field->posULOwnPenaltyArea();
    ret.push_back(upLeftCorner);
    auto lowRightCorner = wm->field->posLROwnPenaltyArea();
    ret.push_back(lowRightCorner);
    // get field length and width
    int penaltyWidth = (int)wm->field->getGoalAreaWidth();
    int penaltyLength = (int)wm->field->getGoalAreaLength();
    // calculate missing points
    auto upRightCorner = geometry::CNPointAllo(upLeftCorner.x, lowRightCorner.y);
    ret.push_back(upRightCorner);
    auto lowLeftCorner = geometry::CNPointAllo(lowRightCorner.x, upLeftCorner.y);
    ret.push_back(lowLeftCorner);
    // calculate point count to block space in width
    int pointCount = penaltyWidth / 500;
    double rest = penaltyWidth % 500;
    double pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = geometry::CNPointAllo(upRightCorner.x - i * pointDist, lowRightCorner.y);
        auto temp2 = geometry::CNPointAllo(upLeftCorner.x - i * pointDist, lowLeftCorner.y);
        ret.push_back(temp);
        ret.push_back(temp2);
    }
    // calculate point count to block space in length
    pointCount = penaltyLength / 500;
    rest = penaltyLength % 500;
    pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = geometry::CNPointAllo(upLeftCorner.x, lowRightCorner.y + i * pointDist);
        ret.push_back(temp);
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

void VoronoiNet::blockOwnGoalArea()
{
    auto ret = vector<geometry::CNPointAllo>();

    // get cornerpoints of own goal area
    auto upLeftCorner = wm->field->posULOwnGoalArea();
    ret.push_back(upLeftCorner);
    auto lowRightCorner = wm->field->posLROwnGoalArea();
    ret.push_back(lowRightCorner);
    // get field length and width
    int penaltyWidth = (int)wm->field->getPenaltyAreaWidth();
    int penaltyLength = (int)wm->field->getPenaltyAreaLength();
    // calculate missing points
    auto upRightCorner = geometry::CNPointAllo(upLeftCorner.x, lowRightCorner.y);
    ret.push_back(upRightCorner);
    auto lowLeftCorner = geometry::CNPointAllo(lowRightCorner.x, upLeftCorner.y);
    ret.push_back(lowLeftCorner);
    // calculate point count to block space in width
    int pointCount = penaltyWidth / 500;
    double rest = penaltyWidth % 500;
    double pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = geometry::CNPointAllo(upRightCorner.x - i * pointDist, lowRightCorner.y);
        auto temp2 = geometry::CNPointAllo(upLeftCorner.x - i * pointDist, lowLeftCorner.y);
        ret.push_back(temp);
        ret.push_back(temp2);
    }
    // calculate point count to block space in length
    pointCount = penaltyLength / 500;
    rest = penaltyLength % 500;
    pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = geometry::CNPointAllo(upRightCorner.x, lowRightCorner.y + i * pointDist);
        ret.push_back(temp);
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

void VoronoiNet::blockThreeMeterAroundBall()
{
    auto ret = vector<geometry::CNPointAllo>();
    // get ball pos
    auto alloBall = wm->ball->getPositionAllo();
    if (alloBall == nullopt)
    {
        return;
    }
    // 3m radius
    int radius = 3000;
    // calculate perimeter
    double perimeter = 2 * M_PI * radius;
    // calculate point count on perimeter
    int pointCount = (int)ceil(perimeter / 500);
    // calculate agle between slices of the circle
    double slice = 2 * M_PI / pointCount;
    // calculate points to block area
    for (int i = 0; i < pointCount; i++)
    {
        // calculate point
        double angle = slice * i;
        int newX = (int)(alloBall->x + radius * cos(angle));
        int newY = (int)(alloBall->y + radius * sin(angle));
        ret.push_back(geometry::CNPointAllo(newX, newY));
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

void VoronoiNet::blockCircle(geometry::CNPointAllo centerPoint, double radius)
{
    auto ret = vector<geometry::CNPointAllo>();
    // calculate perimeter
    double perimeter = 2 * M_PI * radius;
    // calculate point count on perimeter
    int pointCount = (int)ceil(perimeter / 500);
    // calculate agle between slices of the circle
    double slice = 2 * M_PI / pointCount;
    // calculate points to block area
    cout << pointCount << endl;
    for (int i = 0; i < pointCount; i++)
    {
        // calculate point
        double angle = slice * i;
        int newX = (int)(centerPoint.x + radius * cos(angle));
        int newY = (int)(centerPoint.y + radius * sin(angle));
        ret.push_back(geometry::CNPointAllo(newX, newY));
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

void VoronoiNet::blockRectangle(geometry::CNPointAllo upLeftCorner, geometry::CNPointAllo lowRightCorner)
{
    auto ret = vector<geometry::CNPointAllo>();

    ret.push_back(upLeftCorner);
    ret.push_back(lowRightCorner);
    // calculate missing points
    auto upRightCorner = geometry::CNPointAllo(upLeftCorner.x, lowRightCorner.y);
    ret.push_back(upRightCorner);
    auto lowLeftCorner = geometry::CNPointAllo(lowRightCorner.x, upLeftCorner.y);
    ret.push_back(lowLeftCorner);
    int width = (int)upLeftCorner.distanceTo(lowLeftCorner);
    int length = (int)upLeftCorner.distanceTo(upRightCorner);
    // calculate point count to block space in width
    int pointCount = width / 500;
    double rest = width % 500;
    double pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = geometry::CNPointAllo(lowRightCorner.x - i * pointDist, lowRightCorner.y);
        auto temp2 = geometry::CNPointAllo(lowLeftCorner.x - i * pointDist, lowLeftCorner.y);
        ret.push_back(temp);
        ret.push_back(temp2);
    }
    // calculate point count to block space in length
    pointCount = length / 500;
    rest = length % 500;
    pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = geometry::CNPointAllo(lowLeftCorner.x, upLeftCorner.y + i * pointDist);
        auto temp2 = geometry::CNPointAllo(upLeftCorner.x, upRightCorner.y - i * pointDist);
        ret.push_back(temp);
        ret.push_back(temp2);
    }
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

void VoronoiNet::printSites()
{
    cout << "Voronoi Diagram Sites: " << endl;
    for (VoronoiDiagram::Face_iterator it = this->voronoi->faces_begin(); it != this->voronoi->faces_end(); it++)
    {
        cout << it->dual()->point() << endl;
    }
}

void VoronoiNet::printVertices()
{
    cout << "Voronoi Diagram Vertices: " << endl;
    for (VoronoiDiagram::Vertex_iterator it = this->voronoi->vertices_begin(); it != this->voronoi->vertices_end(); it++)
    {
        cout << it->point() << endl;
    }
}

string VoronoiNet::toString()
{
    stringstream ss;
    ss << "Voronoi Diagram Vertices: " << endl;
    for (VoronoiDiagram::Vertex_iterator it = this->voronoi->vertices_begin(); it != this->voronoi->vertices_end(); it++)
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

}/* namespace msl */
