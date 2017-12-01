/*
 * PathProxy.cpp
 *
 *  Created on: May 17, 2015
 *      Author: Stefan Jakob
 */

#include "pathplanner/PathProxy.h"
#include "RawSensorData.h"
#include "msl_msgs/PathPlanner.h"
#include "msl_msgs/VoronoiNetInfo.h"
#include "pathplanner/PathPlanner.h"
#include "pathplanner/PathPlannerQuery.h"
#include "pathplanner/VoronoiNet.h"

using nonstd::nullopt;
using std::shared_ptr;
using std::make_shared;

namespace msl
{

/**
 * struct for generating a cropped voronoi from a delaunay
 */
struct Cropped_voronoi_from_delaunay
{
    std::list<Segment_2> m_cropped_vd;
    Iso_rectangle_2 m_bbox;

    Cropped_voronoi_from_delaunay(const Iso_rectangle_2 &bbox)
        : m_bbox(bbox)
    {
    }

    template <class RSL>
    void crop_and_extract_segment(const RSL &rsl)
    {
        CGAL::Object obj = CGAL::intersection(rsl, m_bbox);
        const Segment_2 *s = CGAL::object_cast<Segment_2>(&obj);
        if (s)
            m_cropped_vd.push_back(*s);
    }

    void operator<<(const Ray_2 &ray)
    {
        crop_and_extract_segment(ray);
    }
    void operator<<(const Line_2 &line)
    {
        crop_and_extract_segment(line);
    }
    void operator<<(const Segment_2 &seg)
    {
        crop_and_extract_segment(seg);
    }
};

PathProxy::PathProxy()
{
    this->wm = MSLWorldModel::get();
    pathPub = n.advertise<msl_msgs::PathPlanner>("/PathPlanner/PathPlanner", 10);
    voroniPub = n.advertise<msl_msgs::VoronoiNetInfo>("/PathPlanner/VoronoiNet", 10);
    sc = supplementary::SystemConfig::getInstance();
    this->pathPlannerDebug = (*this->sc)["PathPlanner"]->get<bool>("PathPlanner", "pathPlannerDebug", NULL);
}

PathProxy::~PathProxy()
{
}

nonstd::optional<geometry::CNPointEgo>
PathProxy::getEgoDirection(geometry::CNPointEgo egoTarget, const IPathEvaluator &pathEvaluator,
                           shared_ptr<const vector<geometry::CNPointAllo>> additionalPoints)
{
    // save target
    lastPathTarget = egoTarget;

    // get voronoi diagram, which is ready to be used
    auto net = this->wm->pathPlanner->getCurrentVoronoiNet();

    auto ownPosInfo = this->wm->rawSensorData->getOwnPositionVisionBuffer().getLastValid();
    if (net == nullptr || !net->ownPosAvail || ownPosInfo == nullptr)
    {
        // We are not localized (or have no vNet), so we can't plan a path.
        return nullopt;
    }
    auto ownPos = ownPosInfo->getInformation();
    // if there are additional points insert them into the voronoi diagram
    if (additionalPoints != nullptr)
    {
        net->insertAdditionalPoints(*additionalPoints, EntityType::Obstacle);
    }

    // plan
    nonstd::optional<geometry::CNPointAllo> alloRetPoint;
    auto alloTarget = egoTarget.toAllo(ownPos);
    auto path = this->wm->pathPlanner->plan(net, ownPos.getPoint(), alloTarget, pathEvaluator);

    if (path != nullptr && path->size() > 0)
    {
        // get first point of returned path
        if (path->size() < 3)
        {
            alloRetPoint = nonstd::make_optional<geometry::CNPointAllo>(path->at(0));
        }
        else
        {
            path = applyShortcut(*path, ownPos, *net);
            alloRetPoint = nonstd::make_optional<geometry::CNPointAllo>(path->at(0));
        }
        // send debug msgs
        if (pathPlannerDebug)
        {
            path->insert(path->begin(), ownPos.getPoint());
            sendPathPlannerMsg(*path);
            sendVoronoiNetMsg(*net);
        }
    }

    // remove additional points from voronoi diagram
    if (additionalPoints != nullptr)
    {
        net->removeSites(additionalPoints);
    }

    if (alloRetPoint == nullopt)
    {
        return nullopt;
    }

    // cout << "PathProxy: getEgoDirection returns " << retPoint->alloToEgo(*ownPos)->toString() << endl;
    return alloRetPoint->toEgo(ownPos);
}

nonstd::optional<geometry::CNPointEgo> PathProxy::getEgoDirection(geometry::CNPointEgo egoTarget,
                                                                  const IPathEvaluator &pathEvaluator,
                                                                  const PathPlannerQuery &query)
{
    lastPathTarget = egoTarget;

    // get voronoi diagram, which is ready to be used
    auto net = this->wm->pathPlanner->getCurrentVoronoiNet();

    auto ownPosInfo = this->wm->rawSensorData->getOwnPositionVisionBuffer().getLastValid();
    if (net == nullptr || !net->ownPosAvail || ownPosInfo == nullptr)
    {
        // We are not localized (or have no vNet), so we can't plan a path.
        return nullopt;
    }
    auto ownPos = ownPosInfo->getInformation();
    // if there are additional points insert them into the voronoi diagram
    if (query.additionalPoints != nullopt)
    {
        net->insertAdditionalPoints(*query.additionalPoints, EntityType::Obstacle);
    }

    // block specific field areas
    if (query.block3MetersAroundBall)
    {
        net->blockThreeMeterAroundBall();
    }
    if (query.blockOppPenaltyArea)
    {
        net->blockOppPenaltyArea();
    }
    if (query.blockOwnPenaltyArea)
    {
        net->blockOwnPenaltyArea();
    }
    if (query.blockOppGoalArea && !query.blockOppPenaltyArea)
    {
        net->blockOppGoalArea();
    }
    if (query.blockOwnGoalArea && !query.blockOwnPenaltyArea)
    {
        net->blockOwnGoalArea();
    }
    if (query.circleRadius != -1 && query.circleCenterPoint != nullopt)
    {
        net->blockCircle(*query.circleCenterPoint, query.circleRadius);
    }
    if (query.rectangleLowerRightCorner != nullopt && query.rectangleUpperLeftCorner != nullopt)
    {
        net->blockRectangle(*query.rectangleUpperLeftCorner, *query.rectangleLowerRightCorner);
    }

    // plan
    nonstd::optional<geometry::CNPointAllo> alloRetPoint = nullopt;
    auto alloTarget = egoTarget.toAllo(ownPos);
    auto path = this->wm->pathPlanner->plan(net, ownPos.getPoint(), alloTarget, pathEvaluator);

    if (path != nullptr && path->size() > 0)
    {
        // get first point of returned path
        if (path->size() < 3)
        {
            alloRetPoint = nonstd::make_optional<geometry::CNPointAllo>(path->at(0));
        }
        else
        {
            path = applyShortcut(*path, ownPos, *net);
            alloRetPoint = nonstd::make_optional<geometry::CNPointAllo>(path->at(0));
        }
        // send debug msgs
        if (pathPlannerDebug)
        {
            path->insert(path->begin(), ownPos.getPoint());
            sendPathPlannerMsg(*path);
            sendVoronoiNetMsg(*net);
        }
    }

    if (alloRetPoint == nullopt)
    {
        return nullopt;
    }

    //		cout << "PathProxy: getEgoDirection returns " << retPoint->alloToEgo(*ownPos)->toString() << endl;
    return alloRetPoint->toEgo(ownPos);
}

shared_ptr<vector<geometry::CNPointAllo>> PathProxy::applyShortcut(const vector<geometry::CNPointAllo> &path,
                                                                   geometry::CNPositionAllo ownPos,
                                                                   const VoronoiNet &net)
{
    auto ret = make_shared<vector<geometry::CNPointAllo>>(path);
    bool shortcutBlocked = false;
    for (int i = path.size() - 2; i >= 0; i--)
    {
        for (auto current : *net.getAlloClusteredObsWithMe())
        {
            if (current.id == wm->getOwnId())
            {
                continue;
            }
            shortcutBlocked = shortcutBlocked ||
                              wm->pathPlanner->corridorCheck(ownPos.getPoint(), path.at(i), current.position.getPoint(),
                                                             wm->pathPlanner->getRobotRadius());
        }
        for (auto current : *net.getAdditionalObstacles())
        {
            shortcutBlocked = shortcutBlocked ||
                              wm->pathPlanner->corridorCheck(ownPos.getPoint(), path.at(i), current,
                                                             wm->pathPlanner->getRobotRadius());
        }
        if (!shortcutBlocked)
        {
            ret->erase(ret->begin(), ret->begin() + i);
            break;
        }
    }
    return ret;
}

/**
 * get the path proxy instacne
 */
PathProxy *PathProxy::getInstance()
{
    static PathProxy instance;
    return &instance;
}

/**
 * send debug msg with pathplanner path
 * @param path shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
 */
void PathProxy::sendPathPlannerMsg(const vector<geometry::CNPointAllo> &path)
{
    msl_msgs::PathPlanner pathMsg;
    pathMsg.senderId = this->wm->getOwnId();
    for (int i = 0; i < path.size(); i++)
    {
        msl_msgs::Point2dInfo info;
        info.x = path.at(i).x;
        info.y = path.at(i).y;
        pathMsg.pathPoints.push_back(info);
    }
    this->pathPub.publish(pathMsg);
}

/**
 * send debug msg with voroni infos
 * @param sites shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
 * @param voronoi shared_ptr<VoronoiNet>
 */
void PathProxy::sendVoronoiNetMsg(const VoronoiNet &voronoi)
{
    msl_msgs::VoronoiNetInfo netMsg;
    netMsg.senderId = this->wm->getOwnId();
    for (auto cluster : *voronoi.getAlloClusteredObsWithMe())
    {
        msl_msgs::Point2dInfo info;
        info.x = cluster.position.x;
        info.y = cluster.position.y;
        netMsg.sites.push_back(info);
    }

    for (auto ob : *voronoi.getArtificialObstacles())
    {
        msl_msgs::Point2dInfo info;
        info.x = ob.x;
        info.y = ob.y;
        netMsg.sites.push_back(info);
    }

    shared_ptr<vector<geometry::CNPointAllo>> linePoints = calculateCroppedVoronoi(voronoi);
    for (int i = 0; i < linePoints->size(); i++)
    {
        msl_msgs::Point2dInfo info;
        info.x = linePoints->at(i).x;
        info.y = linePoints->at(i).y;
        netMsg.linePoints.push_back(info);
    }
    this->voroniPub.publish(netMsg);
	return;
}

/**
 * calculates cropped voronoi for debug msg
 * @param sites shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
 * @param voronoi shared_ptr<VoronoiNet>
 */
shared_ptr<vector<geometry::CNPointAllo>> PathProxy::calculateCroppedVoronoi(const VoronoiNet &voronoi)
{
    // create bounding box around field with additional 3m distance
    shared_ptr<vector<geometry::CNPointAllo>> ret = make_shared<vector<geometry::CNPointAllo>>();
    Iso_rectangle_2 bbox(-wm->field->getFieldLength() / 2 - 3000, -wm->field->getFieldWidth() / 2 - 3000,
                         wm->field->getFieldLength() / 2 + 3000, wm->field->getFieldWidth() / 2 + 3000);
    // create cropped voronoi
    Cropped_voronoi_from_delaunay vor(bbox);
    // get delaunay
    DelaunayTriangulation dt = voronoi.getVoronoi()->dual();
    dt.draw_dual(vor);
    // get sites
    for (auto it = vor.m_cropped_vd.begin(); it != vor.m_cropped_vd.end(); it++)
    {
        // check if source and target exist
        // if there is no source or target the edge is connected to point at infinity
        if (!std::isnan(it->source().x()) && !std::isnan(it->source().y()) && !std::isnan(it->target().x()) &&
            !std::isnan(it->target().y()))
        {
            // push back source and target
            ret->push_back(geometry::CNPointAllo(it->source().x(), it->source().y()));
            ret->push_back(geometry::CNPointAllo(it->target().x(), it->target().y()));
        }
    }
    return ret;
}

} /* namespace msl */
