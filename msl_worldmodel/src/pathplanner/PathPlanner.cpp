#include "Ball.h"

#include "MSLWorldModel.h"
#include "RawSensorData.h"
#include <cnc_geometry/Calculator.h>
#include <pathplanner/PathPlanner.h>

using nonstd::optional;
using nonstd::nullopt;
using std::vector;
using std::shared_ptr;
using std::make_shared;

namespace msl
{

PathPlanner::PathPlanner(MSLWorldModel *wm, int count)
{
    this->wm = wm;
    this->artificialObjectNet = make_shared<VoronoiNet>(wm);
    this->lastClosestNode = nullopt;
    this->lastClosestPointToBlock = nullopt;
    this->lastTarget == nullopt;
    this->sc = supplementary::SystemConfig::getInstance();
    this->voronoiDiagrams.reserve(count);
    for (int i = 0; i < count; i++)
    {
        auto voi = make_shared<VoronoiNet>(wm);
        // TODO maybe VoronoiNet::generateVoronoiDiagram can spare inserting these art. obstacles again, if we maintain
        // them properly
        // TODO save delaunay for initialization
        voi->setVoronoi(
            make_shared<VoronoiDiagram>((DelaunayTriangulation) this->artificialObjectNet->getVoronoi()->dual()));

        this->voronoiDiagrams.push_back(voi);
    }
    this->robotRadius = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL);
    this->minEdgeWidth = (*sc)["PathPlanner"]->get<double>("PathPlanner", "minEdgeWidth", NULL);
    this->pathDeviationWeight = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "pathDeviationWeight", NULL);
    this->currentVoronoiPos = -1;
    this->corridorWidthDivisor = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "corridorWidthDivisor", NULL);
    this->corridorWidthDivisorBall =
        (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "corridorWidthDivisorBall", NULL);
    this->pathPlannerDebug = (*this->sc)["PathPlanner"]->get<bool>("PathPlanner", "pathPlannerDebug", NULL);
    this->additionalCorridorWidth =
        (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "additionalCorridorWidth", NULL);
    this->additionalBallCorridorWidth =
        (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "additionalBallCorridorWidth", NULL);
    this->snapDistance = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "snapDistance", NULL);
    this->marginToBlockedArea = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "marginToBlockedArea", NULL);
    this->lastPath = nullptr;
    this->corridorPub = n.advertise<msl_msgs::CorridorCheck>("/PathPlanner/CorridorCheck", 10);
    initializeArtificialObstacles();
}

PathPlanner::~PathPlanner()
{
}

void PathPlanner::prepareVoronoiDiagram()
{
    lock_guard<mutex> lock(voronoiMutex);
    auto ownPosInfo = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValid();
    currentVoronoiPos = (currentVoronoiPos + 1) % voronoiDiagrams.size();
    if (ownPosInfo != nullptr)
    {
        voronoiDiagrams.at(currentVoronoiPos)->generateVoronoiDiagram(true);
    }
    else
    {
        voronoiDiagrams.at(currentVoronoiPos)->generateVoronoiDiagram(false);
    }
}

void PathPlanner::initializeArtificialObstacles()
{
    // TODO before 2 Meters now field->surrounding * 2
    vector<VoronoiDiagram::Site_2> toInsert;
    int baseSize = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "artificialObjectBaseSize", NULL);
    if (wm->field->getFieldLength() / baseSize > 20 || wm->field->getFieldWidth() / baseSize > 20)
    {
        baseSize = (int)max(wm->field->getFieldLength() / 20, wm->field->getFieldWidth() / 20);
    }
    // TODO was time 2 before
    int lengthInterval =
        (int)(baseSize +
              ((int)(wm->field->getFieldLength() + wm->field->getSurrounding() * 1.5) % baseSize) /
                  (int)((int)(wm->field->getFieldLength() + wm->field->getSurrounding() * 1.5) / baseSize));
    int widthInterval =
        (int)(baseSize +
              ((int)(wm->field->getFieldWidth() + wm->field->getSurrounding() * 1.5) % baseSize) /
                  (int)((int)(wm->field->getFieldWidth() + wm->field->getSurrounding() * 1.5) / baseSize));
    int halfFieldLength = (int)wm->field->getFieldLength() / 2 + wm->field->getSurrounding();
    int halfFieldWidth = (int)wm->field->getFieldWidth() / 2 + wm->field->getSurrounding();

    // up right
    toInsert.push_back(VoronoiDiagram::Site_2(halfFieldLength, -halfFieldWidth));
    // down right
    toInsert.push_back(VoronoiDiagram::Site_2(-halfFieldLength, -halfFieldWidth));
    // down left
    toInsert.push_back(VoronoiDiagram::Site_2(-halfFieldLength, halfFieldWidth));
    // up left
    toInsert.push_back(VoronoiDiagram::Site_2(halfFieldLength, halfFieldWidth));

    int x = 0;
    int y = halfFieldWidth;
    for (x = -halfFieldLength + lengthInterval; x <= halfFieldLength - lengthInterval; x += lengthInterval)
    { // side sides
        toInsert.push_back(VoronoiDiagram::Site_2(x, y));
        toInsert.push_back(VoronoiDiagram::Site_2(x, -y));
    }

    x = halfFieldLength;
    for (y = -halfFieldWidth + widthInterval; y <= halfFieldWidth - widthInterval; y += widthInterval)
    { // goal sides
        toInsert.push_back(VoronoiDiagram::Site_2(x, y));
        toInsert.push_back(VoronoiDiagram::Site_2(-x, y));
    }
    this->artificialObjectNet->getVoronoi()->insert(toInsert.begin(), toInsert.end());
}

shared_ptr<vector<geometry::CNPointAllo>> PathPlanner::plan(const VoronoiNet &voronoi, geometry::CNPointAllo startPos,
                                                            geometry::CNPointAllo goal, const IPathEvaluator &eval)
{
    if (lastTarget == nullopt || lastTarget->distanceTo(goal) > 50)
    { // if there was no target before || there was a target before, it has to be different to be saved
        lastTarget = goal;
    }

    // check if the goal is reachable directly by checking a corridor between the robot and the goal
    bool reachable = true;

    auto vec = voronoi.getAlloClusteredObsWithMe();
    double shortestDist = 100000000.0;
    optional<geometry::CNPointAllo> closestOpp;
    for (auto cluster : *vec)
    {
        if (cluster.id == wm->getOwnId())
        {
            continue;
        }

        // if there is an obstacle inside the corridor the goal is not reachable
        auto point = cluster.position.getPoint();
        if (corridorCheck(startPos, goal, point, this->robotRadius))
        {
            reachable = false;
            break;
        }

        double tmpDist = goal.distanceTo(point);
        if (tmpDist < shortestDist)
        {
            closestOpp = point;
            shortestDist = tmpDist;
        }
    }

    // this helps to avoid obstacles which are very close to the goal, but behind it -
    // in such cases the corridor check say, you can drive directly to the goal, although an opponent robots corner can
    // reach into the corridor...
    if (closestOpp && shortestDist < 400 && closeOppToBallCheck(voronoi, startPos, goal, *closestOpp))
    {
        reachable = false;
    }

    if (reachable)
    {
        auto vec = voronoi.getArtificialObstacles();
        for (auto obs : *vec)
        {
            // if there is an obstacle inside the corridor the goal is not reachable
            if (corridorCheck(startPos, goal, obs))
            {
                reachable = false;
                break;
            }
        }
    }

    if (reachable)
    {
        auto vec = voronoi.getAdditionalObstacles();
        for (auto obs : *vec)
        {
            // if there is an obstacle inside the corridor the goal is not reachable
            if (corridorCheck(startPos, goal, obs))
            {
                reachable = false;
                break;
            }
        }
    }

    // if the corridor is free the robot can drive towards the goal
    if (reachable)
    {
        auto ret = make_shared<vector<geometry::CNPointAllo>>();
        ret->push_back(goal);
        this->lastPath = ret;
        return ret;
    }

    // there is no direct way so we have to search a way to the goal
    auto ret = aStarSearch(voronoi, startPos, goal, eval);
    // if there is a path save and return it
    if (ret != nullptr)
    {
        lastPath = ret;
        return ret;
    }

    /**
     * In this case, we should be surrounded and therefore, we try to drive in the direction which gives us the most
     * space according to
     * our surrounding voronoi vertices.
     */

    ret = make_shared<vector<geometry::CNPointAllo>>();
    shared_ptr<Vertex> bestVertex = nullptr;
    double bestDist = 0;
    // find vertex of robots voronoi face and which has maximum distance
    for (auto vertice : *voronoi.getVerticesOfFace(startPos))
    {
        double curDist = distanceTo(startPos, vertice);
        if (curDist > bestDist)
        {
            bestVertex = make_shared<Vertex>(vertice);
            bestDist = curDist;
        }
    }

    ret->push_back(geometry::CNPointAllo(bestVertex->point().x(), bestVertex->point().y()));

    lastPath = ret;
    return ret;
}

shared_ptr<vector<geometry::CNPointAllo>> PathPlanner::aStarSearch(const VoronoiNet &voronoi,
                                                                   geometry::CNPointAllo startPos,
                                                                   geometry::CNPointAllo goal,
                                                                   const IPathEvaluator &pathEvaluator)
{
    // return
    auto ret = make_shared<vector<geometry::CNPointAllo>>();

    // vector with open searchnodes
    auto open = make_shared<vector<SearchNode>>();

    // vector with closed search nodes
    auto closed = make_shared<vector<SearchNode>>();

    // get closest Vertices to ownPos => start point for a star serach
    auto closestVerticesToOwnPos = *voronoi.getVerticesOfFace(startPos);

    // get closest Vertices to goal => goal for a star serach
    auto closestVerticesToGoal = *voronoi.getVerticesOfFace(goal);

    // a star serach

    // insert all vertices of startpos face
    for (auto vertice : closestVerticesToOwnPos)
    {
        // TODO create the right initial costs
        auto node = make_shared<SearchNode>(vertice, 0, 0, nullptr);
        pair<double, double> costHeuristicPair =
                pathEvaluator.evalInitial(startPos, goal, node, voronoi, this->lastPath, this->lastTarget);
        node->setCost(costHeuristicPair.first);
        node->setHeuristic(costHeuristicPair.second);
        insert(*open, node);
    }

    shared_ptr<SearchNode> currentNode, stop;

    // while there is still a node to expand
    while (open->size() != 0)
    {
        // get first node in open
        currentNode = make_shared<SearchNode>(open->at(0));
        open->erase(open->begin());
        closed->push_back(*currentNode);

        // check if the goal is reachable
        if (checkGoalReachable(voronoi, *currentNode, closestVerticesToGoal, goal))
        {
            stop = currentNode;
            ret->push_back(goal);
            break;
        }

        // expand the current node
        this->expandNode(currentNode, *open, *closed, startPos, goal, pathEvaluator, voronoi);

        // if there is no closest node || the distance of the current node is closer to the goal than the last one
        if (stop == nullptr || goal.distanceTo(currentNode->getPoint()) < goal.distanceTo(stop->getPoint()))
        {
            stop = currentNode;
        }
    }

    if (stop == nullptr)
        return nullptr;

    // get the way to reach this node
    ret->push_back(stop->getPoint());
    currentNode = stop;

    shared_ptr<SearchNode> predecessor = currentNode;
    while ((predecessor = predecessor->getPredecessor()) != nullptr)
    {
        ret->push_back(predecessor->getPoint());
    }
    reverse(ret->begin(), ret->end());

    //		lastPath = ret;
    //		lastClosestNode = nullptr;
    return ret;
}

bool PathPlanner::isAdmissableEdge(VoronoiDiagram::Halfedge_around_vertex_circulator incidentHalfEdge,
                                   geometry::CNPointAllo startPos, const VoronoiNet &voronoi) const
{
    // dont follow edges to infinity
    if (!incidentHalfEdge->has_source())
    {
        return false;
    }

    // dont follow edges through art obstacles
    if (voronoi.getTypeOfSite(incidentHalfEdge->up()->point()) == EntityType::ArtificialObstacle &&
        voronoi.getTypeOfSite(incidentHalfEdge->down()->point()) == EntityType::ArtificialObstacle)
    {
        return !wm->field->isInsideField(startPos);
    }

    double distToEdge = geometry::distancePointToLineSegmentCalc(
            incidentHalfEdge->up()->point().x(), incidentHalfEdge->up()->point().y(),
            incidentHalfEdge->source()->point().x(), incidentHalfEdge->source()->point().y(),
            incidentHalfEdge->target()->point().x(), incidentHalfEdge->target()->point().y());

    return distToEdge > this->minEdgeWidth;
}

void PathPlanner::expandNode(shared_ptr<SearchNode> currentNode, vector<SearchNode> &open,
                             const vector<SearchNode> &closed, geometry::CNPointAllo startPos,
                             geometry::CNPointAllo goal, const IPathEvaluator &pathEvaluator, const VoronoiNet &voronoi)
{
    // get neighbored nodes
    vector<VoronoiDiagram::Halfedge_around_vertex_circulator> neighborEdges; // = getNeighboredVertices(currentNode);
    VoronoiDiagram::Halfedge_around_vertex_circulator incidentHalfEdge = currentNode->getIncidentEdges();
    VoronoiDiagram::Halfedge_around_vertex_circulator begin = incidentHalfEdge;
    do
    {
        if (this->isAdmissableEdge(incidentHalfEdge, startPos, voronoi))
        {
            // if node is already closed || if node has still to be expanded but there is a cheaper way
            if (contains(closed, incidentHalfEdge) || contains(open, incidentHalfEdge))
            {
                continue;
            }

            // TODO init correct costs + heuristic
            neighborEdges.push_back(incidentHalfEdge);
        }
    } while (begin != ++incidentHalfEdge);

    for (auto edge : neighborEdges)
    {
        auto nextNode = make_shared<SearchNode>(edge, 0, 0, nullptr);
        pair<double, double> costHeuristicPair = pathEvaluator.eval(goal, currentNode, nextNode, voronoi);
        if (costHeuristicPair.first > 0)
        {
            nextNode->setPredecessor(currentNode);
            nextNode->setCost(costHeuristicPair.first);
            nextNode->setHeuristic(costHeuristicPair.second);
            PathPlanner::insert(open, nextNode);
        }
    }
}

bool PathPlanner::contains(const vector<SearchNode> &nodes, VoronoiDiagram::Halfedge_around_vertex_circulator edge)
{
    for (auto node : nodes)
    {
        if (edge == node.getEdge())
        {
            return true;
        }
    }
    return false;
}

void PathPlanner::insert(vector<SearchNode> &vect, shared_ptr<SearchNode> currentNode)
{
    vector<SearchNode>::iterator it =
        std::upper_bound(vect.begin(), vect.end(), *currentNode,
                         PathPlanner::compare); // find proper position in descending order

    vect.insert(it, *currentNode);              // insert before iterator it
}

bool PathPlanner::compare(const SearchNode &first, const SearchNode &second)
{
    if (first.getCost() + first.getHeuristic() < second.getCost() + second.getHeuristic())
    {
        return true;
    }
    return false;
}

bool PathPlanner::checkGoalVerticesReached(const vector<Vertex> &closestVerticesToGoal, SearchNode &currentNode)
{
    for (auto vertice : closestVerticesToGoal)
    {
        if (currentNode.matches(vertice))
        {
            return true;
        }
    }
    return false;
}

bool PathPlanner::closeOppToBallCheck(const VoronoiNet &voronoi, geometry::CNPointAllo currentPos,
                                      geometry::CNPointAllo goal, geometry::CNPointAllo obstaclePoint)
{
    auto g2opp = obstaclePoint - goal;
    double angle2BallOppLine = abs(geometry::deltaAngle(currentPos.angleZ(), g2opp.angleZ()));
    // cout << "PathPlanner: angle2BallOppLine: \t" << angle2BallOppLine << endl;
    if (angle2BallOppLine > M_PI / 2.0 && angle2BallOppLine < M_PI * 2.0 / 3.0)
    {
        return true;
    }
    return false;
}

bool PathPlanner::corridorCheck(geometry::CNPointAllo currentPos, geometry::CNPointAllo goal,
                                geometry::CNPointAllo obstaclePoint, double obstacleRadius)
{
    // calculate length x and y offset
    double length = currentPos.distanceTo(goal);
    double dx = currentPos.x - goal.x;
    double dy = currentPos.y - goal.y;
    dx /= length;
    dy /= length;
    // calculate corridor corner points
    geometry::CNPointAllo p1 =
        geometry::CNPointAllo(currentPos.x + (this->robotRadius + this->additionalCorridorWidth) * dy,
                              currentPos.y - (this->robotRadius + this->additionalCorridorWidth) * dx);
    geometry::CNPointAllo p2 =
        geometry::CNPointAllo(currentPos.x - (this->robotRadius + this->additionalCorridorWidth) * dy,
                              currentPos.y + (this->robotRadius + this->additionalCorridorWidth) * dx);
    geometry::CNPointAllo p3 =
        geometry::CNPointAllo(goal.x +
                                  std::max(this->robotRadius + this->additionalCorridorWidth,
                                           length / this->corridorWidthDivisor + this->additionalCorridorWidth) *
                                      dy,
                              goal.y -
                                  std::max(this->robotRadius + this->additionalCorridorWidth,
                                           length / this->corridorWidthDivisor + this->additionalCorridorWidth) *
                                      dx);
    geometry::CNPointAllo p4 =
        geometry::CNPointAllo(goal.x -
                                  std::max(this->robotRadius + this->additionalCorridorWidth,
                                           length / this->corridorWidthDivisor + this->additionalCorridorWidth) *
                                      dy,
                              goal.y +
                                  std::max(this->robotRadius + this->additionalCorridorWidth,
                                           length / this->corridorWidthDivisor + this->additionalCorridorWidth) *
                                      dx);
    vector<geometry::CNPointAllo> points;
    points.push_back(p1);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p2);
    // send debug msg
    if (pathPlannerDebug)
    {
        sendCorridorCheck(points);
    }

    //		cout << "----------------------" << endl;
    return (geometry::distancePointToLine(p2, p4, obstaclePoint) < this->robotRadius &&
            geometry::distancePointToLine(p4, p3, obstaclePoint) < 0.0 &&
            geometry::distancePointToLine(p3, p1, obstaclePoint) < this->robotRadius &&
            geometry::distancePointToLine(p1, p2, obstaclePoint) < 0.0);
    // return result
    // return obstaclePoint != nullptr && geometry::isInsidePolygon(points, obstaclePoint);
}

bool PathPlanner::corridorCheckBall(geometry::CNPointAllo currentPos, geometry::CNPointAllo goal,
                                    geometry::CNPointAllo obstaclePoint, double obstacleRadius)
{
    // calculate length x and y offset
    double length = currentPos.distanceTo(goal);
    double dx = currentPos.x - goal.x;
    double dy = currentPos.y - goal.y;
    double dist = length;
    dx /= dist;
    dy /= dist;
    // calculate corridor corner points
    geometry::CNPointAllo p1 =
        geometry::CNPointAllo(currentPos.x + (this->robotRadius) * dy, currentPos.y - (this->robotRadius) * dx);
    geometry::CNPointAllo p2 =
        geometry::CNPointAllo(currentPos.x - (this->robotRadius) * dy, currentPos.y + (this->robotRadius) * dx);
    geometry::CNPointAllo p3 = geometry::CNPointAllo(
        goal.x +
            std::max(wm->ball->getBallDiameter() / 2 + this->additionalBallCorridorWidth,
                     length / this->corridorWidthDivisorBall + this->additionalBallCorridorWidth) *
                dy,
        goal.y -
            std::max(wm->ball->getBallDiameter() / 2 + this->additionalBallCorridorWidth,
                     length / this->corridorWidthDivisorBall + this->additionalBallCorridorWidth) *
                dx);
    geometry::CNPointAllo p4 = geometry::CNPointAllo(
        goal.x -
            std::max(wm->ball->getBallDiameter() / 2 + this->additionalBallCorridorWidth,
                     length / this->corridorWidthDivisorBall + this->additionalBallCorridorWidth) *
                dy,
        goal.y +
            std::max(wm->ball->getBallDiameter() / 2 + this->additionalBallCorridorWidth,
                     length / this->corridorWidthDivisorBall + this->additionalBallCorridorWidth) *
                dx);
    vector<geometry::CNPointAllo> points;
    points.push_back(p1);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p2);
    // send debug msg
    if (pathPlannerDebug)
    {
        sendCorridorCheck(points);
    }
    return (geometry::distancePointToLine(p2, p4, obstaclePoint) < this->robotRadius &&
            geometry::distancePointToLine(p4, p3, obstaclePoint) < this->robotRadius &&
            geometry::distancePointToLine(p3, p1, obstaclePoint) < this->robotRadius &&
            geometry::distancePointToLine(p1, p2, obstaclePoint) < this->robotRadius);
    // return result
    // return obstaclePoint != nullptr && geometry::isInsidePolygon(points, obstaclePoint);
}

void PathPlanner::sendCorridorCheck(vector<geometry::CNPointAllo> points)
{
    msl_msgs::CorridorCheck cc;
    cc.senderId = this->wm->getOwnId();
    for (int i = 0; i < points.size(); i++)
    {
        msl_msgs::Point2dInfo info;
        info.x = points.at(i).x;
        info.y = points.at(i).y;
        cc.corridorPoints.push_back(info);
    }
    this->corridorPub.publish(cc);
	return;
}

shared_ptr<const vector<geometry::CNPointAllo>> PathPlanner::getLastPath() const
{
    return lastPath;
}

double PathPlanner::getAdditionalCorridorWidth() const
{
    return additionalCorridorWidth;
}

optional<geometry::CNPointAllo> PathPlanner::getLastTarget() const
{
    return lastTarget;
}

double PathPlanner::getRobotRadius() const
{
    return robotRadius;
}

bool PathPlanner::checkGoalReachable(const VoronoiNet &voronoi, SearchNode &currentNode,
                                     const vector<Vertex> &closestVerticesToGoal, geometry::CNPointAllo goal)
{
    // we have to reach the goal vertices
    if (checkGoalVerticesReached(closestVerticesToGoal, currentNode))
    {
        // if the goal vertices are reached
        optional<VoronoiDiagram::Point_2> obstacle = voronoi.getSiteOfFace(Point_2(goal.x, goal.y));
        if(!obstacle)
        {
            return false;
        }

        if (voronoi.getTypeOfSite(*obstacle) == this->wm->getOwnId())
        {
            return true;
        }

        auto obstaclePoint = geometry::CNPointAllo(obstacle->x(), obstacle->y());
        // check if there is an obstacle on the way to the goal
        auto vertexPoint = currentNode.getPoint();
        return !corridorCheck(vertexPoint, goal, obstaclePoint, this->robotRadius);
    }
    return false;
}

shared_ptr<VoronoiNet> PathPlanner::getArtificialObjectNet()
{
    return artificialObjectNet;
}

shared_ptr<const VoronoiNet> PathPlanner::getArtificialObjectNet() const
{
    return artificialObjectNet;
}

vector<shared_ptr<VoronoiNet>> PathPlanner::getVoronoiNets() const
{
    // TODO: stopped working here
    lock_guard<mutex> lock(this->voronoiMutex);
    return this->voronoiDiagrams;
}

shared_ptr<VoronoiNet> PathPlanner::getCurrentVoronoiNet()
{
    lock_guard<mutex> lock(this->voronoiMutex);
    if (this->currentVoronoiPos == -1)
    {
        return nullptr;
    }
    return this->voronoiDiagrams.at(currentVoronoiPos);
}

double PathPlanner::getPathDeviationWeight() const
{
    return pathDeviationWeight;
}

double PathPlanner::getDribbleAngleTolerance() const
{
    return dribble_angleTolerance;
}

double PathPlanner::getDribbleRotationWeight() const
{
    return dribble_rotationWeight;
}

shared_ptr<vector<geometry::CNPointAllo>> PathPlanner::getArtificialFieldSurroundingObs() const
{
    shared_ptr<vector<geometry::CNPointAllo>> toInsert = make_shared<vector<geometry::CNPointAllo>>();
    int baseSize = (*this->sc)["PathPlanner"]->get<double>("PathPlanner", "artificialObjectBaseSize", NULL);
    if (this->wm->field->getFieldLength() / baseSize > 20 || this->wm->field->getFieldWidth() / baseSize > 20)
    {
        baseSize = (int)max(this->wm->field->getFieldLength() / 20, this->wm->field->getFieldWidth() / 20);
    }
    int lengthInterval = (int)(baseSize +
                               ((int)(this->wm->field->getFieldLength() + 2000) % baseSize) /
                                   (int)((int)(this->wm->field->getFieldLength() + 2000) / baseSize));
    int widthInterval = (int)(baseSize +
                              ((int)(this->wm->field->getFieldWidth() + 2000) % baseSize) /
                                  (int)((int)(this->wm->field->getFieldWidth() + 2000) / baseSize));
    int halfFieldLength = (int)this->wm->field->getFieldLength() / 2 + 1000;
    int halfFieldWidth = (int)this->wm->field->getFieldWidth() / 2 + 1000;

    // up right
    toInsert->push_back(geometry::CNPointAllo(halfFieldLength, -halfFieldWidth));
    // down right
    toInsert->push_back(geometry::CNPointAllo(-halfFieldLength, -halfFieldWidth));
    // down left
    toInsert->push_back(geometry::CNPointAllo(-halfFieldLength, halfFieldWidth));
    // up left
    toInsert->push_back(geometry::CNPointAllo(halfFieldLength, halfFieldWidth));

    int x = 0;
    int y = halfFieldWidth;
    for (x = -halfFieldLength + lengthInterval; x <= halfFieldLength - lengthInterval; x += lengthInterval)
    { // side sides
        toInsert->push_back(geometry::CNPointAllo(x, y));
        toInsert->push_back(geometry::CNPointAllo(x, -y));
    }

    x = halfFieldLength;
    for (y = -halfFieldWidth + widthInterval; y <= halfFieldWidth - widthInterval; y += widthInterval)
    { // goal sides
        toInsert->push_back(geometry::CNPointAllo(x, y));
        toInsert->push_back(geometry::CNPointAllo(-x, y));
    }
    return toInsert;
}

double PathPlanner::distanceTo(geometry::CNPointAllo v1, Vertex v2)
{
    return std::sqrt(std::pow(v2.point().x() - v1.x, 2) + std::pow(v2.point().y() - v1.y, 2));
}

} /* namespace alica */
