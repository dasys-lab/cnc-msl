#include "pathplanner/VoronoiNet.h"
#include "Ball.h"
#include "MSLWorldModel.h"
#include "Robots.h"
#include "obstaclehandler/Obstacles.h"
#include "pathplanner/PathPlanner.h"

#include <msl/robot/IntRobotID.h>

#include <SystemConfig.h>

namespace msl
{

VoronoiNet::VoronoiNet(MSLWorldModel *wm)
{
    this->wm = wm;
    this->sc = supplementary::SystemConfig::getInstance();
    this->voronoi = make_shared<VoronoiDiagram>();
    this->ownPosAvail = false;
    this->alloClusteredObsWithMe = make_shared<vector<shared_ptr<geometry::CNRobot>>>();
    this->artificialObstacles = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    this->additionalObstacles = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
}

VoronoiNet::VoronoiNet(shared_ptr<VoronoiNet> net)
{
    this->wm = net->wm;
    this->sc = net->sc;
    this->voronoi = make_shared<VoronoiDiagram>();
    this->ownPosAvail = net->ownPosAvail;

    this->alloClusteredObsWithMe = make_shared<vector<shared_ptr<geometry::CNRobot>>>();
    for (auto cluster : *net->getAlloClusteredObsWithMe())
    {
        this->alloClusteredObsWithMe->push_back(cluster);
    }

    this->artificialObstacles = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    for (auto obs : *net->getArtificialObstacles())
    {
        this->artificialObstacles->push_back(obs);
    }

    this->additionalObstacles = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
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

int VoronoiNet::getTypeOfSite(Site_2 site)
{
    auto result = this->pointRobotKindMapping.find(site);
    if (result != this->pointRobotKindMapping.end())
    {
        return result->second;
    }
    return EntityType::UndefinedEntity;
}

/**
 * generates a VoronoiDiagram and inserts given points
 * @param points vector<shared_ptr<geometry::CNPoint2D>>
 * @return shared_ptr<VoronoiDiagram>
 */
void VoronoiNet::generateVoronoiDiagram(bool ownPosAvail)
{
    lock_guard<mutex> lock(netMutex);
    // clear data
    this->clearVoronoiNet();

    this->ownPosAvail = ownPosAvail;

    // insert allo obstacles (including me) into voronoi diagram
    vector<Site_2> sites;
    this->alloClusteredObsWithMe = make_shared<vector<shared_ptr<geometry::CNRobot>>>();
    auto alloObs = wm->obstacles->getAlloObstaclesWithMe();
    if (alloObs != nullptr)
    {
        for (auto cluster : *alloObs)
        {
            Site_2 site(cluster->x, cluster->y);
            pointRobotKindMapping[site] = dynamic_cast<const msl::robot::IntRobotID*>(cluster->id)->getId();
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
        pointRobotKindMapping[*iter] = EntityType::ArtificialObstacle;
    } while (++iter != wm->pathPlanner->getArtificialObjectNet()->getVoronoi()->sites_end());

    this->voronoi->insert(wm->pathPlanner->getArtificialObjectNet()->getVoronoi()->sites_begin(),
                          wm->pathPlanner->getArtificialObjectNet()->getVoronoi()->sites_end());

    //		cout << "VoronoiNet: obstWithMe " << alloClusteredObs->size() << " : sites " << sites.size() << " : artObs " << artObs->size() << endl;
}

/**
 * insert additional points into the voronoi diagram
 * @param points shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
 */
void VoronoiNet::insertAdditionalPoints(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points, EntityType type)
{
    lock_guard<mutex> lock(netMutex);
    vector<Site_2> sites;

    for (auto point = points->begin(); point != points->end();)
    {

        if (!wm->field->isInsideField(*point))
        {
            point = points->erase(point);
        }
        else
        {
            Site_2 site((*point)->x, (*point)->y);
            this->pointRobotKindMapping[site] = type;
            sites.push_back(site);

            if (type == EntityType::Obstacle)
                this->additionalObstacles->push_back(*point);
            else if (type == EntityType::ArtificialObstacle)
                this->artificialObstacles->push_back(*point);

            ++point;
        }
    }
    this->voronoi->insert(sites.begin(), sites.end());
}

shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> VoronoiNet::getTeamMateVerticesCNPoint2D(const msl::robot::IntRobotID* teamMateId)
{
    // locate teammate
    shared_ptr<geometry::CNPosition> teamMatePos = wm->robots->teammates.getTeamMatePosition(teamMateId);

    if (teamMatePos == nullptr)
    {
        return nullptr;
    }

    // get vertices
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    auto vertices = this->getVerticesOfFace(make_shared<geometry::CNPoint2D>(teamMatePos->x, teamMatePos->y));
    for (int i = 0; i < vertices->size(); i++)
    {
        ret->push_back(make_shared<geometry::CNPoint2D>(vertices->at(i)->point().x(), vertices->at(i)->point().y()));
    }
    return ret;
}

/**
 * removes given sites from voronoi net
 * @param sites shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
 */
void VoronoiNet::removeSites(shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> sites)
{
    for (int i = 0; i < sites->size(); i++)
    {
        VoronoiDiagram::Point_2 point(sites->at(i).first->x, sites->at(i).first->y);
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

/**
 * deletes sites from voronoi net and clears pointRobotKindMapping
 */
void VoronoiNet::clearVoronoiNet()
{
    this->voronoi->clear();
    this->pointRobotKindMapping.clear();
    this->alloClusteredObsWithMe->clear();
    this->artificialObstacles->clear();
    this->additionalObstacles->clear();
}

/**
 * locates face of point and returns verticespointRobotKindMapping
 * @param point shared_ptr<geometry::CNPoint2D>
 * @return shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
 */
shared_ptr<vector<shared_ptr<Vertex>>> VoronoiNet::getVerticesOfFace(shared_ptr<geometry::CNPoint2D> point)
{
    shared_ptr<vector<shared_ptr<Vertex>>> ret = make_shared<vector<shared_ptr<Vertex>>>();
    // locate point
    VoronoiDiagram::Point_2 p(point->x, point->y);
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
                ret->push_back(make_shared<VoronoiDiagram::Vertex>(*(edge->source())));
            }
            edge = edge->next();
        } while (edge != begin);
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
    // locate point
    VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
    // result is face
    if (loc.which() == 0)
    {
        // get site from delaunay
        VoronoiDiagram::Face_handle handle = boost::get<VoronoiDiagram::Face_handle>(loc);
        return make_shared<VoronoiDiagram::Site_2>(handle->dual()->point());
    }
    return nullptr;
}

/**
 * return the obstacle positions
 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > >
 */
shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> VoronoiNet::getObstaclePositions()
{
    auto ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    for (auto ob : *this->alloClusteredObsWithMe)
    {
        // obstacles have negative ids
        if (ob->id < 0)
        {
            ret->push_back(ob->getPoint());
        }
    }
    return ret;
}

/**
 * return the site positions
 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > >
 */
shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> VoronoiNet::getOpponentPositions()
{
    auto ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    for (auto cluster : *alloClusteredObsWithMe)
    {
        if (*reinterpret_cast<const int*>(cluster->id->toByteVector().data()) == EntityType::Opponent)
        {
            ret->push_back(cluster->getPoint());
        }
    }
    return ret;
}

/**
 * removes given sites from voronoi net
 * @param sites shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
 */
void VoronoiNet::removeSites(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> sites)
{
    for (int i = 0; i < sites->size(); i++)
    {
        // locate point
        VoronoiDiagram::Point_2 point(sites->at(i)->x, sites->at(i)->y);
        VoronoiDiagram::Locate_result loc = this->voronoi->locate(point);
        // if location is face
        if (VoronoiDiagram::Face_handle *handle = boost::get<VoronoiDiagram::Face_handle>(&loc))
        {
            // delete it from delaunay graph
            ((DelaunayTriangulation) this->voronoi->dual()).remove((*handle)->dual());
        }
    }
}

shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> VoronoiNet::getArtificialObstacles()
{
    return this->artificialObstacles;
}

shared_ptr<vector<shared_ptr<geometry::CNRobot>>> VoronoiNet::getAlloClusteredObsWithMe()
{
    return this->alloClusteredObsWithMe;
}

shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> VoronoiNet::getAdditionalObstacles()
{
    return this->additionalObstacles;
}

/**
 * block opponent penalty area
 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
 */
void VoronoiNet::blockOppPenaltyArea()
{
    auto ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();

    // get cornerpoints of opp penalty area
    auto upLeftCorner = wm->field->posULOppPenaltyArea();
    ret->push_back(upLeftCorner);
    auto lowRightCorner = wm->field->posLROppPenaltyArea();
    ret->push_back(lowRightCorner);
    // get field length and width
    int penaltyWidth = wm->field->getGoalAreaWidth();
    int penaltyLength = wm->field->getGoalAreaLength();
    // calculate missing points
    auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
    ret->push_back(upRightCorner);
    auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
    ret->push_back(lowLeftCorner);
    // calculate point count to block space in width
    int pointCount = penaltyWidth / 500;
    double rest = penaltyWidth % 500;
    double pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x + i * pointDist, lowRightCorner->y);
        auto temp2 = make_shared<geometry::CNPoint2D>(lowLeftCorner->x + i * pointDist, lowLeftCorner->y);
        ret->push_back(temp);
        ret->push_back(temp2);
    }
    // calculate point count to block space in length
    pointCount = penaltyLength / 500;
    rest = penaltyLength % 500;
    pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x, lowRightCorner->y + i * pointDist);
        ret->push_back(temp);
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

/**
 * bolck opponent goal area
 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
 */
void VoronoiNet::blockOppGoalArea()
{
    auto ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();

    // get cornerpoints of opp goal area
    auto upLeftCorner = wm->field->posULOppGoalArea();
    ret->push_back(upLeftCorner);
    auto lowRightCorner = wm->field->posLROppGoalArea();
    ret->push_back(lowRightCorner);
    // get field length and width
    int penaltyWidth = wm->field->getPenaltyAreaWidth();
    int penaltyLength = wm->field->getPenaltyAreaLength();
    // calculate missing points
    auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
    ret->push_back(upRightCorner);
    auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
    ret->push_back(lowLeftCorner);
    // calculate point count to block space in width
    int pointCount = penaltyWidth / 500;
    double rest = penaltyWidth % 500;
    double pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x + i * pointDist, lowRightCorner->y);
        auto temp2 = make_shared<geometry::CNPoint2D>(lowLeftCorner->x + i * pointDist, lowLeftCorner->y);
        ret->push_back(temp);
        ret->push_back(temp2);
    }
    // calculate point count to block space in length
    pointCount = penaltyLength / 500;
    rest = penaltyLength % 500;
    pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x, lowRightCorner->y + i * pointDist);
        ret->push_back(temp);
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

/**
 * bolck own penalty area
 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
 */
void VoronoiNet::blockOwnPenaltyArea()
{
    auto ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();

    // get cornerpoints of own penalty area
    auto upLeftCorner = wm->field->posULOwnPenaltyArea();
    ret->push_back(upLeftCorner);
    auto lowRightCorner = wm->field->posLROwnPenaltyArea();
    ret->push_back(lowRightCorner);
    // get field length and width
    int penaltyWidth = wm->field->getGoalAreaWidth();
    int penaltyLength = wm->field->getGoalAreaLength();
    // calculate missing points
    auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
    ret->push_back(upRightCorner);
    auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
    ret->push_back(lowLeftCorner);
    // calculate point count to block space in width
    int pointCount = penaltyWidth / 500;
    double rest = penaltyWidth % 500;
    double pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = make_shared<geometry::CNPoint2D>(upRightCorner->x - i * pointDist, lowRightCorner->y);
        auto temp2 = make_shared<geometry::CNPoint2D>(upLeftCorner->x - i * pointDist, lowLeftCorner->y);
        ret->push_back(temp);
        ret->push_back(temp2);
    }
    // calculate point count to block space in length
    pointCount = penaltyLength / 500;
    rest = penaltyLength % 500;
    pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y + i * pointDist);
        ret->push_back(temp);
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

/**
 * bolck own goal area
 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
 */
void VoronoiNet::blockOwnGoalArea()
{
    auto ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();

    // get cornerpoints of own goal area
    auto upLeftCorner = wm->field->posULOwnGoalArea();
    ret->push_back(upLeftCorner);
    auto lowRightCorner = wm->field->posLROwnGoalArea();
    ret->push_back(lowRightCorner);
    // get field length and width
    int penaltyWidth = wm->field->getPenaltyAreaWidth();
    int penaltyLength = wm->field->getPenaltyAreaLength();
    // calculate missing points
    auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
    ret->push_back(upRightCorner);
    auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
    ret->push_back(lowLeftCorner);
    // calculate point count to block space in width
    int pointCount = penaltyWidth / 500;
    double rest = penaltyWidth % 500;
    double pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = make_shared<geometry::CNPoint2D>(upRightCorner->x - i * pointDist, lowRightCorner->y);
        auto temp2 = make_shared<geometry::CNPoint2D>(upLeftCorner->x - i * pointDist, lowLeftCorner->y);
        ret->push_back(temp);
        ret->push_back(temp2);
    }
    // calculate point count to block space in length
    pointCount = penaltyLength / 500;
    rest = penaltyLength % 500;
    pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = make_shared<geometry::CNPoint2D>(upRightCorner->x, lowRightCorner->y + i * pointDist);
        ret->push_back(temp);
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

/**
 * bolck 3 meters around the ball
 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
 */
void VoronoiNet::blockThreeMeterAroundBall()
{
    auto ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    // get ball pos
    auto alloBall = wm->ball->getAlloBallPosition();
    if (alloBall == nullptr)
    {
        return;
    }
    // 3m radius
    int radius = 3000;
    // calculate perimeter
    double perimeter = 2 * M_PI * radius;
    // calculate point count on perimeter
    int pointCount = perimeter / 500 + 0.5;
    // calculate agle between slices of the circle
    double slice = 2 * M_PI / pointCount;
    // calculate points to block area
    for (int i = 0; i < pointCount; i++)
    {
        // calculate point
        double angle = slice * i;
        int newX = (int)(alloBall->x + radius * cos(angle));
        int newY = (int)(alloBall->y + radius * sin(angle));
        ret->push_back(make_shared<geometry::CNPoint2D>(newX, newY));
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

/**
 * bolck circle shaped area
 * @param centerPoint shared_ptr<geometry::CNPoint2D>
 * @param radious double
 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
 */
void VoronoiNet::blockCircle(shared_ptr<geometry::CNPoint2D> centerPoint, double radius)
{
    auto ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    // calculate perimeter
    double perimeter = 2 * M_PI * radius;
    // calculate point count on perimeter
    int pointCount = perimeter / 500 + 0.5;
    // calculate agle between slices of the circle
    double slice = 2 * M_PI / pointCount;
    // calculate points to block area
    cout << pointCount << endl;
    for (int i = 0; i < pointCount; i++)
    {
        // calculate point
        double angle = slice * i;
        int newX = (int)(centerPoint->x + radius * cos(angle));
        int newY = (int)(centerPoint->y + radius * sin(angle));
        ret->push_back(make_shared<geometry::CNPoint2D>(newX, newY));
    }
    // insert them into the voronoi diagram
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

/**
 * bolck opponent penalty area
 * @param upLeftCorner shared_ptr<geometry::CNPoint2D>
 * @param lowRightCorner shared_ptr<geometry::CNPoint2D>
 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
 */
void VoronoiNet::blockRectangle(shared_ptr<geometry::CNPoint2D> upLeftCorner, shared_ptr<geometry::CNPoint2D> lowRightCorner)
{
    auto ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();

    ret->push_back(upLeftCorner);
    ret->push_back(lowRightCorner);
    // calculate missing points
    auto upRightCorner = make_shared<geometry::CNPoint2D>(upLeftCorner->x, lowRightCorner->y);
    ret->push_back(upRightCorner);
    auto lowLeftCorner = make_shared<geometry::CNPoint2D>(lowRightCorner->x, upLeftCorner->y);
    ret->push_back(lowLeftCorner);
    int width = upLeftCorner->distanceTo(lowLeftCorner);
    int length = upLeftCorner->distanceTo(upRightCorner);
    // calculate point count to block space in width
    int pointCount = width / 500;
    double rest = width % 500;
    double pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = make_shared<geometry::CNPoint2D>(lowRightCorner->x - i * pointDist, lowRightCorner->y);
        auto temp2 = make_shared<geometry::CNPoint2D>(lowLeftCorner->x - i * pointDist, lowLeftCorner->y);
        ret->push_back(temp);
        ret->push_back(temp2);
    }
    // calculate point count to block space in length
    pointCount = length / 500;
    rest = length % 500;
    pointDist = 500 + rest / 500;
    // calculate points to block area
    for (int i = 1; i < pointCount; i++)
    {
        auto temp = make_shared<geometry::CNPoint2D>(lowLeftCorner->x, upLeftCorner->y + i * pointDist);
        auto temp2 = make_shared<geometry::CNPoint2D>(upLeftCorner->x, upRightCorner->y - i * pointDist);
        ret->push_back(temp);
        ret->push_back(temp2);
    }
    insertAdditionalPoints(ret, EntityType::ArtificialObstacle);
}

/**
 * print the voronoi diagrams sites
 */
void VoronoiNet::printSites()
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
void VoronoiNet::printVertices()
{
    cout << "Voronoi Diagram Vertices: " << endl;
    for (VoronoiDiagram::Vertex_iterator it = this->voronoi->vertices_begin(); it != this->voronoi->vertices_end(); it++)
    {
        cout << it->point() << endl;
    }
}

/**
 * to string
 */
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
}

/* namespace msl */
