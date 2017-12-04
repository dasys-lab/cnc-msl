/*
 * Obstacles.cpp
 *
 *  Created on: Feb 11, 2016
 *      Author: Stefan Jakob
 */

#include "obstaclehandler/Obstacles.h"

#include "MSLFootballField.h"
#include "MSLWorldModel.h"
#include "RawSensorData.h"
#include "Robots.h"
#include "obstaclehandler/AnnotatedObstacleClusterPool.h"
#include "obstaclehandler/SimpleCluster.h"

using std::vector;
using std::shared_ptr;
using std::make_shared;
using nonstd::nullopt;
using nonstd::optional;
using supplementary::InfoTime;
using supplementary::InfoBuffer;
using supplementary::InformationElement;

namespace msl
{

Obstacles::Obstacles(MSLWorldModel *wm, int ringbufferLength)
    : obstaclesInfoBuffer(ringbufferLength)
    , clusteredObstaclesAlloBuffer(ringbufferLength)
    , clusteredObstaclesAlloWithMeBuffer(ringbufferLength)
    , clusteredObstaclesEgoBuffer(ringbufferLength)
    , rawObstaclesAlloBuffer(ringbufferLength)
    , rawObstaclesEgoBuffer(ringbufferLength)
{
    this->wm = wm;

    // read config
    sc = supplementary::SystemConfig::getInstance();
    DENSITY = (*sc)["PathPlanner"]->get<double>("PathPlanner.ObHandler.density", NULL);
    VARIANCE_THRESHOLD = DENSITY * DENSITY;
    TERRITORY_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler.territoryRadius", NULL);
    SIGHT_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler.sightradius",
                                                     NULL); // how far an obstacle can be seen proper
    FIELD_TOL = (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler.fieldTol", NULL);
    POS_CERTAINTY_TH_CLUSTERING = (*sc)["PathPlanner"]->get<double>("PathPlanner.posCertaintyTHClustering", NULL);
    POS_CERTAINTY_HYS = (*sc)["PathPlanner"]->get<double>("PathPlanner.posCertaintyHys", NULL);
    DFLT_OB_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "obstacleRadius", NULL);
    DFLT_ROB_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "teammateRadius", NULL);
    OBSTACLE_MAP_OUT_TOLERANCE =
        (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler", "obstacleMapOutTolerance", NULL);
    LOCALIZATION_SUCCESS_CONFIDENCE = (*sc)["Localization"]->get<double>("Localization", "LocalizationSuccess", NULL);

    // init stuff
    pool = new AnnotatedObstacleClusterPool();
}

Obstacles::~Obstacles()
{
}

double Obstacles::getDistanceToObstacle(geometry::CNPointEgo target) const
{
    auto distScan = wm->rawSensorData->getDistanceScanBuffer().getLastValidContent();

    if (distScan)
    {
        unsigned long startSector = 0;
        double sectorWidth = 2 * M_PI / (*distScan)->size();
        double angleToTarget = target.angleZ();

        // Adjust angle to target
        if (angleToTarget < 0)
        {
            angleToTarget += 2.0 * M_PI;
        }
        //				if (angleToTarget >= 2.0 * Math.PI) {
        //					angleToTarget -= 2.0 * Math.PI;
        //				}

        // Get the number of the sector we start in
        startSector = (unsigned int)floor(angleToTarget / sectorWidth) % (*distScan)->size();

        if ((startSector < 0) || (startSector >= (*distScan)->size()))
        {
            cout << "ERROR: wrong startSector in GetDistanceToObstacle(Point2d)! (" << startSector << " "
                 << angleToTarget << "  " << sectorWidth << " )" << endl;
        }

        return (*distScan)->at(startSector);
    }

    return 0.0;
}

optional<geometry::CNPointEgo> Obstacles::getBiggestFreeGoalAreaMidPoint() const
{
    auto leftPost = wm->field->posLeftOppGoalPost();
    auto rightPost = wm->field->posRightOppGoalPost();
    auto goalMid = wm->field->posOppGoalMid();

    auto ownPosInfo = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValid();
    if (ownPosInfo == nullptr)
        return nullopt;

    auto ownPos = ownPosInfo->getInformation();

    auto tmpPoint = leftPost;

    double dist = 100.0;
    int right = 0, left = 0;
    int i = 0;
    int iter = (int)floor((abs(leftPost.y) + abs(rightPost.y)) / dist);
    while (i <= iter)
    {
        i++;
        auto egoTmp = tmpPoint.toEgo(ownPos);
        double tmpDouble = getDistanceToObstacle(egoTmp);
        // Console.WriteLine(tmpDouble + " " + tmpPoint.Y);
        if ((tmpDouble - 100.0) > egoTmp.length() || tmpDouble > 18000.0)
        {
            if (tmpPoint.y < 0)
            {
                right++;
            }
            else
            {
                left++;
            }
        }
        tmpPoint.y -= dist;
    }
    // Console.WriteLine("wurst left : " + left + " right : " + right);
    if (abs(right - left) < 5)
    {
        return nullopt;
    }
    else if (right > left)
    {
        rightPost.y += 500.0;
        return rightPost.toEgo(ownPos);
    }
    else
    {
        leftPost.y -= 500.0;
        return leftPost.toEgo(ownPos);
    }
}

/**
 * Clusters and saves the given obstacles
 * @param myObstacles the new obstacles to process
 */
void Obstacles::handleObstacles(shared_ptr<const vector<geometry::CNPointEgo>> myObstacles)
{

    auto odoInfo = wm->rawSensorData->getCorrectedOdometryBuffer().getLastValid(); // TODO: or getLast()?
    if (!odoInfo)
    {
        // The check is necessary, because in the simulator you can get obstacles without an correctedOdometryInfo().
        return;
    }

    auto rawEgoObs = make_shared<InformationElement<shared_ptr<const vector<geometry::CNPointEgo>>>>(myObstacles, wm->getTime(), this->maxInfoValidity, 1.0);
    this->rawObstaclesEgoBuffer.add(rawEgoObs);
    auto odo = odoInfo->getInformation();
    auto ownPosOdo = geometry::CNPositionAllo(odo.position.x, odo.position.y, odo.position.angle);
    auto alloObs = make_shared<vector<geometry::CNPointAllo>>();
    for(auto obs : *myObstacles)
    {
    	alloObs->push_back(obs.toAllo(ownPosOdo));
    }
    auto rawAlloObs =
                make_shared<InformationElement<shared_ptr<const vector<geometry::CNPointAllo>>>>(alloObs, wm->getTime(), this->maxInfoValidity, 1.0);
    this->rawObstaclesAlloBuffer.add(rawAlloObs);

    // SETUP
    auto annotatedObstacles = setupAnnotatedObstacles(myObstacles, odo);
    // CLUSTERING
    auto clusteredObstacles = clusterAnnotatedObstacles(annotatedObstacles);
    // CREATE DATASTRUCTURES FOR WM, DELAUNAY-GENERATOR, ETC.

    auto newObsClustersAllo = make_shared<vector<CNRobotAllo>>();
    auto newObsClustersAlloWithMe = make_shared<vector<CNRobotAllo>>();

    auto newObsClusteredEgo = make_shared<vector<CNRobotEgo>>();

    auto newOppEgo = make_shared<vector<geometry::CNPointEgo>>();
    auto newOppAllo = make_shared<vector<geometry::CNPointAllo>>();
    auto newTeammatesEgo = make_shared<vector<geometry::CNPointEgo>>();
    auto newTeammatesAllo = make_shared<vector<geometry::CNPointAllo>>();

    for (unsigned long i = 0; i < clusteredObstacles->size(); ++i)
    {
        auto clusterInfo = CNRobotAllo();
        auto current = clusteredObstacles->at(i);

        clusterInfo.id = current->ident;
        clusterInfo.radius = current->radius;
        clusterInfo.position = geometry::CNPositionAllo(current->x, current->y, current->angle);
        clusterInfo.velocity = geometry::CNVecAllo(current->velX, current->velY);
        clusterInfo.opposer = current->opposer;
        clusterInfo.supporter = current->supporter;
        clusterInfo.certainty = current->certainty;
        clusterInfo.rotationVel = current->rotationVel;

        // Store for buffers
        if (current->ident != wm->getOwnId())
        {
            newObsClustersAllo->push_back(clusterInfo); // allo w/o me
        }
        newObsClustersAlloWithMe->push_back(clusterInfo);            // allo with me
        newObsClusteredEgo->push_back(clusterInfo.toEgo(ownPosOdo)); // ego

        auto curAlloPoint = geometry::CNPointAllo(current->x, current->y);
        auto curEgoPoint = curAlloPoint.toEgo(ownPosOdo);

        if (current->ident == EntityType::Opponent)
        {
            // it is not a teammate
            if (wm->field->isInsideField(curAlloPoint, FIELD_TOL))
            {
                newOppAllo->push_back(curAlloPoint);
                // egocentric obstacles, which are inside the field and do not belong to our team
                newOppEgo->push_back(curEgoPoint);
            }
        }
        else if (current->ident != wm->getOwnId())
        {
            newTeammatesEgo->push_back(curEgoPoint);
            newTeammatesAllo->push_back(curAlloPoint);
        }
    }

    // create info elements for buffers

    // allo
    auto obstaclesWithMeInfo = make_shared<InformationElement<shared_ptr<const vector<CNRobotAllo>>>>(
        newObsClustersAlloWithMe, wm->getTime(), this->maxInfoValidity, 1.0);
    auto obstaclesInfo = make_shared<InformationElement<shared_ptr<const vector<CNRobotAllo>>>>(
        newObsClustersAllo, wm->getTime(), this->maxInfoValidity, 1.0);

    // ego
    auto obstaclesEgoInfo = make_shared<InformationElement<shared_ptr<const vector<CNRobotEgo>>>>(
        newObsClusteredEgo, wm->getTime(), this->maxInfoValidity, 1.0);

    // Store Data

    // allo
    this->clusteredObstaclesAlloBuffer.add(obstaclesInfo);
    this->clusteredObstaclesAlloWithMeBuffer.add(obstaclesWithMeInfo);

    // ego
    this->clusteredObstaclesEgoBuffer.add(obstaclesEgoInfo);

    wm->robots->opponents.integrateOpponentsEgoClustered(newOppEgo);
    wm->robots->opponents.integrateOpponentsAlloClustered(newOppAllo);
    wm->robots->teammates.integrateTeammatesEgoClustered(newTeammatesEgo);
    wm->robots->teammates.integrateTeammatesAlloClustered(newTeammatesAllo);
    pool->reset();
}

double Obstacles::getObstacleRadius() const
{
    return this->DFLT_OB_RADIUS;
}

/* TODO: remove?
shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
Obstacles::clusterPoint2D(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> obstacles, double varianceThreshold)
{
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> retList =
        make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    shared_ptr<vector<shared_ptr<SimpleCluster>>> clusterList = make_shared<vector<shared_ptr<SimpleCluster>>>();
    bool mergedCluster = true;

    // init cluster objects
    for (int i = 0; i < obstacles->size(); ++i)
    {
        clusterList->push_back(make_shared<SimpleCluster>(obstacles->at(i)));
    }

    while (mergedCluster)
    {
        // find the two nearest clusters
        int fstClusterId = -1;
        int sndClusterId = -1;
        double minDist = numeric_limits<double>::max();
        double curDist = 0;
        for (int i = 0; i < clusterList->size(); ++i)
        {
            for (int j = 0; j < i; j++)
            {
                // check dist
                curDist = clusterList->at(i)->distanceTo(clusterList->at(j));
                if (curDist < minDist)
                {
                    fstClusterId = i;
                    sndClusterId = j;
                    minDist = curDist;
                }
            }
        }

        // check if variance after merging is below VARIANCE_THRESHOLD
        if (fstClusterId != -1)
        {
            mergedCluster =
                clusterList->at(fstClusterId)->checkAndMerge(clusterList->at(sndClusterId), varianceThreshold);
            if (mergedCluster)
            {
                clusterList->erase(clusterList->begin() + sndClusterId);
            }
        }
        else
        {
            mergedCluster = false;
        }
    }

    for (int i = 0; i < clusterList->size(); ++i)
    {
        retList->push_back(make_shared<geometry::CNPoint2D>(clusterList->at(i)->x, clusterList->at(i)->y));
    }

    return retList;
}
*/

shared_ptr<vector<AnnotatedObstacleCluster *>>
Obstacles::clusterAnnotatedObstacles(shared_ptr<vector<AnnotatedObstacleCluster *>> clusterArray)
{
    bool mergedCluster = true;
    while (mergedCluster)
    {
        // find the two nearest mergeable clusters
        long firstClusterId = -1;
        long secondClusterId = -1;

        double minDist = numeric_limits<double>::max();
        double curDist = 0;
        for (unsigned long i = 0; i < clusterArray->size(); ++i)
        {
            auto ith = clusterArray->at(i);
            for (int j = 0; j < i; ++j)
            {
                auto jth = clusterArray->at(j);
                if ((ith->ident == EntityType::Opponent || jth->ident == EntityType::Opponent) &&
                    std::find(ith->supporter->begin(), ith->supporter->end(), jth->ident) == ith->supporter->end() &&
                    std::find(jth->supporter->begin(), jth->supporter->end(), ith->ident) == jth->supporter->end())
                {
                    // mergeable, check dist
                    curDist = ith->distanceTo(jth);
                    if (curDist < minDist)
                    {
                        firstClusterId = i;
                        secondClusterId = j;
                        minDist = curDist;
                    }
                }
            }
        }
        // check if variance after merging is below VARIANCE_THRESHOLD
        if (firstClusterId == -1)
        {
            // TODO: flag is set to false AND break is called???
            mergedCluster = false;
            break;
        }

        mergedCluster =
            clusterArray->at(firstClusterId)->checkAndMerge(clusterArray->at(secondClusterId), VARIANCE_THRESHOLD);
        if (mergedCluster)
        {
            clusterArray->erase(clusterArray->begin() + secondClusterId);
        }
    }

    // copy array
    auto newClusterArray = make_shared<vector<AnnotatedObstacleCluster *>>(*clusterArray);

    // sort clusters
    std::sort(newClusterArray->begin(), newClusterArray->end(), AnnotatedObstacleCluster::compareTo);

    return newClusterArray;
}

shared_ptr<vector<AnnotatedObstacleCluster *>>
Obstacles::setupAnnotatedObstacles(shared_ptr<const vector<geometry::CNPointEgo>> ownObs,
                                   msl_sensor_msgs::CorrectedOdometryInfo myOdo)
{
    auto clusters = make_shared<vector<AnnotatedObstacleCluster *>>();

    AnnotatedObstacleCluster *obs = nullptr;
    int velX = 0;
    int velY = 0;
    for (auto swmd : wm->robots->getSharedWmDataBuffersMap())
    {
        auto information = swmd.second->getLast();
        if (information == nullptr)
        {
            continue;
        }

        auto currentRobot = information->getInformation();
        auto robotOdo = currentRobot.odom;

        /* Ignore every robot, which:
         * - is unlocalised
         * - messages are old (25 ms)
         * - I am (the obstacles from the WM are newer)*/
        if (robotOdo.certainty < 0.8 || currentRobot.senderID == wm->getOwnId() ||
            information->getCreationTime() + 250000000 < wm->getTime())
        {
            continue;
        }

        // add all obstacles seen by curRobot
        for (auto ob : currentRobot.obstacles)
        {
            // Nobody knows the positions of obstacles around me better than I do!
            if (geometry::distance(ob.x, ob.y, myOdo.position.x, myOdo.position.y) < TERRITORY_RADIUS)
            {
                continue;
            }

            if (wm->field->isInsideField(ob.x, ob.y, OBSTACLE_MAP_OUT_TOLERANCE))
            {
                obs = AnnotatedObstacleCluster::getNew(this->pool);
                obs->init((int)round(ob.x), (int)round(ob.y), // pos
                          DFLT_OB_RADIUS, EntityType::Opponent, swmd.first);
                clusters->push_back(obs);
            }
        }

        /* add the curRobot itself as an obstacle, to identify teammates */

        // Convert ego motion angle to allo motion angle
        double alloMotionAngle = geometry::normalizeAngle(robotOdo.position.angle + robotOdo.motion.angle);

        // Calc x and y the velocity
        velX = (int)(cos(alloMotionAngle) * robotOdo.motion.translation + 0.5);
        velY = (int)(sin(alloMotionAngle) * robotOdo.motion.translation + 0.5);

        // predict the position along the translation
        double seconds = (double)(wm->getTime() - information->getCreationTime()) / 1000000000.0;

        obs = AnnotatedObstacleCluster::getNew(this->pool);
        obs->init((int)round(robotOdo.position.x + seconds * velX),
                  (int)round(robotOdo.position.y + seconds * velY), // pos
                  DFLT_ROB_RADIUS, velX, velY,                      // velocity
                  swmd.first, swmd.first);
        clusters->push_back(obs);
    }

    /* add my own obstacles from the worldmodel (they are egocentric :-( ) */

    auto ownPos = geometry::CNPositionAllo(myOdo.position.x, myOdo.position.y, myOdo.position.angle);

    for (unsigned long i = 0; i < ownObs->size(); ++i)
    {
        auto curPoint = ownObs->at(i).toAllo(ownPos);
        if (wm->field->isInsideField(curPoint, OBSTACLE_MAP_OUT_TOLERANCE))
        {
            obs = AnnotatedObstacleCluster::getNew(this->pool);
            obs->init((int)(curPoint.x + 0.5), (int)(curPoint.y + 0.5), DFLT_OB_RADIUS, EntityType::Opponent,
                      wm->getOwnId());
            clusters->push_back(obs);
        }
    }

    /* add my own position: */

    // Convert ego motion angle to allo motion angle
    double alloMotAngle = geometry::normalizeAngle(myOdo.position.angle + myOdo.motion.angle);

    velX = (int)round(myOdo.motion.translation * cos(alloMotAngle));
    velY = (int)round(myOdo.motion.translation * sin(alloMotAngle));
    obs = AnnotatedObstacleCluster::getNew(this->pool);
    obs->init((int)(myOdo.position.x + 0.5), (int)(myOdo.position.y + 0.5), myOdo.position.angle, DFLT_ROB_RADIUS, velX,
              velY, myOdo.motion.rotation, myOdo.position.certainty, wm->getOwnId(), wm->getOwnId());
    clusters->push_back(obs);

    std::sort(clusters->begin(), clusters->end(), AnnotatedObstacleCluster::compareTo);

    return clusters;
}

/* TODO: remove?
void Obstacles::processNegSupporter(geometry::CNPositionAllo myPosition)
{
    double curAngle = 0.0;
    double curAngle2 = 0.0;
    double dangle = 0.0;
    double dangle2 = 0.0;
    double left = 0.0;
    double right = 0.0;
    double left2 = 0.0;
    double right2 = 0.0;
    double curDist = 0.0;
    double curDist2 = 0.0;
    shared_ptr<geometry::CNPoint2D> curPoint = make_shared<geometry::CNPoint2D>();
    shared_ptr<geometry::CNPoint2D> curPoint2 = make_shared<geometry::CNPoint2D>();
    bool sightIsBlocked;

    for (pair<int, shared_ptr<InfoBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>>> curRobot :
         wm->robots->sharedWorldModelData)
    {
        // cout << "Robot: " << curRobot.first << endl;
        // Ignore every robot, which is:
        // - unlocalised
        // - myself
        //
        shared_ptr<msl_sensor_msgs::SharedWorldInfo> currentRobot = curRobot.second->getLast()->getInformation();
        if (currentRobot == nullptr || currentRobot->odom.certainty < 0.8 || currentRobot->senderID == wm->getOwnId())
        {
            //				 	cout << "Skip" << endl;
            continue;
        }

        for (int i = 0; i < newClusterArray->size(); ++i)
        {
            // cout << "Cluster: " << (newClusterArray->at(i)->x /1000.0) <<  " " << (newClusterArray->at(i)->y /
            // 1000.0) << endl;
            // continue, if the curRobot is a supporter of the curCluster or
            // the curCluster is out of the sight of the curRobot or
            // the curCluster is near me (<TERRITORY_RADIUS) so nobody was allowed to merg
            if (newClusterArray->at(i)->distanceTo(myPosition) < TERRITORY_RADIUS)
            {
                continue;
            }
            curDist = newClusterArray->at(i)->distanceTo(make_shared<geometry::CNPosition>(
                currentRobot->odom.position.x, currentRobot->odom.position.y, currentRobot->odom.position.angle));
            if (curDist > SIGHT_RADIUS)
            {
                // cout << "Too far away!" << endl;
                continue;
            }

            if (find(newClusterArray->at(i)->supporter->begin(), newClusterArray->at(i)->supporter->end(),
                     currentRobot->senderID) != newClusterArray->at(i)->supporter->end())
            {
                // cout << "I am supporter!" << endl;
                continue;
            }

            curPoint = make_shared<geometry::CNPoint2D>(newClusterArray->at(i)->x - currentRobot->odom.position.x,
                                                        newClusterArray->at(i)->y - currentRobot->odom.position.y);
            curAngle = atan2(curPoint->y, curPoint->x);
            dangle = abs(asin(DENSITY / curDist));

            // normalize angles
            left = geometry::normalizeAngle(curAngle + dangle);
            right = geometry::normalizeAngle(curAngle - dangle);
            //
            //									cout << "Cluster Angels: \n\tleft: " <<
            //(left
            //*
            // 180)
            ///
            // M_PI
            //									                  << "\n\tmiddle: " <<
            //(curAngle
            //*
            // 180)
            ///
            // M_PI
            //									                  << "\n\tright: " <<
            //(right
            //*
            // 180)
            ///
            // M_PI
            //									                  << "\n\tdangle: " <<
(dangle
            //*
            // 180)
            ///
            // M_PI
            //<<
            // endl;

            sightIsBlocked = false;

            // Für jedes Obstacle überprüfen, ob es im Weg steht
            if (currentRobot->obstacles.size() > 0) // != nullptr)
            {
                for (int j = 0; j < currentRobot->obstacles.size(); ++j)
                {
                    cout << "Own Obstacle: " << (currentRobot->obstacles.at(j).x / 1000.0) << " "
                         << (currentRobot->obstacles.at(j).y / 1000.0) << endl;

                    curPoint2 = make_shared<geometry::CNPoint2D>(
                        currentRobot->obstacles.at(j).x - currentRobot->odom.position.x,
                        currentRobot->obstacles.at(j).y - currentRobot->odom.position.y);

                    curDist2 = curPoint2->length();
                    if (curDist2 < curDist)
                    { // the curPoint2 is closer then curPoint
                        curAngle2 = atan2(curPoint2->y, curPoint2->x);
                        dangle2 = abs(asin(DENSITY / curDist2));

                        // normalize angles
                        left2 = geometry::normalizeAngle(curAngle2 + dangle2);
                        right2 = geometry::normalizeAngle(curAngle2 - dangle2);
                        //
                        //
cout
                        //<<
                        //"Own Obstacle Angels: \n\tleft: " <<
                        //(left2 * 180) / M_PI
                        //
<<
                        //"\n\tmiddle: " << (curAngle2
                        //* 180) / M_PI
                        //
<<
                        //"\n\tright: " << (right2 *
                        // 180) / M_PI
                        //
<<
                        //"\n\tdangle: " << (dangle2 *
                        // 180) / M_PI << endl;

                        if (leftOf(left, right2) && !leftOf(left, left2))
                        {
                            //								cout << "Left of Cluster is
                            // behind
                            // Obstacle"
                            //<<
                            // endl;
                            sightIsBlocked = true;
                            break;
                        }

                        if (leftOf(right, right2) && !leftOf(right, left2))
                        {
                            //								cout << "Right of Cluster is
                            // behind
                            // Obstacle"
                            //<<
                            // endl;
                            sightIsBlocked = true;
                            break;
                        }

                        //							cout << "Obstacle does not block the
                        // sight!"
                        //<<
                        // endl;
                    }
                    //						else
                    //						{
                    //							cout << "Own obstacle is too far away!" << endl;
                    //						}
                }
            }
            else
            {
                sightIsBlocked = true;
            }

            // Wenn die Sicht nicht blockiert ist bin ich gegen das Obstacle
            if (!sightIsBlocked)
            {
                newClusterArray->at(i)->opposer->push_back(currentRobot->senderID);
            }
        }
    }

    // Lösche einfach alle Obstacle, die mehr Opposer als Supporter haben
    for (int i = 0; i < newClusterArray->size(); ++i)
    {
        if (newClusterArray->at(i)->supporter->size() < newClusterArray->at(i)->opposer->size())
        {
            cout << "OH: removed this obstacle X: " << newClusterArray->at(i)->x << " Y:" << newClusterArray->at(i)->y
                 << endl;
            newClusterArray->erase(newClusterArray->begin() + i);
        }
    }
}

*/

bool Obstacles::leftOf(double angle1, double angle2) const
{
    if ((angle1 > 0.0 && angle2 > 0.0) || (angle1 < 0.0 && angle2 < 0.0))
    {
        if (angle1 > angle2)
        {
            return true;
        }
    }
    else
    {
        if (angle1 > 0)
        {
            if (angle1 - angle2 < M_PI)
            {
                return true;
            }
        }
        else
        {
            if (angle2 - angle1 > M_PI)
            {
                return true;
            }
        }
    }
    return false;
}

/* ===== Buffer Access ===== */

// Raw Info

const InfoBuffer<shared_ptr<const std::vector<msl_sensor_msgs::ObstacleInfo>>> &
Obstacles::getObstaclesInfoBuffer() const
{
    return this->obstaclesInfoBuffer;
}

// Raw Obstacles

const InfoBuffer<std::shared_ptr<const std::vector<geometry::CNPointAllo>>> &
Obstacles::getRawObstaclesAlloBuffer() const
{
    return this->rawObstaclesAlloBuffer;
}
const InfoBuffer<std::shared_ptr<const std::vector<geometry::CNPointEgo>>> &Obstacles::getRawObstaclesEgoBuffer() const
{
    return this->rawObstaclesEgoBuffer;
}

// Clustered Obstacles

const InfoBuffer<shared_ptr<const vector<CNRobotAllo>>> &Obstacles::getClusteredObstaclesAlloBuffer() const
{
    return this->clusteredObstaclesAlloBuffer;
}

const InfoBuffer<shared_ptr<const vector<CNRobotAllo>>> &
Obstacles::getClusteredObstaclesAlloWithMeBuffer() const
{
    return this->clusteredObstaclesAlloWithMeBuffer;
}

const InfoBuffer<shared_ptr<const vector<CNRobotEgo>>> &Obstacles::getClusteredObstaclesEgoBuffer() const
{
    return this->clusteredObstaclesEgoBuffer;
}
/*

shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Obstacles::getEgoVisionObstaclePoints(int index)
{
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    auto x = obstacles.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    msl_sensor_msgs::ObstacleInfo current;
    for (int i = 0; i < x->getInformation()->size(); i++)
    {
        current = x->getInformation()->at(i);
        ret->push_back(make_shared<geometry::CNPoint2D>(current.x, current.y));
    }
    return ret;
}

shared_ptr<vector<msl_sensor_msgs::ObstacleInfo>> Obstacles::getEgoVisionObstacles(int index)
{
    auto x = obstacles.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}
*/
void Obstacles::processWorldModelData(msl_sensor_msgs::WorldModelData &data)
{
    InfoTime time = wm->getTime();

    auto infoContent = make_shared<const vector<msl_sensor_msgs::ObstacleInfo>>(data.obstacles);

    // Raw Info
    auto o = make_shared<InformationElement<shared_ptr<const vector<msl_sensor_msgs::ObstacleInfo>>>>(
        infoContent, time, this->maxInfoValidity, 1.0); // TODO: certainty?
    this->obstaclesInfoBuffer.add(o);

    auto obstaclesPtr = make_shared<vector<geometry::CNPointEgo>>(data.obstacles.size());
    for (auto ob : data.obstacles)
    {
        obstaclesPtr->push_back(geometry::CNPointEgo(ob.x, ob.y));
    }
    handleObstacles(obstaclesPtr);
}

} /* namespace msl */
