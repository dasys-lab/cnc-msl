#include "MSLFootballField.h"
#include "MSLWorldModel.h"
#include "RawSensorData.h"
#include "Robots.h"
#include "obstaclehandler/AnnotatedObstacleClusterPool.h"
#include "obstaclehandler/SimpleCluster.h"
#include <obstaclehandler/Obstacles.h>
#include <engine/AlicaEngine.h>
#include <msl/robot/IntRobotID.h>

namespace msl
{

Obstacles::Obstacles(MSLWorldModel *wm, int ringbufferLength)
    : obstacles(ringbufferLength)
    , obstaclesAlloClustered(ringbufferLength)
    , obstaclesEgoClustered(ringbufferLength)
    , obstaclesAlloClusteredWithMe(ringbufferLength)
{
    this->wm = wm;
    sc = supplementary::SystemConfig::getInstance();
    DENSITY = (*sc)["PathPlanner"]->get<double>("PathPlanner.ObHandler.density", NULL);
    VARIANCE_THRESHOLD = DENSITY * DENSITY;
    TERRITORY_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler.territoryRadius", NULL);
    SIGHT_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler.sightradius", NULL); // how far an obstacle can be seen proper
    FIELD_TOL = (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler.fieldTol", NULL);
    POS_CERTAINTY_TH_CLUSTERING = (*sc)["PathPlanner"]->get<double>("PathPlanner.posCertaintyTHClustering", NULL);
    POS_CERTAINTY_HYS = (*sc)["PathPlanner"]->get<double>("PathPlanner.posCertaintyHys", NULL);
    DFLT_OB_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "obstacleRadius", NULL);
    DFLT_ROB_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "teammateRadius", NULL);
    OBSTACLE_MAP_OUT_TOLERANCE = (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler", "obstacleMapOutTolerance", NULL);
    LOCALIZATION_SUCCESS_CONFIDENCE = (*sc)["Localization"]->get<double>("Localization", "LocalizationSuccess", NULL);
    clusterArray = make_shared<vector<AnnotatedObstacleCluster *>>();
    newClusterArray = make_shared<vector<AnnotatedObstacleCluster *>>();
    pool = new AnnotatedObstacleClusterPool();
}

Obstacles::~Obstacles()
{
}

double Obstacles::getDistanceToObstacle(shared_ptr<geometry::CNPoint2D> target)
{
    auto distScan = wm->rawSensorData->getDistanceScan();

    if (distScan != nullptr)
    {
        int startSector = 0;
        double sectorWidth = 2 * M_PI / distScan->size();
        double angleToTarget = target->angleTo();

        // Adjust angle to target
        if (angleToTarget < 0)
        {
            angleToTarget += 2.0 * M_PI;
        }
        //				if (angleToTarget >= 2.0 * Math.PI) {
        //					angleToTarget -= 2.0 * Math.PI;
        //				}

        // Get the number of the sector we start in
        startSector = (((int)floor((angleToTarget / sectorWidth)) % (int)distScan->size()));

        if ((startSector < 0) || (startSector >= distScan->size()))
        {
            cout << "ERROR: wrong startSector in GetDistanceToObstacle(Point2d)! (" << startSector << " " << angleToTarget << "  " << sectorWidth << " )"
                 << endl;
        }

        return distScan->at(startSector);
    }

    return 0.0;
}

shared_ptr<geometry::CNPoint2D> Obstacles::getBiggestFreeGoalAreaMidPoint()
{
    auto leftPost = wm->field->posLeftOppGoalPost();
    auto rightPost = wm->field->posRightOppGoalPost();
    auto goalMid = wm->field->posOppGoalMid();

    auto ownPos = wm->rawSensorData->getOwnPositionVision();
    if (ownPos == nullptr)
        return nullptr;

    shared_ptr<geometry::CNPoint2D> tmpPoint = make_shared<geometry::CNPoint2D>(leftPost->x, leftPost->y);

    double dist = 100.0;
    int right = 0, left = 0;
    int i = 0;
    int iter = (int)floor((abs(leftPost->y) + abs(rightPost->y)) / dist);
    while (i <= iter)
    {
        i++;
        auto egoTmp = tmpPoint->alloToEgo(*ownPos);
        double tmpDouble = getDistanceToObstacle(egoTmp);
        // Console.WriteLine(tmpDouble + " " + tmpPoint.Y);
        if ((tmpDouble - 100.0) > egoTmp->length() || tmpDouble > 18000.0)
        {
            if (tmpPoint->y < 0)
            {
                right++;
            }
            else
            {
                left++;
            }
        }
        tmpPoint->y -= dist;
    }
    // Console.WriteLine("wurst left : " + left + " right : " + right);
    if (abs(right - left) < 5)
    {
        return nullptr;
    }
    else if (right > left)
    {
        rightPost->y += 500.0;
        return rightPost->alloToEgo(*ownPos);
    }
    else
    {
        leftPost->y -= 500.0;
        return leftPost->alloToEgo(*ownPos);
    }
}

void Obstacles::handleObstacles(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> myObstacles)
{

    auto myOdo = wm->rawSensorData->getCorrectedOdometryInfo();
    if (!myOdo)
    {
        // The check is necessary, because in the simulator you can get obstacles without an correctedOdometryInfo().
        return;
    }

    // SETUP
    setupAnnotatedObstacles(myObstacles, myOdo);
    // CLUSTERING
    clusterAnnotatedObstacles();
    // CREATE DATASTRUCTURES FOR WM, DELAUNAY-GENERATOR, ETC.
    shared_ptr<vector<shared_ptr<geometry::CNRobot>>> newObsClustersAllo = make_shared<vector<shared_ptr<geometry::CNRobot>>>();
    shared_ptr<vector<shared_ptr<geometry::CNRobot>>> newObsClustersAlloWithMe = make_shared<vector<shared_ptr<geometry::CNRobot>>>();
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> newOppEgo = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> newOppAllo = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> newTeammatesEgo = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> newTeammatesAllo = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();

    shared_ptr<geometry::CNPoint2D> curAlloPoint = nullptr;
    shared_ptr<geometry::CNPoint2D> curEgoPoint = nullptr;
    for (int i = 0; i < newClusterArray->size(); ++i)
    {
        shared_ptr<geometry::CNRobot> clusterInfo = make_shared<geometry::CNRobot>();
        auto current = newClusterArray->at(i);
        clusterInfo->id = wm->getEngine()->getID(current->ident);;
        clusterInfo->radius = current->radius;
        clusterInfo->x = current->x;
        clusterInfo->y = current->y;
        clusterInfo->theta = current->angle;
        clusterInfo->velocityX = current->velX;
        clusterInfo->velocityY = current->velY;
        clusterInfo->opposer = current->opposer;
        clusterInfo->supporter = current->supporter;
        clusterInfo->certainty = current->certainty;
        clusterInfo->rotation = current->rotation;
        if (newClusterArray->at(i)->ident != dynamic_cast<const msl::robot::IntRobotID *>(this->wm->getOwnId())->getId())
        {
            newObsClustersAllo->push_back(clusterInfo);
        }
        newObsClustersAlloWithMe->push_back(clusterInfo);

        curAlloPoint = make_shared<geometry::CNPoint2D>(newClusterArray->at(i)->x, newClusterArray->at(i)->y);
        curEgoPoint = curAlloPoint->alloToEgo(*(make_shared<geometry::CNPosition>(wm->rawSensorData->getCorrectedOdometryInfo()->position.x,
                                                                                  wm->rawSensorData->getCorrectedOdometryInfo()->position.y,
                                                                                  wm->rawSensorData->getCorrectedOdometryInfo()->position.angle)));

        if (newClusterArray->at(i)->ident == EntityType::Opponent)
        {
            // it is not a teammate
            if (wm->field->isInsideField(curAlloPoint, FIELD_TOL))
            {
                newOppAllo->push_back(curAlloPoint);
                // egocentric obstacles, which are inside the field and do not belong to our team
                newOppEgo->push_back(curEgoPoint);
            }
        }
        else if (newClusterArray->at(i)->ident != dynamic_cast<const msl::robot::IntRobotID *>(this->wm->getOwnId())->getId())
        {
            newTeammatesEgo->push_back(curEgoPoint);
            newTeammatesAllo->push_back(curAlloPoint);
        }
    }
    // change the vNet references to the new lists
    shared_ptr<InformationElement<vector<shared_ptr<geometry::CNRobot>>>> owm =
        make_shared<InformationElement<vector<shared_ptr<geometry::CNRobot>>>>(newObsClustersAlloWithMe, wm->getTime());
    owm->certainty = 1;
    shared_ptr<InformationElement<vector<shared_ptr<geometry::CNRobot>>>> o =
        make_shared<InformationElement<vector<shared_ptr<geometry::CNRobot>>>>(newObsClustersAllo, wm->getTime());
    o->certainty = 1;
    this->obstaclesAlloClustered.add(o);
    this->obstaclesAlloClusteredWithMe.add(owm);
    wm->robots->opponents.processOpponentsEgoClustered(newOppEgo);
    wm->robots->opponents.processOpponentsAlloClustered(newOppAllo);
    wm->robots->teammates.processTeammatesEgoClustered(newTeammatesEgo);
    wm->robots->teammates.processTeammatesAlloClustered(newTeammatesAllo);
    pool->reset();
}

double Obstacles::getObstacleRadius()
{
    return this->DFLT_OB_RADIUS;
}

shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Obstacles::clusterPoint2D(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> obstacles,
                                                                              double varianceThreshold)
{
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> retList = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
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
            mergedCluster = clusterList->at(fstClusterId)->checkAndMerge(clusterList->at(sndClusterId), varianceThreshold);
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

void Obstacles::clusterAnnotatedObstacles()
{
    bool mergedCluster = true;
    while (mergedCluster)
    {
        // find the two nearest mergeable clusters
        int fstClusterId = -1;
        int sndClusterId = -1;
        double minDist = numeric_limits<double>::max();
        double curDist = 0;
        for (int i = 0; i < this->clusterArray->size(); ++i)
        {
            auto ith = this->clusterArray->at(i);
            for (int j = 0; j < i; ++j)
            {
                auto jth = this->clusterArray->at(j);
                if ((ith->ident == EntityType::Opponent || jth->ident == EntityType::Opponent) &&
                    std::find(ith->supporter->begin(), ith->supporter->end(), jth->ident) == ith->supporter->end() &&
                    std::find(jth->supporter->begin(), jth->supporter->end(), ith->ident) == jth->supporter->end())
                {
                    // mergeable, check dist
                    curDist = ith->distanceTo(jth);
                    if (curDist < minDist)
                    {
                        fstClusterId = i;
                        sndClusterId = j;
                        minDist = curDist;
                    }
                }
            }
        }
        // check if variance after merging is below VARIANCE_THRESHOLD
        if (fstClusterId == -1)
        {
            mergedCluster = false;
            break;
        }

        mergedCluster = this->clusterArray->at(fstClusterId)->checkAndMerge(this->clusterArray->at(sndClusterId), VARIANCE_THRESHOLD);
        if (mergedCluster)
        {
            this->clusterArray->erase(this->clusterArray->begin() + sndClusterId);
        }
    }

    this->newClusterArray->clear();
    this->newClusterArray->reserve(this->clusterArray->size());
    for (int i = 0; i < this->clusterArray->size(); i++)
    {
        this->newClusterArray->push_back(this->clusterArray->at(i));
    }

    std::sort(this->newClusterArray->begin(), this->newClusterArray->end(), AnnotatedObstacleCluster::compareTo);
    this->clusterArray->clear();
}

void Obstacles::setupAnnotatedObstacles(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ownObs, shared_ptr<msl_sensor_msgs::CorrectedOdometryInfo> myOdo)
{
    clusterArray->clear();
    AnnotatedObstacleCluster *obs = nullptr;
    int velX = 0;
    int velY = 0;
    for (auto swmd : wm->robots->sharedWolrdModelData)
    {
        auto information = swmd.second->getLast();
        if (information == nullptr)
        {
            continue;
        }

        auto currentRobot = information->getInformation();
        auto codo = currentRobot->odom;

        /* Ignore every robot, which:
         * - is unlocalised
         * - messages are old (25 ms)
         * - I am (the obstacles from the WM are newer)*/
        if (currentRobot == nullptr || codo.certainty < 0.8 ||
            equal(currentRobot->senderID.id.begin(), currentRobot->senderID.id.end(), wm->getOwnId()->toByteVector().begin()) ||
            swmd.second->getLast()->timeStamp + 250000000 < wm->getTime())
        {
            continue;
        }

        // add all obstacles seen by curRobot
        vector<msl_msgs::Point2dInfo> &curOppList = currentRobot->obstacles;
        for (auto ob : curOppList)
        {
            // Nobody knows the positions of obstacles around me better than I do!
            if (geometry::distance(ob.x, ob.y, myOdo->position.x, myOdo->position.y) < TERRITORY_RADIUS)
            {
                continue;
            }

            if (wm->field->isInsideField(ob.x, ob.y, OBSTACLE_MAP_OUT_TOLERANCE))
            {
                obs = AnnotatedObstacleCluster::getNew(this->pool);
                obs->init(round(ob.x), round(ob.y), // pos
                          DFLT_OB_RADIUS, EntityType::Opponent, dynamic_cast<const msl::robot::IntRobotID *>(swmd.first)->getId());
                clusterArray->push_back(obs);
            }
        }

        /* add the curRobot itself as an obstacle, to identify teammates */

        // Convert ego motion angle to allo motion angle
        double alloMotionAngle = geometry::normalizeAngle(codo.position.angle + codo.motion.angle);

        // Calc x and y the velocity
        velX = (int)(cos(alloMotionAngle) * codo.motion.translation + 0.5);
        velY = (int)(sin(alloMotionAngle) * codo.motion.translation + 0.5);

        // predict the position along the translation
        double seconds = (double)(wm->getTime() - swmd.second->getLast()->timeStamp) / 1000000000.0;

        obs = AnnotatedObstacleCluster::getNew(this->pool);
        obs->init(round(codo.position.x + seconds * velX), round(codo.position.y + seconds * velY), // pos
                  DFLT_ROB_RADIUS, velX, velY,                                                      // velocity
				  dynamic_cast<const msl::robot::IntRobotID *>(swmd.first)->getId(), dynamic_cast<const msl::robot::IntRobotID *>(swmd.first)->getId());
        clusterArray->push_back(obs);
    }

    /* add my own obstacles from the worldmodel (they are egocentric :-( ) */
    shared_ptr<geometry::CNPoint2D> curPoint = make_shared<geometry::CNPoint2D>();
    for (int i = 0; i < ownObs->size(); ++i)
    {
        shared_ptr<geometry::CNPosition> me = make_shared<geometry::CNPosition>(myOdo->position.x, myOdo->position.y, myOdo->position.angle);
        curPoint = ownObs->at(i)->egoToAllo(*me);
        if (wm->field->isInsideField(curPoint, OBSTACLE_MAP_OUT_TOLERANCE))
        {
            obs = AnnotatedObstacleCluster::getNew(this->pool);
            obs->init((int)(curPoint->x + 0.5), (int)(curPoint->y + 0.5), DFLT_OB_RADIUS, EntityType::Opponent,
            		dynamic_cast<const msl::robot::IntRobotID *>(wm->getOwnId())->getId());
            clusterArray->push_back(obs);
        }
    }

    /* add my own position: */

    // Convert ego motion angle to allo motion angle
    double alloMotAngle = geometry::normalizeAngle(myOdo->position.angle + myOdo->motion.angle);

    velX = (int)round(myOdo->motion.translation * cos(alloMotAngle));
    velY = (int)round(myOdo->motion.translation * sin(alloMotAngle));
    obs = AnnotatedObstacleCluster::getNew(this->pool);
    obs->init((int)(myOdo->position.x + 0.5), (int)(myOdo->position.y + 0.5), myOdo->position.angle, DFLT_ROB_RADIUS, velX, velY, myOdo->motion.rotation,
              myOdo->position.certainty, dynamic_cast<const msl::robot::IntRobotID *>(wm->getOwnId())->getId(),
			  dynamic_cast<const msl::robot::IntRobotID *>(wm->getOwnId())->getId());
    clusterArray->push_back(obs);

    std::sort(this->clusterArray->begin(), this->clusterArray->end(), AnnotatedObstacleCluster::compareTo);
}

void Obstacles::processNegSupporter(shared_ptr<geometry::CNPosition> myPosition)
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

    for (pair<const supplementary::IAgentID *, shared_ptr<RingBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>>> curRobot :
         wm->robots->sharedWolrdModelData)
    {
        /* Ignore every robot, which is:
         * - unlocalised
         * - myself
         */
        shared_ptr<msl_sensor_msgs::SharedWorldInfo> currentRobot = curRobot.second->getLast()->getInformation();
        if (currentRobot == nullptr || currentRobot->odom.certainty < 0.8 ||
            equal(currentRobot->senderID.id.begin(), currentRobot->senderID.id.end(), wm->getOwnId()->toByteVector().begin()))
        {
            continue;
        }

        for (int i = 0; i < newClusterArray->size(); ++i)
        {
            // continue, if the curRobot is a supporter of the curCluster or
            // the curCluster is out of the sight of the curRobot or
            // the curCluster is near me (<TERRITORY_RADIUS) so nobody was allowed to merg
            if (newClusterArray->at(i)->distanceTo(myPosition) < TERRITORY_RADIUS)
            {
                continue;
            }
            curDist = newClusterArray->at(i)->distanceTo(
                make_shared<geometry::CNPosition>(currentRobot->odom.position.x, currentRobot->odom.position.y, currentRobot->odom.position.angle));
            if (curDist > SIGHT_RADIUS)
            {
                continue;
            }

            if (find(newClusterArray->at(i)->supporter->begin(), newClusterArray->at(i)->supporter->end(),
                     *reinterpret_cast<const int *>(currentRobot->senderID.id.data())) != newClusterArray->at(i)->supporter->end())
            {
                continue;
            }

            curPoint = make_shared<geometry::CNPoint2D>(newClusterArray->at(i)->x - currentRobot->odom.position.x,
                                                        newClusterArray->at(i)->y - currentRobot->odom.position.y);
            curAngle = atan2(curPoint->y, curPoint->x);
            dangle = abs(asin(DENSITY / curDist));

            // normalize angles
            left = geometry::normalizeAngle(curAngle + dangle);
            right = geometry::normalizeAngle(curAngle - dangle);

            sightIsBlocked = false;

            // Für jedes Obstacle überprüfen, ob es im Weg steht
            if (currentRobot->obstacles.size() > 0) // != nullptr)
            {
                for (int j = 0; j < currentRobot->obstacles.size(); ++j)
                {
                    cout << "Own Obstacle: " << (currentRobot->obstacles.at(j).x / 1000.0) << " " << (currentRobot->obstacles.at(j).y / 1000.0) << endl;

                    curPoint2 = make_shared<geometry::CNPoint2D>(currentRobot->obstacles.at(j).x - currentRobot->odom.position.x,
                                                                 currentRobot->obstacles.at(j).y - currentRobot->odom.position.y);

                    curDist2 = curPoint2->length();
                    if (curDist2 < curDist)
                    { // the curPoint2 is closer then curPoint
                        curAngle2 = atan2(curPoint2->y, curPoint2->x);
                        dangle2 = abs(asin(DENSITY / curDist2));

                        // normalize angles
                        left2 = geometry::normalizeAngle(curAngle2 + dangle2);
                        right2 = geometry::normalizeAngle(curAngle2 - dangle2);

                        if (leftOf(left, right2) && !leftOf(left, left2))
                        {
                            sightIsBlocked = true;
                            break;
                        }

                        if (leftOf(right, right2) && !leftOf(right, left2))
                        {
                            sightIsBlocked = true;
                            break;
                        }
                    }
                }
            }
            else
            {
                sightIsBlocked = true;
            }

            // Wenn die Sicht nicht blockiert ist bin ich gegen das Obstacle
            if (!sightIsBlocked)
            {
            	auto id = this->wm->getEngine()->getIDFromBytes(currentRobot->senderID.id);
            	newClusterArray->at(i)->opposer->push_back(dynamic_cast<const msl::robot::IntRobotID*>(id)->getId());
                //newClusterArray->at(i)->opposer->push_back(*reinterpret_cast<const int *>(currentRobot->senderID.id.data()));
            }
        }
    }

    // Lösche einfach alle Obstacle, die mehr Opposer als Supporter haben
    for (int i = 0; i < newClusterArray->size(); ++i)
    {
        if (newClusterArray->at(i)->supporter->size() < newClusterArray->at(i)->opposer->size())
        {
            cout << "OH: removed this obstacle X: " << newClusterArray->at(i)->x << " Y:" << newClusterArray->at(i)->y << endl;
            newClusterArray->erase(newClusterArray->begin() + i);
            // was missing
            i--;
        }
    }
}

bool Obstacles::leftOf(double angle1, double angle2)
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

shared_ptr<vector<shared_ptr<geometry::CNRobot>>> Obstacles::getAlloObstacles(int index)
{
    auto x = this->obstaclesAlloClustered.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}

shared_ptr<vector<shared_ptr<geometry::CNRobot>>> Obstacles::getEgoObstacles(int index)
{
    // TODO save ego obstacles
    auto ownPos = wm->rawSensorData->getOwnPositionVision();
    if (ownPos == nullptr)
    {
        return nullptr;
    }
    auto x = this->obstaclesAlloClustered.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    shared_ptr<vector<shared_ptr<geometry::CNRobot>>> ret = make_shared<vector<shared_ptr<geometry::CNRobot>>>();
    for (int i = 0; i < this->obstaclesAlloClustered.getLast(index)->getInformation()->size(); i++)
    {
        shared_ptr<geometry::CNRobot> current = this->obstaclesAlloClustered.getLast(index)->getInformation()->at(i);
        double x = current->x - ownPos->x;
        double y = current->y - ownPos->y;

        double angle = atan2(y, x) - ownPos->theta;
        double dist = sqrt(x * x + y * y);

        current->x = cos(angle) * dist;
        current->y = sin(angle) * dist;
        ret->push_back(current);
    }
    return ret;
}

shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Obstacles::getAlloObstaclePoints(int index)
{
    auto x = this->obstaclesAlloClustered.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    for (int i = 0; i < this->obstaclesAlloClustered.getLast(index)->getInformation()->size(); i++)
    {
        shared_ptr<geometry::CNRobot> current = this->obstaclesAlloClustered.getLast(index)->getInformation()->at(i);
        ret->push_back(make_shared<geometry::CNPoint2D>(current->x, current->y));
    }
    return ret;
}

shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Obstacles::getEgoObstaclePoints(int index)
{
    shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
    if (ownPos == nullptr)
    {
        return nullptr;
    }
    auto x = this->obstaclesAlloClustered.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    for (int i = 0; i < this->obstaclesAlloClustered.getLast(index)->getInformation()->size(); i++)
    {
        auto current = this->obstaclesAlloClustered.getLast(index)->getInformation()->at(i);
        ret->push_back(make_shared<geometry::CNPoint2D>(current->x, current->y)->alloToEgo(*ownPos));
    }
    return ret;
}

shared_ptr<vector<shared_ptr<geometry::CNRobot>>> Obstacles::getAlloObstaclesWithMe(int index)
{
    auto x = this->obstaclesAlloClusteredWithMe.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}

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

void Obstacles::processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data)
{
    unsigned long time = wm->getTime();
    shared_ptr<vector<msl_sensor_msgs::ObstacleInfo>> obs = make_shared<vector<msl_sensor_msgs::ObstacleInfo>>(data->obstacles);
    shared_ptr<InformationElement<vector<msl_sensor_msgs::ObstacleInfo>>> o = make_shared<InformationElement<vector<msl_sensor_msgs::ObstacleInfo>>>(obs, time);
    obstacles.add(o);
    if (this->getEgoVisionObstaclePoints() != nullptr)
    {
        handleObstacles(this->getEgoVisionObstaclePoints());
    }
}

} /* namespace msl */
