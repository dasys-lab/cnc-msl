using namespace std;
#include "Plans/GenericBehaviours/CheckGoalKick.h"

/*PROTECTED REGION ID(inccpp1449076008755) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1449076008755) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CheckGoalKick::CheckGoalKick() :
            DomainBehaviour("CheckGoalKick")
    {
        /*PROTECTED REGION ID(con1449076008755) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    CheckGoalKick::~CheckGoalKick()
    {
        /*PROTECTED REGION ID(dcon1449076008755) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CheckGoalKick::run(void* msg)
    {
        /*PROTECTED REGION ID(run1449076008755) ENABLED START*/ //Add additional options here
        cout << "Start run CheckGoalKick <=============================================================" << endl;

        //get own Pos
        auto ownPosition = wm->rawSensorData.getOwnPositionVision();
        ownPos = make_shared < geometry::CNPoint2D > (ownPosition->x, ownPosition->y);

        //get ego ball pos
        egoBallPos = wm->ball.getEgoBallPosition();

        readConfigParameters();
        toleranceAngle = calcToleranceAngle();
//        cout << "toleranceAngle: " << toleranceAngle << " degree: " << toleranceAngle * 180 / M_PI << endl;

        if (checkGoalLine() && checkGoalKeeper() && checkShootPossibility())
        {
            // adding checkGoalKeeper()
            cout << "kicking" << endl;
            kicking();
            this->success = true;
        }

        /*PROTECTED REGION END*/
    }
    void CheckGoalKick::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1449076008755) ENABLED START*/ //Add additional options here
        waitingIter = 0;

        field = msl::MSLFootballField::getInstance();
        alloLeftAimPoint = make_shared < geometry::CNPoint2D
                > (field->FieldLength / 2 + 250, field->posLeftOppGoalPost()->y - wm->ball.getBallDiameter() * 1.5);
        alloMidAimPoint = make_shared < geometry::CNPoint2D > (field->FieldLength / 2 + 250, 0);
        alloRightAimPoint = make_shared < geometry::CNPoint2D
                > (field->FieldLength / 2 + 250, field->posRightOppGoalPost()->y + wm->ball.getBallDiameter() * 1.5);

        //get goal pos
        goalPosLeft = field->posLeftOppGoalPost();
        goalPosRight = field->posRightOppGoalPost();
        goalPosMiddle = field->posOppGoalMid();
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1449076008755) ENABLED START*/ //Add additional methods here
    /*
     * @return true if angle to goal is smaller than tolerance angle
     */
    bool CheckGoalKick::checkGoalLine()
    {

//		cout << "goalPosMiddle: " << goalPosMiddle->toString();
        egoAlignPoint = goalPosMiddle;

        auto ownPos = wm->rawSensorData.getOwnPositionVision();
        auto egoTarget = goalPosMiddle->alloToEgo(*ownPos);

//		cout << "egoTarget: " << egoTarget->toString();
//		cout << "angle to goal: " << egoTarget->angleTo() << " degree: " << egoTarget->angleTo() * 180 / M_PI << endl;

//		cout << "if condition1: " << (egoTarget->angleTo() > (M_PI - toleranceAngle)) << endl;
//		cout << "if condition2: " << (egoTarget->angleTo() > (-M_PI + toleranceAngle)) << endl;

        // if angle is smaller then tolerance angle return true
        if (egoTarget->angleTo() > (M_PI - toleranceAngle) || egoTarget->angleTo() < (-M_PI + toleranceAngle))
        {
            cout << "ChackGoalLine = true" << endl;
            return true;
        }
        cout << "ChackGoalLine = false" << endl;
        return false;
    }

    /*
     * checks if there is an obstacle between robot and goal.
     *
     * @return true if it is possible to shoot at the enemy goal
     */
    bool CheckGoalKick::checkShootPossibility()
    {
        auto obstacles = getObstacles();

//		cout << "Distance ownPos <----> obstaclePos = " << ownPos->distanceTo(obstaclePos) << endl;
//		cout << "Distance ownPos <----> GoalPosMiddle = " << ownPos->distanceTo(goalPosMiddle) << endl;
        // new min distance to obstacle depends on distance to goal
        double distGoal = wm->rawSensorData.getOwnPositionVision()->distanceTo(goalPosMiddle);
//		cout << "ownDistGoal: " << distGoal << endl;

        if (distGoal < farGoalDist)
        {
            ownDistObs = -0.2 * (distGoal / 1000 - minOwnDistGoal / 1000) + closeGoalDist;
        }
        else
        {
            ownDistObs = -0.2 * (distGoal / 1000 - minOwnDistGoal / 1000) + farGoalDist;
        }

        if (obstacles != nullptr)
        {
            bool corridorFree = true;

            for (int i = 0; i < obstacles->size(); i++)
            {
                shared_ptr < geometry::CNPoint2D > obstaclePos = make_shared < geometry::CNPoint2D > (0, 0);
                obstaclePos->x = obstacles->at(i).x;
//				cout << "obstaclePos->x = " << obstaclePos->x << endl;
                obstaclePos->y = obstacles->at(i).y;
//				cout << "obstaclePos->y = " << obstaclePos->y << endl;
//				cout << "Distance obstaclePos <----> GoalPosMiddle = " << obstaclePos->distanceTo(goalPosMiddle)
//						<< endl;

                if (ownPos->distanceTo(obstaclePos) < ownDistObs
                        || obstaclePos->distanceTo(goalPosMiddle) < minObsDistGoal)
                {
                    corridorFree = false;
                }
            }

            return corridorFree;

//            cout << "obstaclePosX1 = " << obstaclePos->x << endl;
//            cout << "obstaclePosY1 = " << obstaclePos->y << endl;
        }
        else
        {
            cout << "No obstacle found. Ready to shoot!" << endl;
            return true;
        }
    }

    void CheckGoalKick::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        minObsDistGoal = (*sc)["GoalKick"]->get<double>("GoalKick.Default.minObsDistGoal", NULL);
        minOwnDistGoal = (*sc)["GoalKick"]->get<double>("GoalKick.Default.minOwnDistGoal", NULL);
        minKickPower = (*sc)["GoalKick"]->get<double>("GoalKick.Default.minKickPower", NULL);
        wheelSpeedLeft = (*sc)["GoalKick"]->get<double>("GoalKick.Default.wheelSpeedLeft", NULL);
        wheelSpeedRight = (*sc)["GoalKick"]->get<double>("GoalKick.Default.wheelSpeedRight", NULL);
        keeperDistGoal = (*sc)["GoalKick"]->get<double>("GoalKick.Default.keeperDistGoal", NULL);

        closeGoalDist = (*sc)["GoalKick"]->get<double>("GoalKick.OwnDistObs.closeGoalDist", NULL);
        farGoalDist = (*sc)["GoalKick"]->get<double>("GoalKick.OwnDistObs.farGoalDist", NULL);

    }

    /*
     * @return distance between goalPosMiddle and goalPosLeft
     */
    double CheckGoalKick::calcToleranceAngle()
    {
        // math!!!
        double a = ownPos->distanceTo(goalPosLeft);
        double b = goalPosLeft->distanceTo(goalPosMiddle);
        double c = ownPos->distanceTo(goalPosRight);
//		cout << "a = " << a << "\nb= " << b << "\nc= " << c << endl;

        return acos((pow(b, 2) - pow(c, 2) - pow(a, 2)) / (-2 * a * c));
    }

    void CheckGoalKick::kicking()
    {
        // min KickPower 1200

        // ball handle
        msl_actuator_msgs::BallHandleCmd bhc;
        auto egoBallPos = wm->ball.getEgoBallPosition();
        double distBall = egoBallPos->length();

        if (distBall < 600)
        {
            bhc.rightMotor = (int8_t) + wheelSpeedRight;
            bhc.leftMotor = (int8_t) + wheelSpeedLeft;
        }
        else
        {
            bhc.rightMotor = 0;
            bhc.leftMotor = 0;
        }

        send(bhc);

        // kicking
        msl_actuator_msgs::KickControl kc;
        kc.extension = 1;
        double distGoal = wm->rawSensorData.getOwnPositionVision()->distanceTo(goalPosMiddle);
        cout << "distance: " << distGoal << endl;
        cout << "kick power: " << (distGoal / 1000 - minOwnDistGoal / 1000) * 100 + 1100 << endl;

        if (wm->ball.haveBall())
        {
            // TODO adapt waiting
            // robot should shoot as early as possible

            if (waitingIter > 30)
            {
                kc.enabled = true;
                // TODO adapt kick power
//				kc.power = minKickPower;
                if (distGoal > 5500)
                {
                    kc.power = (distGoal / 1000 - minOwnDistGoal / 1000) * 100 + 1100;
                }
                else
                {
                    kc.power = minKickPower - 50;
                }
                send(kc);
                waitingIter = 0;
            }
            else
            {
                waitingIter++;
            }
        }
    }
    /**
     *
     * @return true if keeper isn't blocking
     */
    bool CheckGoalKick::checkGoalKeeper()
    {
        // copied and adapt from GoalKick.cpp
        cout << "entered checlGoalKeeper()" << endl;
        auto vNet = wm->pathPlanner.getCurrentVoronoiNet();
        auto ownPosV = wm->rawSensorData.getOwnPositionVision();

        if (ownPos == nullptr || egoBallPos == nullptr || vNet == nullptr)
        {
            return false;
        }

        shared_ptr < geometry::CNPoint2D > alloAimPoint = nullptr;

        double angleTolerance = 0.075;
        auto obs = wm->obstacles.getObstacles();
//		auto obs = getObstacles();
        bool leftBlocked = false;
        bool midBlocked = false;
        bool rightBlocked = false;
        if (obs == nullptr || obs->size() == 0)
        {
            cout << "no obstacle found!" << endl;
            return true;
        }

        for (int i = 0; i < obs->size(); i++)
        {
            shared_ptr < geometry::CNPoint2D > obstaclePoint = make_shared < geometry::CNPoint2D > (0, 0);
            obstaclePoint->x = obs->at(i).x;
            obstaclePoint->y = obs->at(i).y;
            double obsDist = obstaclePoint->alloToEgo(*ownPosV)->distanceTo(goalPosMiddle);
//			cout << "CheckGoalKeeper: obsDist = " << obsDist << endl;
            if (leftBlocked && midBlocked && rightBlocked)
            {
                break;
            }
            if (wm->pathPlanner.corridorCheckBall(
                    vNet, make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y), alloLeftAimPoint,
                    make_shared < geometry::CNPoint2D > (obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPosV))
                    && obsDist < keeperDistGoal)
            {
                leftBlocked = true;
            }
            if (wm->pathPlanner.corridorCheckBall(
                    vNet, make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y), alloMidAimPoint,
                    make_shared < geometry::CNPoint2D > (obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPosV))
                    && obsDist < keeperDistGoal)
            {
                midBlocked = true;
            }
            if (wm->pathPlanner.corridorCheckBall(
                    vNet, make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y), alloRightAimPoint,
                    make_shared < geometry::CNPoint2D > (obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPosV))
                    && obsDist < keeperDistGoal)
            {
                rightBlocked = true;
            }

        }
        if (!leftBlocked && !midBlocked && !rightBlocked)
        {
            return true;
        }
        if (!leftBlocked && alloAimPoint == nullptr)
        {
//			cout << "aimig left" << endl;
            alloAimPoint = alloLeftAimPoint;
        }
        if (!midBlocked && alloAimPoint == nullptr)
        {
//			cout << "aimig mid" << endl;
            alloAimPoint = alloMidAimPoint;
        }
        if (!rightBlocked && alloAimPoint == nullptr)
        {
//            cout << "aimig right" << endl;
            alloAimPoint = alloRightAimPoint;
        }
        if (leftBlocked && midBlocked && rightBlocked && alloAimPoint == nullptr)
        {

//            this->failure = true;
        }
        if (alloAimPoint != nullptr)
        {
            auto egoAimPoint = alloAimPoint->alloToEgo(*wm->rawSensorData.getOwnPositionVision());

            if (fabs(geometry::GeometryCalculator::deltaAngle(egoAimPoint->angleTo(), M_PI)) > angleTolerance)
            {
                cout << "angle: " << fabs(geometry::GeometryCalculator::deltaAngle(egoAimPoint->angleTo(), M_PI))
                        << endl;
                return false;
            }
            else
            {
                return true;
            }
        }

        cout << "CheckGoalKeeper: return false" << endl;
        return false;
    }

    /**
     *
     * @return all obstacles between robot and goal (without goal keeper)
     */

    shared_ptr<vector<geometry::CNPoint2D>> CheckGoalKick::getObstacles()
    {
        // check if obstacle lays in corridor
        auto vNet = wm->pathPlanner.getCurrentVoronoiNet();
        auto ownPosV = wm->rawSensorData.getOwnPositionVision();

        auto obstacles = wm->obstacles.getObstacles();
        shared_ptr < vector < geometry::CNPoint2D >> foundObs = make_shared<vector<geometry::CNPoint2D>>();

        if (obstacles == nullptr || obstacles->size() == 0)
        {
            return NULL;
        }
        cout << "found " << obstacles->size() << " obstacle" << endl;

        for (int i = 0; i < obstacles->size(); i++)
        {
            shared_ptr < geometry::CNPoint2D > egoObstaclePoint = make_shared < geometry::CNPoint2D > (0, 0);
            egoObstaclePoint->x = obstacles->at(i).x;
            egoObstaclePoint->y = obstacles->at(i).y;

            shared_ptr < geometry::CNPoint2D > obstaclePoint = make_shared < geometry::CNPoint2D > (0, 0);
            obstaclePoint = egoObstaclePoint->egoToAllo(*ownPosV);

//			if (wm->pathPlanner.corridorCheckBall(wm->pathPlanner.getCurrentVoronoiNet(), ownPos, goalPosMiddle,
//												obstaclePoint)
            if (wm->pathPlanner.corridorCheckBall(
                    vNet, make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y), goalPosMiddle,
                    make_shared < geometry::CNPoint2D > (obstacles->at(i).x, obstacles->at(i).y)->egoToAllo(*ownPosV))
                    && obstaclePoint->distanceTo(goalPosMiddle) > keeperDistGoal)

            {
                foundObs->push_back(*obstaclePoint);
                break;
            }
        }
        if (foundObs->size() == 0 || foundObs == nullptr)
        {
//			cerr << "foundObs is empty or null" << endl;
            return NULL;
        }
//		cout << "getObstacles: return found obstacles" << endl;
        return foundObs;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
