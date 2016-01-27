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
        //get own Pos
        cout << "Start run CheckGoalKick <=============================================================" << endl;
        auto ownPosition = wm->rawSensorData.getOwnPositionVision();
        ownPos = make_shared < geometry::CNPoint2D > (ownPosition->x, ownPosition->y);
        //get ego ball pos
        egoBallPos = wm->ball.getEgoBallPosition();

        //get goal pos
        goalPosLeft = field->posLeftOppGoalPost();
        goalPosRight = field->posRightOppGoalPost();
        goalPosMiddle = field->posOppGoalMid();
//		cout << "das Glück ist eine Huuuure!!" << endl;

        readConfigParameters();
        toleranceAngle = calcToleranceAngle();
        cout << "toleranceAngle: " << toleranceAngle << " degree: " << toleranceAngle * 180 / M_PI << endl;

        if (checkGoalLine() && checkShootPossibility())
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
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1449076008755) ENABLED START*/ //Add additional methods here
    /*
     * @return true if angle to goal is smaller than tolerance angle
     */
    bool CheckGoalKick::checkGoalLine()
    {
        // for testing!!! <=========================================
        // TODO remove later
//		return true;

//		cout << "angle to goal: " << ownPos->angleToPoint(goalPosMiddle) << " degree: "
//				<< ownPos->angleToPoint(goalPosMiddle) * 180 / M_PI << endl;
//
        cout << "goalPosMiddle: " << goalPosMiddle->toString();
        egoAlignPoint = goalPosMiddle;

        auto ownPos = wm->rawSensorData.getOwnPositionVision();
        auto egoTarget = goalPosMiddle->alloToEgo(*ownPos);

        cout << "egoTarget: " << egoTarget->toString();
        cout << "angle to goal: " << egoTarget->angleTo() << " degree: " << egoTarget->angleTo() * 180 / M_PI << endl;

        cout << "if condition1: " << (egoTarget->angleTo() > (M_PI - toleranceAngle)) << endl;
        cout << "if condition2: " << (egoTarget->angleTo() > (-M_PI + toleranceAngle)) << endl;

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
        cout << "checkShootPossibility() ============================================" << endl;
        // check if obstacle lays in corridor
        auto obstacles = wm->robots.getObstacles();
        shared_ptr < geometry::CNPoint2D > obstaclePoint = make_shared < geometry::CNPoint2D > (0, 0);
        bool foundObstacle = false;
        int obstacleAt = -1;

        cout << "obstacle size = " << obstacles->size() << endl;
        for (int i = 0; i < obstacles->size(); i++)
        {
            obstaclePoint->x = obstacles->at(i).x;
            obstaclePoint->y = obstacles->at(i).y;

            if (wm->pathPlanner.corridorCheck(wm->pathPlanner.getCurrentVoronoiNet(), ownPos, goalPosMiddle,
                                              obstaclePoint))
            {
                foundObstacle = true;
                obstacleAt = i;
                break;
            }
        }

        cout << "found Obstacle = " << foundObstacle << endl;

        if (obstacleAt > -1)
        {
            // check if obstacle is blocking (Own distance -> obstacle and obstacle -> OppGoal)
            auto obstacle = obstacles->at(obstacleAt);
            shared_ptr < geometry::CNPoint2D > obstaclePos = make_shared < geometry::CNPoint2D > (0, 0);
            obstaclePos->x = obstacle.x;
            obstaclePos->y = obstacle.y;

            cout << "obstaclePosX = " << obstaclePos->x << endl;
            cout << "obstaclePosY = " << obstaclePos->y << endl;

            cout << "ownPos <----> obstaclePos = " << ownPos->distanceTo(obstaclePos) << endl;
            cout << "if condition 1 = " << (ownPos->distanceTo(obstaclePos) > robotShootDistanceOwn) << endl;
            cout << "obstaclePos <----> GoalPosMiddle = " << obstaclePos->distanceTo(goalPosMiddle) << endl;
            cout << "if condition 2 = " << (obstaclePos->distanceTo(goalPosMiddle) > robotShootDistanceGoal) << endl;

            if (ownPos->distanceTo(obstaclePos) > robotShootDistanceOwn
                    || obstaclePos->distanceTo(goalPosMiddle) > robotShootDistanceGoal)
            {
            	cout << "return true" << endl;
                return false;
            }
            else
            {
            	cout << "return true" << endl;
                return true;
            }
        }
        else
        {
            return true;
        }
    }

    void CheckGoalKick::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        robotShootDistanceOwn = (*sc)["GoalKick"]->get<double>("GoalKick.Default.robotShootDistanceOwn", NULL);
        robotShootDistanceGoal = (*sc)["GoalKick"]->get<double>("GoalKick.Default.robotShootDistanceGoal", NULL);
        minKickPower = (*sc)["GoalKick"]->get<double>("GoalKick.Default.minKickPower", NULL);
        wheelSpeedLeft = (*sc)["GoalKick"]->get<double>("GoalKick.Default.wheelSpeedLeft", NULL);
        wheelSpeedRight = (*sc)["GoalKick"]->get<double>("GoalKick.Default.wheelSpeedRight", NULL);
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
        cout << "a = " << a << "\nb= " << b << "\nc= " << c << endl;

        return acos((pow(b, 2) - pow(c, 2) - pow(a, 2)) / (-2 * a * c));
    }

    void CheckGoalKick::kicking()
    {
//		double minKickPower = 1500.0;

        msl_actuator_msgs::BallHandleCmd bhc;
        auto egoBallPos = wm->ball.getEgoBallPosition();
        double distance = egoBallPos->length();
        cout << "distance: " << distance << endl;
        if (distance < 600)
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
        cout << "send BallHandleCmd" << endl;
        cout << "haveBall() = " << wm->ball.haveBall() << endl;

        cout << "shovel selection to: 1" << endl;
        msl_actuator_msgs::KickControl kc;
        kc.extension = 1;

        if (wm->ball.haveBall())
        {
            if (waitingIter > 30)
            {
                kc.enabled = true;
//			kc.kicker = egoBallPos->angleTo();
//			kc.power = min(minKickPower, egoAimPoint->length());
                kc.power = minKickPower;
                send(kc);
                waitingIter = 0;
            }
            else
            {
                waitingIter++;
            }
        }
    }

    bool CheckGoalKick::checkGoalKeeper()
    {
        // copied and adapt from GoalKick.cpp

//		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision();
//		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball.getEgoBallPosition();
        auto vNet = wm->pathPlanner.getCurrentVoronoiNet();
        auto ownPosV = wm->rawSensorData.getOwnPositionVision();

        if (ownPos == nullptr || egoBallPos == nullptr || vNet == nullptr)
        {
            return false;
        }

//		msl_actuator_msgs::BallHandleCmd bhc;
//		bhc.leftMotor = (int8_t)-70;
//		bhc.rightMotor = (int8_t)-70;
//		send(bhc);

        auto alloAimPoint = nullptr;

        auto obs = wm->robots.getObstacles();
        bool leftBlocked = false;
        bool midBlocked = false;
        bool rightBlocked = false;
        for (int i = 0; i < obs->size(); i++)
        {
            if (leftBlocked && midBlocked && rightBlocked)
            {
                break;
            }
            if (wm->pathPlanner.corridorCheckBall(
                    vNet, make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y), alloLeftAimPoint,
                    make_shared < geometry::CNPoint2D > (obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPosV)))
            {
                leftBlocked = true;
            }
            if (wm->pathPlanner.corridorCheckBall(
                    vNet, make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y), alloMidAimPoint,
                    make_shared < geometry::CNPoint2D > (obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPosV)))
            {
                midBlocked = true;
            }
            if (wm->pathPlanner.corridorCheckBall(
                    vNet, make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y), alloRightAimPoint,
                    make_shared < geometry::CNPoint2D > (obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPosV)))
            {
                rightBlocked = true;
            }

        }
//		if (!leftBlocked && alloAimPoint == nullptr)
//		{
//			cout << "aimig left" << endl;
//			alloAimPoint = alloLeftAimPoint;
//		}
//		if (!midBlocked && alloAimPoint == nullptr)
//		{
//			cout << "aimig mid" << endl;
//			alloAimPoint = alloMidAimPoint;
//		}
//		if (!rightBlocked && alloAimPoint == nullptr)
//		{
//			cout << "aimig right" << endl;
//			alloAimPoint = alloRightAimPoint;
//		}
//		if (leftBlocked && midBlocked && rightBlocked && alloAimPoint == nullptr)
//		{
//			this->failure = true;
//		}
        return false;
    }

/*PROTECTED REGION END*/
} /* namespace alica */
