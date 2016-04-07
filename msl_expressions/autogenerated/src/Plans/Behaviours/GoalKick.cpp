using namespace std;
#include "Plans/Behaviours/GoalKick.h"

/*PROTECTED REGION ID(inccpp1415205565589) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1415205565589) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    GoalKick::GoalKick() :
            DomainBehaviour("GoalKick")
    {
        /*PROTECTED REGION ID(con1415205565589) ENABLED START*/ //Add additional options here
        alloLeftAimPoint = nullptr;
        alloMidAimPoint = nullptr;
        alloRightAimPoint = nullptr;
        alloAimPoint = nullptr;
        angleTolerance = 0.05;
        minKickPower = 1500.0;
        /*PROTECTED REGION END*/
    }
    GoalKick::~GoalKick()
    {
        /*PROTECTED REGION ID(dcon1415205565589) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void GoalKick::run(void* msg)
    {
        /*PROTECTED REGION ID(run1415205565589) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        msl_actuator_msgs::BallHandleCmd bhc;
        bhc.leftMotor = (int8_t) - 70;
        bhc.rightMotor = (int8_t) - 70;
        send(bhc);

        alloAimPoint = nullptr;

        auto obs = wm->obstacles.getEgoVisionObstacles();
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
                    make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y), alloLeftAimPoint,
                    make_shared < geometry::CNPoint2D > (obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPos)))
            {
                leftBlocked = true;
            }
            if (wm->pathPlanner.corridorCheckBall(
                    make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y), alloMidAimPoint,
                    make_shared < geometry::CNPoint2D > (obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPos)))
            {
                midBlocked = true;
            }
            if (wm->pathPlanner.corridorCheckBall(
                    make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y), alloRightAimPoint,
                    make_shared < geometry::CNPoint2D > (obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPos)))
            {
                rightBlocked = true;
            }

        }
        if (!leftBlocked && alloAimPoint == nullptr)
        {
            cout << "aimig left" << endl;
            alloAimPoint = alloLeftAimPoint;
        }
        if (!midBlocked && alloAimPoint == nullptr)
        {
            cout << "aimig mid" << endl;
            alloAimPoint = alloMidAimPoint;
        }
        if (!rightBlocked && alloAimPoint == nullptr)
        {
            cout << "aimig right" << endl;
            alloAimPoint = alloRightAimPoint;
        }
        if (leftBlocked && midBlocked && rightBlocked && alloAimPoint == nullptr)
        {
            this->failure = true;
        }
        if (alloAimPoint != nullptr)
        {
            auto egoAimPoint = alloAimPoint->alloToEgo(*ownPos);
            msl_actuator_msgs::MotionControl mc = msl::RobotMovement::rapidAlignToPointWithBall(
                    egoAimPoint->rotate(M_PI), egoBallPos, this->angleTolerance, this->angleTolerance);

            if (fabs(geometry::deltaAngle(egoAimPoint->angleTo(), M_PI)) > this->angleTolerance)
            {
                cout << "angle: " << fabs(geometry::deltaAngle(egoAimPoint->angleTo(), M_PI)) << endl;
                send(mc);
            }
            else
            {
                cout << "kicking" << endl;
                msl_actuator_msgs::KickControl kc;
                kc.enabled = true;
                kc.kicker = egoBallPos->angleTo();
                kc.power = min(minKickPower, egoAimPoint->length());
                send(kc);
                this->success = true;
            }
        }
        /*PROTECTED REGION END*/
    }
    void GoalKick::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1415205565589) ENABLED START*/ //Add additional options here
        alloLeftAimPoint = make_shared < geometry::CNPoint2D
                > (wm->field.getFieldLength() / 2 + 250, wm->field.posLeftOppGoalPost()->y - wm->ball.getBallDiameter() * 1.5);
        alloMidAimPoint = make_shared < geometry::CNPoint2D > (wm->field.getFieldLength() / 2 + 250, 0);
        alloRightAimPoint = make_shared < geometry::CNPoint2D
                > (wm->field.getFieldLength() / 2 + 250, wm->field.posRightOppGoalPost()->y + wm->ball.getBallDiameter() * 1.5);
        alloAimPoint = nullptr;
        angleTolerance = 0.075;
        minKickPower = 1500.0;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1415205565589) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
