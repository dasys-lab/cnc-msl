using namespace std;
#include "Plans/Behaviours/GetBall.h"

/*PROTECTED REGION ID(inccpp1414828300860) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <obstaclehandler/Obstacles.h>
#include <pathplanner/PathPlanner.h>
#include <Game.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1414828300860) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    GetBall::GetBall() :
            DomainBehaviour("GetBall")
    {
        /*PROTECTED REGION ID(con1414828300860) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    GetBall::~GetBall()
    {
        /*PROTECTED REGION ID(dcon1414828300860) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void GetBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1414828300860) ENABLED START*/ //Add additional options here
    	msl::RobotMovement rm;

        shared_ptr < geometry::CNPosition > me = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();
        if (me == nullptr || egoBallPos == nullptr)
        {
            return;
        }
        auto obstacles = wm->obstacles->getAlloObstaclePoints();
        bool blocked = false;
        msl_actuator_msgs::MotionControl mc;
        if (obstacles != nullptr)
        {
            for (int i = 0; i < obstacles->size(); i++)
            {
                if (wm->pathPlanner->corridorCheck(make_shared < geometry::CNPoint2D > (me->x, me->y),
                                                   egoBallPos->egoToAllo(*me), obstacles->at(i)))
                {
                    blocked = true;
                    break;
                }
            }
        }
        if (!blocked)
        {
            auto egoBallVelocity = wm->ball->getEgoBallVelocity();
            if (egoBallVelocity == nullptr)
            {
                egoBallVelocity = make_shared<geometry::CNVelocity2D>();
            }
            auto vector = egoBallVelocity + egoBallPos;
            double vectorLength = vector->length();
            if (wm->ball->haveBall())
            {
                isMovingAwayIter = 0;
                isMovingCloserIter = 0;
                this->setSuccess(true);
                mc = driveToMovingBall(egoBallPos, egoBallVelocity);
                mc.motion.translation = 500;
                send(mc);
                return;
            }
            else if (wm->game->getTimeSinceStart() >= timeForPass)
            {
                this->setFailure(true);
            }
            else if (vectorLength < egoBallPos->length() && egoBallVelocity->length() > 250)
            {
                isMovingCloserIter++;
                isMovingAwayIter = 0;
            }
            else if (vectorLength > egoBallPos->length() && egoBallVelocity->length() > 250)
            {
                isMovingAwayIter++;
                isMovingCloserIter = 0;
            }

            if (isMovingAwayIter >= maxIter || egoBallVelocity->length() <= 250)
            {
                mc = driveToMovingBall(egoBallPos, egoBallVelocity);
            }
            else if (isMovingCloserIter >= maxIter)
            {
                mc = driveToApproachingBall(egoBallVelocity, egoBallPos);
            }
            else
            {
                mc = driveToMovingBall(egoBallPos, egoBallVelocity);
            }
        }
        else
        {
            mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 0);
        }
        // replaced with new method
//        mc = msl::RobotMovement::nearGoalArea(mc);
        mc = rm.ruleActionForBallGetter();
//        cout <<"GetBall: " << mc.motion.angle << " " << mc.motion.translation << " " << endl;
        send(mc);
        /*PROTECTED REGION END*/
    }
    void GetBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1414828300860) ENABLED START*/ //Add additional options here
        oldDistance = 0.0;
        kP = 2.0;
        kD = 1.7;
        rotate_P = 1.8;
        isMovingCloserIter = 0;
        isMovingAwayIter = 0;
        maxIter = 4;
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        timeForPass = (*sc)["Rules"]->get<double>("Rules.Standards.PenaltyTimeForShot", NULL) * 1000000;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1414828300860) ENABLED START*/ //Add additional methods here
    msl_actuator_msgs::MotionControl GetBall::driveToMovingBall(shared_ptr<geometry::CNPoint2D> egoBallPos,
                                                                shared_ptr<geometry::CNVelocity2D> egoBallVel)
    {

        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;

        double distance = egoBallPos->length();
        double movement = kP * distance + kD * (distance - oldDistance);
        oldDistance = distance;
        double ballSpeed = egoBallVel->length();
        movement += ballSpeed;

        mc.motion.translation = movement;
        mc.motion.angle = egoBallPos->angleTo();
        mc.motion.rotation = egoBallPos->rotate(M_PI)->angleTo() * rotate_P;
        return mc;
    }

    msl_actuator_msgs::MotionControl GetBall::driveToApproachingBall(shared_ptr<geometry::CNVelocity2D> ballVelocity,
                                                                     shared_ptr<geometry::CNPoint2D> egoBallPos)
    {
        double yIntersection = egoBallPos->y + (-(egoBallPos->x / ballVelocity->x)) * ballVelocity->y;

        shared_ptr < geometry::CNPoint2D > interPoint = make_shared < geometry::CNPoint2D > (0, yIntersection);

        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;
        mc = RobotMovement::moveToPointCarefully(interPoint, egoBallPos, 100);
        return mc;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
