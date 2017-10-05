using namespace std;
#include "Plans/Behaviours/GetBall.h"

/*PROTECTED REGION ID(inccpp1414828300860) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <obstaclehandler/Obstacles.h>
#include <pathplanner/PathPlanner.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <MSLWorldModel.h>
#include <Game.h>
using geometry::CNVecEgo;
using geometry::CNPointEgo;
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
        msl_actuator_msgs::MotionControl mc;

        auto me = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto egoBallPos = wm->ball->getPositionEgo();
        if (!me || !egoBallPos)
        {
            return;
        }
        auto obstacles = wm->obstacles->getClusteredObstaclesAlloBuffer().getLastValidContent();
        bool blocked = false;

        if (obstacles)
        {
            for (int i = 0; i < (*obstacles)->size(); i++)
            {
                if (wm->pathPlanner->corridorCheck(geometry::CNPointAllo (me->x, me->y),
                                                   egoBallPos->toAllo(*me), (*obstacles)->at(i).position.getPoint()))
                {
                    blocked = true;
                    break;
                }
            }
        }
        if (!blocked)
        {
            auto egoBallVelocity = wm->ball->getVisionBallVelocityBuffer().getLastValidContent();
            if (!egoBallVelocity)
            {
                *egoBallVelocity = CNVecEgo();
            }
            auto vector = egoBallVelocity + egoBallPos;
            double vectorLength = vector->length();
            if (wm->ball->haveBall())
            {
                isMovingAwayIter = 0;
                isMovingCloserIter = 0;
                this->setSuccess(true);
                mc = driveToMovingBall(*egoBallPos, *egoBallVelocity);
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
                mc = driveToMovingBall(*egoBallPos, *egoBallVelocity);
            }
            else if (isMovingCloserIter >= maxIter)
            {
                mc = driveToApproachingBall(*egoBallVelocity, *egoBallPos);
            }
            else
            {
                mc = driveToMovingBall(*egoBallPos, *egoBallVelocity);
            }
        }
        else
        {
//            mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 0);
            query.egoDestinationPoint = egoBallPos;
            query.egoAlignPoint = egoBallPos;
            mc = rm.moveToPoint(query);
        }
        // replaced with new method
        auto tmpMC = rm.ruleActionForBallGetter();
        if (!std::isnan(tmpMC.motion.translation))
        {
            send(tmpMC);
            send(mc);
        }
        else
        {
            cout << "Motion command is NaN!" << endl;
        }
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
    msl_actuator_msgs::MotionControl GetBall::driveToMovingBall(CNPointEgo egoBallPos,
                                                                CNVecEgo egoBallVel)
    {

        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;

        double distance = egoBallPos.length();
        double movement = kP * distance + kD * (distance - oldDistance);
        oldDistance = distance;
        double ballSpeed = egoBallVel.length();
        movement += ballSpeed;

        mc.motion.translation = movement;
        mc.motion.angle = egoBallPos.angleZ();
        mc.motion.rotation = egoBallPos.rotateZ(M_PI).angleZ() * rotate_P;
        return mc;
    }

    msl_actuator_msgs::MotionControl GetBall::driveToApproachingBall(CNVecEgo ballVelocity,
                                                                    CNPointEgo egoBallPos)
    {
        double yIntersection = egoBallPos.y + (-(egoBallPos.x / ballVelocity.x)) * ballVelocity.y;

        shared_ptr < geometry::CNPoint2D > interPoint = make_shared < geometry::CNPoint2D > (0, yIntersection);

        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;
//        mc = RobotMovement::moveToPointCarefully(interPoint, egoBallPos, 100);
        msl::RobotMovement rm;
        query.egoDestinationPoint = interPoint;
        query.egoAlignPoint = egoBallPos;
        query.snapDistance = 100;
        mc = rm.moveToPoint(query);
        return mc;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
