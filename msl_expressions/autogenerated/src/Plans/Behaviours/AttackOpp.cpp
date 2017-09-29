#include "Plans/Behaviours/AttackOpp.h"

/*PROTECTED REGION ID(inccpp1430324527403) ENABLED START*/ // Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <Ball.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <cmath>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <obstaclehandler/Obstacles.h>
#include <pathplanner/PathPlanner.h>

using geometry::CNPointEgo;
using geometry::CNVecEgo;
using geometry::CNPositionAllo;
using nonstd::optional;
using std::cout;
using std::endl;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1430324527403) ENABLED START*/ // initialise static variables here
    /*PROTECTED REGION END*/
    AttackOpp::AttackOpp() :
            DomainBehaviour("AttackOpp")
    {
        /*PROTECTED REGION ID(con1430324527403) ENABLED START*/ // Add additional options here
        this->oldDistance = 0;
        this->maxIter = 4;
        this->isMovingAwayIter = 0;
        this->isMovingCloserIter = 0;
        this->kD = 1.7;
        this->kI = 0.0;
        this->kP = 2.0;
        this->rotate_P = 1.8;
        /*PROTECTED REGION END*/
    }
    AttackOpp::~AttackOpp()
    {
        /*PROTECTED REGION ID(dcon1430324527403) ENABLED START*/ // Add additional options here
        /*PROTECTED REGION END*/
    }
    void AttackOpp::run(void *msg)
    {
        /*PROTECTED REGION ID(run1430324527403) ENABLED START*/
        msl::RobotMovement rm;

        auto me = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();

        auto egoBallPos = wm->ball->getPositionEgo();

        if (!me || !egoBallPos)
        {
            return;
        }

        auto obstacles = wm->obstacles->getRawObstaclesAlloBuffer().getLastValidContent();
        bool blocked = false;
        msl_actuator_msgs::MotionControl mc;
        for (int i = 0; i < (*obstacles)->size(); i++)
        {
            if (wm->pathPlanner->corridorCheck(me->getPoint(), egoBallPos->toAllo(*me), (*obstacles)->at(i)))
            {
                blocked = true;
                break;
            }
        }
        if (!blocked)
        {
            auto egoBallVelocity = wm->ball->getVelocityEgo();

            if (egoBallVelocity)
            {
                cout << "ego ball vel: " << egoBallVelocity->x << "|" << egoBallVelocity->y << " "
                        << egoBallVelocity->length() << endl;

                auto vector = egoBallVelocity + egoBallPos;
                double vectorLength = vector->length();
                if (wm->ball->haveBall())
                {
                    isMovingAwayIter = 0;
                    isMovingCloserIter = 0;
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
                if (isMovingAwayIter >= maxIter || egoBallVelocity->length() < 250)
                {
                    cout << "roll away" << endl;
                    mc = this->driveToMovingBall(egoBallPos, egoBallVelocity);
                }
                else if (isMovingCloserIter >= maxIter)
                {
                    cout << "get closer" << endl;
                    mc = this->ballGetsCloser(me, egoBallVelocity, egoBallPos);
                }
                else
                {
                    mc.motion.angle = 0;
                    mc.motion.translation = 0;
                    mc.motion.rotation = 0;
                }
            }
        }
        else
        {
            //            mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 0);
            query.egoDestinationPoint = egoBallPos;
            query.egoAlignPoint = egoBallPos;

            mc = rm.moveToPoint(query);
        }
        send(mc);

        // Add additional options here
        /*PROTECTED REGION END*/
    }
    void AttackOpp::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1430324527403) ENABLED START*/ // Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1430324527403) ENABLED START*/ // Add additional methods here
    msl_actuator_msgs::MotionControl AttackOpp::driveToMovingBall(optional<CNPointEgo> egoBallPos, optional<CNVecEgo> egoBallVel)
    {

        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;

        mc.motion.angle = egoBallPos.angleZ();
        mc.motion.rotation = egoBallPos.rotateZ(M_PI).angleZ() * rotate_P;

        double distance = egoBallPos.length();
        double movement = kP * distance + kD * (distance - oldDistance);
        oldDistance = distance;

        double ball_speed = egoBallVel.length();

        movement += ball_speed;
        mc.motion.translation = movement;

        if (egoBallPos.length() < 300)
        {

            bhc.leftMotor = -30;
            bhc.rightMotor = -30;

            this->send(bhc);
            // this->success = true;
        }
        return mc;
    }

    msl_actuator_msgs::MotionControl AttackOpp::ballGetsCloser(CNPositionAllo robotPosition, optional<CNVecEgo> ballVelocity,
                                                               optional<CNPointEgo> egoBallPos)
    {
        double yIntersection = egoBallPos.y + (-(egoBallPos.x / ballVelocity.x)) * ballVelocity.y;

        CNPointEgo interPoint = make_shared<geometry::CNPoint2D>(0, yIntersection);

        msl_actuator_msgs::MotionControl mc;
        msl::RobotMovement rm;
        query.egoDestinationPoint = interPoint;
        query.egoAlignPoint = egoBallPos;

        mc = rm.moveToPoint(query);

        return mc;
    }

/*PROTECTED REGION END*/
} /* namespace alica */
