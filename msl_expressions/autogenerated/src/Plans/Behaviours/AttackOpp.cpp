using namespace std;
#include "Plans/Behaviours/AttackOpp.h"

/*PROTECTED REGION ID(inccpp1430324527403) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <cmath>
#include <RawSensorData.h>
#include <Ball.h>
#include <obstaclehandler/Obstacles.h>
#include <pathplanner/PathPlanner.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <MSLWorldModel.h>
#include <Logger.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1430324527403) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AttackOpp::AttackOpp() :
            DomainBehaviour("AttackOpp")
    {
        /*PROTECTED REGION ID(con1430324527403) ENABLED START*/ //Add additional options here
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    AttackOpp::~AttackOpp()
    {
        /*PROTECTED REGION ID(dcon1430324527403) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AttackOpp::run(void* msg)
    {
        /*PROTECTED REGION ID(run1430324527403) ENABLED START*/
        shared_ptr < geometry::CNPosition > me = wm->rawSensorData->getOwnPositionVision();

        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();

        if (me == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        auto obstacles = wm->obstacles->getEgoVisionObstacles();
        bool blocked = false;
        msl_actuator_msgs::MotionControl mc;
        for (int i = 0; i < obstacles->size(); i++)
        {
            if (wm->pathPlanner->corridorCheck(
                    make_shared < geometry::CNPoint2D > (me->x, me->y), egoBallPos->egoToAllo(*me),
                    make_shared < geometry::CNPoint2D > (obstacles->at(i).x, obstacles->at(i).y)))
            {
                blocked = true;
                break;
            }
        }
        if (!blocked)
        {
            auto egoBallVelocity = wm->ball->getEgoBallVelocity();
            //cout << "ego ball vel: " << egoBallVelocity->x << "|" << egoBallVelocity->y << " "
            //        << egoBallVelocity->length() << endl;
            this->logger->log(this->getName(),"ego ball vel: " + std::to_string(egoBallVelocity->x) + "|" + std::to_string(egoBallVelocity->y) + " " + std::to_string(egoBallVelocity->length()),msl::LogLevels::debug);
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
                //cout << "roll away" << endl;
            	this->logger->log(this->getName(),"roll away",msl::LogLevels::debug);
                mc = driveToMovingBall(egoBallPos, egoBallVelocity);
            }
            else if (isMovingCloserIter >= maxIter)
            {
                //cout << "get closer" << endl;
            	this->logger->log(this->getName(), "get closer", msl::LogLevels::debug);
                mc = ballGetsCloser(me, egoBallVelocity, egoBallPos);

            }
            else
            {
                mc.motion.angle = 0;
                mc.motion.translation = 0;
                mc.motion.rotation = 0;

            }
        }
        else
        {
//            mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 0);
            query->egoDestinationPoint = egoBallPos;
            query->egoAlignPoint = egoBallPos;

            mc = this->robot->robotMovement->moveToPoint(query);
        }
        send(mc);

//Add additional options here
        /*PROTECTED REGION END*/
    }
    void AttackOpp::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1430324527403) ENABLED START*/ //Add additional options here
        oldDistance = 0.0;
        kP = 2.0;
        kI = 0.0;
        kD = 1.7;
        rotate_P = 1.8;
        isMovingCloserIter = 0;
        isMovingAwayIter = 0;
        maxIter = 4;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1430324527403) ENABLED START*/ //Add additional methods here
    msl_actuator_msgs::MotionControl AttackOpp::driveToMovingBall(shared_ptr<geometry::CNPoint2D> egoBallPos,
                                                                  shared_ptr<geometry::CNVelocity2D> egoBallVel)
    {

        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;

        mc.motion.angle = egoBallPos->angleTo();
        mc.motion.rotation = egoBallPos->rotate(M_PI)->angleTo() * rotate_P;

        double distance = egoBallPos->length();
        double movement = kP * distance + kD * (distance - oldDistance);
        oldDistance = distance;

        double ball_speed = egoBallVel->length();

        movement += ball_speed;
        mc.motion.translation = movement;

        if (egoBallPos->length() < 300)
        {

            bhc.leftMotor = -30;
            bhc.rightMotor = -30;

            this->send(bhc);
            //this->success = true;
        }
        return mc;
    }

    msl_actuator_msgs::MotionControl AttackOpp::ballGetsCloser(shared_ptr<geometry::CNPosition> robotPosition,
                                                               shared_ptr<geometry::CNVelocity2D> ballVelocity,
                                                               shared_ptr<geometry::CNPoint2D> egoBallPos)
    {
        double yIntersection = egoBallPos->y + (-(egoBallPos->x / ballVelocity->x)) * ballVelocity->y;

        shared_ptr < geometry::CNPoint2D > interPoint = make_shared < geometry::CNPoint2D > (0, yIntersection);

        msl_actuator_msgs::MotionControl mc;
        // TODO : remove later
//        mc = RobotMovement::moveToPointCarefully(interPoint, egoBallPos, 300);
        query->egoDestinationPoint = interPoint;
        query->egoAlignPoint = egoBallPos;

        mc = this->robot->robotMovement->moveToPoint(query);

        return mc;
    }

/*PROTECTED REGION END*/
} /* namespace alica */
