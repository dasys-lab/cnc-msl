using namespace std;
#include "Plans/Behaviours/AttackOpp.h"

/*PROTECTED REGION ID(inccpp1430324527403) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <cmath>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1430324527403) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AttackOpp::AttackOpp() :
            DomainBehaviour("AttackOpp")
    {
        /*PROTECTED REGION ID(con1430324527403) ENABLED START*/ //Add additional options here
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

        shared_ptr < geometry::CNPosition > me = wm->rawSensorData.getOwnPositionVision();

        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();

        if (me == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        //auto obstacles = wm->robots.getObstacles();

        //for (auto obstacle : *obstacles)
        //{
        // TODO: Get closest obstacle to ball
        //}

        msl_actuator_msgs::MotionControl mc;
//        // TODO : remove later
//        mc = RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 300);
//        mc.motion.translation = 0;
        auto egoBallVelocity = wm->ball.getEgoBallVelocity();
        auto vector = egoBallVelocity + egoBallPos;
        double vectorLength = vector->length();
        if (vectorLength < egoBallPos->length())
        {
        	isMovingCloserIter++;
        	isMovingAwayIter = 0;
        }
        else
        {
        	isMovingAwayIter++;
        	isMovingCloserIter = 0;
        }
        if(isMovingCloserIter >= maxIter)
        {
        	cout << "get closer" << endl;
            mc = ballGetsCloser(me, egoBallVelocity, egoBallPos);

        }
        else if(isMovingAwayIter >= maxIter)
        {
        	cout << "roll away" << endl;
        	mc = driveToMovingBall(egoBallPos, egoBallVelocity);
        }
        else
        {
        	mc.motion.angle = 0;
        	mc.motion.translation = 0;
        	mc.motion.rotation = 0;

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
        maxIter = 3;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1430324527403) ENABLED START*/ //Add additional methods here
    msl_actuator_msgs::MotionControl AttackOpp::driveToMovingBall(shared_ptr<geometry::CNPoint2D> egoBallPos, shared_ptr<geometry::CNVelocity2D> egoBallVel)
    {

        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;

        mc.motion.angle = egoBallPos->angleTo();
        mc.motion.rotation = egoBallPos->rotate(M_PI)->angleTo() * rotate_P;

        double summe = 0.0;
        //distance ball to robot
        double distance = egoBallPos->length();
        //TODO bullshit summe ist an der stelle IMMER 0.0
        summe = summe + distance;
        double movement = kP * distance + kI * summe + kD * (distance - oldDistance);
        oldDistance = distance;

        double ball_speed = egoBallVel->length();

        movement += ball_speed;

        //cout << "movement: " << movement << endl;
        //cout << "ball speed: " << ball_speed << endl;
        //cout << "distance: " << distance << endl;

        // translation = 1000 => 1 m/s
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

    msl_actuator_msgs::MotionControl AttackOpp::ballGetsCloser(shared_ptr < geometry::CNPosition > robotPosition,
															   shared_ptr<geometry::CNVelocity2D> ballVelocity,
                                   shared_ptr<geometry::CNPoint2D> egoBallPos)
    {
        double yIntersection =  egoBallPos->y + (-(egoBallPos->x / ballVelocity->x)) * ballVelocity->y;

        shared_ptr < geometry::CNPoint2D > interPoint = make_shared < geometry::CNPoint2D > (0, yIntersection);

        msl_actuator_msgs::MotionControl mc;
        // TODO : remove later
        mc = RobotMovement::moveToPointCarefully(interPoint, egoBallPos, 300);

        cout << "xVelocity:" << ballVelocity->x << endl;
        cout << "yVelocity:" << ballVelocity->y << endl;
        cout << "Y-Intersection: " << yIntersection << endl;

        cout << endl;

        return mc;
    }

/*PROTECTED REGION END*/
} /* namespace alica */
