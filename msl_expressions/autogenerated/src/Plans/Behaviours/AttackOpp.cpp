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
        old_x = 0;
        old_y = 0;
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

        //auto obstacles = wm->robots.getObstacles();

        //for (auto obstacle : *obstacles)
        //{
        // TODO: Get closest obstacle to ball
        //}

        auto x = egoBallPos->x;
        auto y = egoBallPos->y;

        if (old_x == 0 && old_y == 0)
        {
            old_x = x;
            old_y = y;
            return;
        }

        msl_actuator_msgs::MotionControl mc;
        // TODO : remove later
        mc = RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 300);
        mc.motion.translation = 0;
        cout << "x: " << x << endl;
        cout << "y: " << y << endl;
        auto egoBallVelocity = wm->ball.getEgoBallVelocity();
        auto vector = egoBallVelocity + egoBallPos;
        double vectorLength = vector->length();

        if (vectorLength < egoBallPos->length())
        {
            cout << "get closer" << endl;

            mc = ballGetsCloser(me, egoBallVelocity, egoBallPos);

        }
        else
        {
            cout << "roll away" << endl;
        }

        old_x = x;
        old_y = y;

        //cout << "egoBallPos x: " << x << " y: " << y << endl;

        if (me == nullptr || egoBallPos == nullptr)
        {
            cerr << "insufficient information for AttackOpp" << endl;
            return;
        }

        if (!me.operator bool())
        {
            return;
        }

        //mc.motion.translation = 0;
        send(mc);

//Add additional options here
        /*PROTECTED REGION END*/
    }
    void AttackOpp::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1430324527403) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1430324527403) ENABLED START*/ //Add additional methods here
    msl_actuator_msgs::MotionControl AttackOpp::driveToMovingBall(shared_ptr<geometry::CNPoint2D> egoBallPos)
    {

        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;
        mc = RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 300);

        const double rotate_P = 1.8;

        mc.motion.angle = egoBallPos->angleTo();
        mc.motion.rotation = egoBallPos->rotate(M_PI)->angleTo() * rotate_P;

        double summe = 0.0;
        static double olddistance = 0.0;

        const double Kp = 2.0;
        const double Ki = 0.0;
        const double Kd = 1.7;

        //distance ball to robot
        double distance = egoBallPos->length();

        summe = summe + distance;
        double movement = Kp * distance + Ki * summe + Kd * (distance - olddistance);
        olddistance = distance;

        auto egoBallVelocity = wm->ball.getEgoBallVelocity();

        double ball_speed = egoBallVelocity->length();

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
    }

    msl_actuator_msgs::MotionControl AttackOpp::ballGetsCloser(shared_ptr < geometry::CNPosition > robotPosition,
															   shared_ptr<geometry::CNVelocity2D> ballVelocity,
                                   shared_ptr<geometry::CNPoint2D> egoBallPos)
    {
        const double xVelocity = ballVelocity->x;
        const double yVelocity = ballVelocity->y;
        const double xDistance = abs(egoBallPos->x);
        const double yDistance = abs(egoBallPos->y);

        const double yIntersection =  yDistance - (xDistance / xVelocity) * yVelocity;

        shared_ptr < geometry::CNPoint2D > interPoint = make_shared < geometry::CNPoint2D > (0, yIntersection);

        msl_actuator_msgs::MotionControl mc;
        // TODO : remove later
        mc = RobotMovement::moveToPointCarefully(interPoint, egoBallPos, 300);

        cout << "xVelocity:" << xVelocity << endl;
        cout << "yVelocity:" << yVelocity << endl;
        cout << "Y-Intersection: " << yIntersection << endl;

        cout << endl;

        return mc;
    }

/*PROTECTED REGION END*/
} /* namespace alica */
