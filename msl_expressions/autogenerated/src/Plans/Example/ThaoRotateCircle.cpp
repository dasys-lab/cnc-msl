using namespace std;
#include "Plans/Example/ThaoRotateCircle.h"

/*PROTECTED REGION ID(inccpp1450104610893) ENABLED START*/ //Add additional includes here
using namespace std;
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1450104610893) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    ThaoRotateCircle::ThaoRotateCircle() :
            DomainBehaviour("ThaoRotateCircle")
    {
        /*PROTECTED REGION ID(con1450104610893) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    ThaoRotateCircle::~ThaoRotateCircle()
    {
        /*PROTECTED REGION ID(dcon1450104610893) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void ThaoRotateCircle::run(void* msg)
    {
        /*PROTECTED REGION ID(run1450104610893) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void ThaoRotateCircle::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450104610893) ENABLED START*/ //Add additional options here
        count = 0;
        ownPos = wm->rawSensorData.getOwnPositionVision();
        this->haveBeenFarAway = false;
        cout << "ownPos=" << (*ownPos).toString();
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
    /*PROTECTED REGION ID(methods1450104610893) ENABLED START*/ //Add additional methods here
    msl_actuator_msgs::MotionControl ThaoRotateCircle::driveToMovingBall(shared_ptr<geometry::CNPoint2D> egoBallPos,
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

        if (egoBallPos->length() < 1500)
        {

            bhc.leftMotor = -30;
            bhc.rightMotor = -30;

            this->send(bhc);
        }
        return mc;
    }

    msl_actuator_msgs::MotionControl ThaoRotateCircle::driveToApproachingBall(
            shared_ptr<geometry::CNVelocity2D> ballVelocity, shared_ptr<geometry::CNPoint2D> egoBallPos)
    {
        double yIntersection = egoBallPos->y + (-(egoBallPos->x / ballVelocity->x)) * ballVelocity->y;

        shared_ptr < geometry::CNPoint2D > interPoint = make_shared < geometry::CNPoint2D > (0, yIntersection);

        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;
        mc = RobotMovement::moveToPointCarefully(interPoint, egoBallPos, 100);

        if (egoBallPos->length() < 500)
        {

            bhc.leftMotor = -30;
            bhc.rightMotor = -30;

            this->send(bhc);
        }

        return mc;

    }

/*PROTECTED REGION END*/
} /* namespace alica */
