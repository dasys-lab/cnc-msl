using namespace std;
#include "Plans/Behaviours/DriveForward.h"

/*PROTECTED REGION ID(inccpp1417017564406) ENABLED START*/ //Add additional includes here
#include "math.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1417017564406) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DriveForward::DriveForward() :
            DomainBehaviour("DriveForward")
    {
        /*PROTECTED REGION ID(con1417017564406) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    DriveForward::~DriveForward()
    {
        /*PROTECTED REGION ID(dcon1417017564406) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DriveForward::run(void* msg)
    {
        /*PROTECTED REGION ID(run1417017564406) ENABLED START*/ //Add additional options here
        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;
        //MOTION:
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();
        mc.motion.angle = 0;
        mc.motion.rotation = 0;
        mc.motion.translation = 1000.0;

        this->send(mc);

        if (egoBallPos->length() < 150)
        {

            bhc.leftMotor = -30;
            bhc.rightMotor = -30;

            this->send(bhc);

            //    return mc;
        }
        /*PROTECTED REGION END*/
    }
    void DriveForward::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1417017564406) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1417017564406) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
