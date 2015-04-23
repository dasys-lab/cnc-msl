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
	//MOTION:
        mc.motion.angle = M_PI;
        mc.motion.rotation = 0;
        mc.motion.translation = 500;

        this->send(mc);

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
