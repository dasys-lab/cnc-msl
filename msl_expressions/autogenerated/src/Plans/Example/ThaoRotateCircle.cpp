using namespace std;
#include "Plans/Example/ThaoRotateCircle.h"

/*PROTECTED REGION ID(inccpp1450104610893) ENABLED START*/ //Add additional includes here
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
        msl_actuator_msgs::MotionControl mcon;
        mcon.motion.angle = 0;
        mcon.motion.rotation = 0;
        mcon.motion.translation = 1000;
        send(mcon);

        /*PROTECTED REGION END*/
    }
    void ThaoRotateCircle::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450104610893) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1450104610893) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
