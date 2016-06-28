using namespace std;
#include "Plans/GenericBehaviours/Stop.h"

/*PROTECTED REGION ID(inccpp1413992604875) ENABLED START*/ //Add additional includes here
#include <msl_actuator_msgs/MotionControl.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1413992604875) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Stop::Stop() :
            DomainBehaviour("Stop")
    {
        /*PROTECTED REGION ID(con1413992604875) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    Stop::~Stop()
    {
        /*PROTECTED REGION ID(dcon1413992604875) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Stop::run(void* msg)
    {
        /*PROTECTED REGION ID(run1413992604875) ENABLED START*/ //Add additional options here
        msl_actuator_msgs::MotionControl mc;
        mc.motion.angle = 0;
        mc.motion.rotation = 0;
        mc.motion.translation = 0;
//	cout << " Stop HaveBall" << wm->ball.haveBall() << endl;
        send(mc);

        /*PROTECTED REGION END*/
    }
    void Stop::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1413992604875) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1413992604875) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
