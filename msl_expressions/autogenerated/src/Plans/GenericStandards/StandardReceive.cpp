using namespace std;
#include "Plans/GenericStandards/StandardReceive.h"

/*PROTECTED REGION ID(inccpp1428509505186) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1428509505186) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StandardReceive::StandardReceive() :
            DomainBehaviour("StandardReceive")
    {
        /*PROTECTED REGION ID(con1428509505186) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    StandardReceive::~StandardReceive()
    {
        /*PROTECTED REGION ID(dcon1428509505186) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void StandardReceive::run(void* msg)
    {
        /*PROTECTED REGION ID(run1428509505186) ENABLED START*/ //Add additional options here
        //TODO hack to stop robot in simulator
        msl_actuator_msgs::MotionControl mc;
        mc.motion.angle = 0;
        mc.motion.translation = 0;
        mc.motion.rotation = 0;
        send(mc);
        /*PROTECTED REGION END*/
    }
    void StandardReceive::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1428509505186) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1428509505186) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
