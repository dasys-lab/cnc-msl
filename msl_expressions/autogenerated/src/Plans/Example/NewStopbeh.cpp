using namespace std;
#include "Plans/Example/NewStopbeh.h"

/*PROTECTED REGION ID(inccpp1449767981309) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1449767981309) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    NewStopbeh::NewStopbeh() :
            DomainBehaviour("NewStopbeh")
    {
        /*PROTECTED REGION ID(con1449767981309) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    NewStopbeh::~NewStopbeh()
    {
        /*PROTECTED REGION ID(dcon1449767981309) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void NewStopbeh::run(void* msg)
    {
        /*PROTECTED REGION ID(run1449767981309) ENABLED START*/ //Add additional options here
        msl_actuator_msgs::MotionControl mcon;
        mcon.motion.angle = 0;
        mcon.motion.rotation = 0;
        mcon.motion.translation = 0;
        send(mcon);
        /*PROTECTED REGION END*/
    }
    void NewStopbeh::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1449767981309) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1449767981309) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
