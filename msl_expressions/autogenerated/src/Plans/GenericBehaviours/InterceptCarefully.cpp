using namespace std;
#include "Plans/GenericBehaviours/InterceptCarefully.h"

/*PROTECTED REGION ID(inccpp1427703218101) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1427703218101) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    InterceptCarefully::InterceptCarefully() :
            DomainBehaviour("InterceptCarefully")
    {
        /*PROTECTED REGION ID(con1427703218101) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    InterceptCarefully::~InterceptCarefully()
    {
        /*PROTECTED REGION ID(dcon1427703218101) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void InterceptCarefully::run(void* msg)
    {
        /*PROTECTED REGION ID(run1427703218101) ENABLED START*/ //Add additional options here
        auto me = wm->rawSensorData.getOwnPositionVision();
        auto egoBallPos = wm->ball.getEgoBallPosition();
        MotionControl mc;
        if (me == nullptr || egoBallPos == nullptr)
        {
            send(mc);
            return;
        }

        mc = msl::RobotMovement::interceptCarefully(egoBallPos, egoBallPos, 100);

        send(mc);

        /*PROTECTED REGION END*/
    }
    void InterceptCarefully::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1427703218101) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1427703218101) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
