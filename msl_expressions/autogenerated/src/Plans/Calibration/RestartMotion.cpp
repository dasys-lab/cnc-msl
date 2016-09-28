using namespace std;
#include "Plans/Calibration/RestartMotion.h"

/*PROTECTED REGION ID(inccpp1472657511112) ENABLED START*/ //Add additional includes here
#include<MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1472657511112) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    RestartMotion::RestartMotion() :
            DomainBehaviour("RestartMotion")
    {
        /*PROTECTED REGION ID(con1472657511112) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    RestartMotion::~RestartMotion()
    {
        /*PROTECTED REGION ID(dcon1472657511112) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void RestartMotion::run(void* msg)
    {
        /*PROTECTED REGION ID(run1472657511112) ENABLED START*/ //Add additional options here
        uninitializedCounter++;

        if (uninitializedCounter == 200)
        {
            wm->sendStartMotionCommand();
            return;
        }
        else if (uninitializedCounter < 200)
        {
            return;
        }

        if (uninitializedCounter != 400)
        {
            return;
        }

        this->setSuccess(true);

        /*PROTECTED REGION END*/
    }
    void RestartMotion::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1472657511112) ENABLED START*/ //Add additional options here
        wm->sendKillMotionCommand();
        uninitializedCounter = 0;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1472657511112) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
