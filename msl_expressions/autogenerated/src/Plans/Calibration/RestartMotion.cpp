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
        uninitializedMotionCounter = 0;
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
        if (this->isSuccess())
        {
            return;
        }
        uninitializedMotionCounter++;

        if (uninitializedMotionCounter == 200)
        {
            wm->sendStartMotionCommand();
            return;
        }
        else if (uninitializedMotionCounter < 200)
        {
            return;
        }

        if (uninitializedMotionCounter != 400)
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
        uninitializedMotionCounter = 0;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1472657511112) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
