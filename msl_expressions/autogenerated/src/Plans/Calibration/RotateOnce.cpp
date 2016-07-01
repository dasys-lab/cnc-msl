using namespace std;
#include "Plans/Calibration/RotateOnce.h"

/*PROTECTED REGION ID(inccpp1467397900274) ENABLED START*/ //Add additional includes here
#include<MSLWorldModel.h>
#include <Game.h>
#include <RawSensorData.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1467397900274) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    RotateOnce::RotateOnce() :
            DomainBehaviour("RotateOnce")
    {
        /*PROTECTED REGION ID(con1467397900274) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    RotateOnce::~RotateOnce()
    {
        /*PROTECTED REGION ID(dcon1467397900274) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void RotateOnce::run(void* msg)
    {
        /*PROTECTED REGION ID(run1467397900274) ENABLED START*/ //Add additional options here
        cout << wm->rawSensorData->getAverageBearing() << endl;
        /*PROTECTED REGION END*/
    }
    void RotateOnce::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1467397900274) ENABLED START*/ //Add additional options here
        initialBearing = wm->rawSensorData->getAverageBearing();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1467397900274) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
