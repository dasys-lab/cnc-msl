using namespace std;
#include "Plans/Behaviours/DribbleToPoint.h"

/*PROTECTED REGION ID(inccpp1414752367688) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1414752367688) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleToPoint::DribbleToPoint() :
            DomainBehaviour("DribbleToPoint")
    {
        /*PROTECTED REGION ID(con1414752367688) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    DribbleToPoint::~DribbleToPoint()
    {
        /*PROTECTED REGION ID(dcon1414752367688) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleToPoint::run(void* msg)
    {
        /*PROTECTED REGION ID(run1414752367688) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData.getOwnPositionVision();
        auto egoBallPos = wm->ball.getEgoBallPosition();
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }
        /*PROTECTED REGION END*/
    }
    void DribbleToPoint::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1414752367688) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1414752367688) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
