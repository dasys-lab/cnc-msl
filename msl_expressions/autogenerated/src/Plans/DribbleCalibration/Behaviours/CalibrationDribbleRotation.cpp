using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalibrationDribbleRotation.h"

/*PROTECTED REGION ID(inccpp1469196350730) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include "container/CNPoint2D.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1469196350730) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CalibrationDribbleRotation::CalibrationDribbleRotation() :
            DomainBehaviour("CalibrationDribbleRotation")
    {
        /*PROTECTED REGION ID(con1469196350730) ENABLED START*/ //Add additional options here
    	curveRotationfactor = 0;
        /*PROTECTED REGION END*/
    }
    CalibrationDribbleRotation::~CalibrationDribbleRotation()
    {
        /*PROTECTED REGION ID(dcon1469196350730) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CalibrationDribbleRotation::run(void* msg)
    {
        /*PROTECTED REGION ID(run1469196350730) ENABLED START*/ //Add additional options here

        /*PROTECTED REGION END*/
    }
    void CalibrationDribbleRotation::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1469196350730) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1469196350730) ENABLED START*/ //Add additional methods here
    void CalibrationDribbleRotation::readConfigParameters()
    {
        curveRotationfactor = dcc.readConfigParameter("Dribble.CurveRotationFactor");
    }
/*PROTECTED REGION END*/
} /* namespace alica */
