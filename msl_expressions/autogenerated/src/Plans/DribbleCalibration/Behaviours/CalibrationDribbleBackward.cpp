using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalibrationDribbleBackward.h"

/*PROTECTED REGION ID(inccpp1469196252478) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include "container/CNPoint2D.h"
#include <Plans/DribbleCalibration/Container/DribbleCalibrationContainer.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1469196252478) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CalibrationDribbleBackward::CalibrationDribbleBackward() :
            DomainBehaviour("CalibrationDribbleBackward")
    {
        /*PROTECTED REGION ID(con1469196252478) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    CalibrationDribbleBackward::~CalibrationDribbleBackward()
    {
        /*PROTECTED REGION ID(dcon1469196252478) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CalibrationDribbleBackward::run(void* msg)
    {
        /*PROTECTED REGION ID(run1469196252478) ENABLED START*/ //Add additional options here
    	this->setSuccess(true);
    	return;
        if (wm->ball->haveBall())
        {
            // try to follow the forward calibration
            // => adapt backward calibration with new parameters
            // => create new subsection in Actuation config for BackwardDribbleSpeed

            // if ball is in kicker
            // drive backward start slowly
            // start with 300 speed
            // use optical flow and light barrier data to analyze the the ball movement
            // adapt actuatorSpeed by specific robotSpeed
            // if robot can handle ball at this speed -> increase the speed and repeat
        }
        else
        {
            DribbleCalibrationContainer dcc;
            MotionControl mc = dcc.getBall();
            send(mc);
        }
        /*PROTECTED REGION END*/
    }
    void CalibrationDribbleBackward::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1469196252478) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1469196252478) ENABLED START*/ //Add additional methods here
    void CalibrationDribbleBackward::readConfigParameters()
    {

    }
/*PROTECTED REGION END*/
} /* namespace alica */
