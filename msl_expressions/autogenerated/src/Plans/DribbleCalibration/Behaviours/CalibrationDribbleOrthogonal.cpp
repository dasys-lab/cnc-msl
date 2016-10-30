using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalibrationDribbleOrthogonal.h"

/*PROTECTED REGION ID(inccpp1469196321064) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include "container/CNPoint2D.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1469196321064) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CalibrationDribbleOrthogonal::CalibrationDribbleOrthogonal() :
            DomainBehaviour("CalibrationDribbleOrthogonal")
    {
        /*PROTECTED REGION ID(con1469196321064) ENABLED START*/ //Add additional options here
    	orthoDriveFactor = 0;
        /*PROTECTED REGION END*/
    }
    CalibrationDribbleOrthogonal::~CalibrationDribbleOrthogonal()
    {
        /*PROTECTED REGION ID(dcon1469196321064) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CalibrationDribbleOrthogonal::run(void* msg)
    {
        /*PROTECTED REGION ID(run1469196321064) ENABLED START*/ //Add additional options here
        if (wm->ball->haveBall())
        {
            // check code and maybe add a new parameter for the orthogonal calculation

            // if ball is in kicker
            // drive to the left
            // check sensor data -> optical flow should say, that the ball isn't moving
            // correct data if the robot is loosing the ball
            // drive to the right
            // check sensor data -> optical flow should say, that the ball isn't moving
            // correct data if the robot is loosing the ball

        }
        else
        {
            MotionControl mc = dcc.getBall();
            send(mc);
        }
        /*PROTECTED REGION END*/
    }
    void CalibrationDribbleOrthogonal::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1469196321064) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1469196321064) ENABLED START*/ //Add additional methods here
    void CalibrationDribbleOrthogonal::readConfigParameters()
    {
        orthoDriveFactor = dcc.readConfigParameter("Dribble.OrthoDriveFactor");
    }
/*PROTECTED REGION END*/
} /* namespace alica */
