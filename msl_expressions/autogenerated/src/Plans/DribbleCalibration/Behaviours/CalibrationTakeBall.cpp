using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalibrationTakeBall.h"

/*PROTECTED REGION ID(inccpp1469109429392) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include "container/CNPoint2D.h"
//#include <msl_actuator_msgs/BallHandleCmd.h>

/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1469109429392) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CalibrationTakeBall::CalibrationTakeBall() :
            DomainBehaviour("CalibrationTakeBall")
    {
        /*PROTECTED REGION ID(con1469109429392) ENABLED START*/ //Add additional options here
        ballRotateCorrect = false;
        ballHoldCorrect = false;

        dribbleFactorRightOld = 0;

        defectWheel = 0;

        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    CalibrationTakeBall::~CalibrationTakeBall()
    {
        /*PROTECTED REGION ID(dcon1469109429392) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CalibrationTakeBall::run(void* msg)
    {
    	// TODO: remove when finished testing
    	this->setSuccess(true);
    	return;

        /*PROTECTED REGION ID(run1469109429392) ENABLED START*/ //Add additional options here
//		BallHandleCmd bhc;
        // check if robot has the ball
        if (wm->rawSensorData->getLightBarrier())
        {
            // check if ball is rotating correctly
            if (!this->ballRotateCorrect)
            {
                // let ball continuously rotate with speedNoBall (should be by 4000)
//				bhc.leftMotor = this->speedNoBall;
//				bhc.rightMotor = this->speedNoBall;
//				send(bhc);
                int ballRotation = checkBallRotation();

                if (ballRotation == ROTATE_ERR)
                {
                    return;
                }
                else if (ballRotation == ROTATE_CORRECT)
                {
                    this->ballRotateCorrect = true;
                }
                else if (ballRotation == ROTATE_LEFT)
                {
                    // ROTATE_LEFT means that the right wheel is spinning too fast so we need to correct the right wheel
                    correctWheelSpeed(ROTATE_LEFT);
                    writeConfigParameters();
                }
                else if (ballRotation == ROTATE_RIGHT)
                {
                    // ROTATE_RIGHT means that the left wheel is spinning too fast so we need to correct the left wheel
                    correctWheelSpeed(ROTATE_RIGHT);
                    writeConfigParameters();
                }
                return;
            }

        }
        else
        {
            MotionControl mc = dcc.getBall();
            send(mc);
            return;
        }

        this->setSuccess(true);

        /*PROTECTED REGION END*/
    }
    void CalibrationTakeBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1469109429392) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1469109429392) ENABLED START*/ //Add additional methods here
    int CalibrationTakeBall::checkBallRotation()
    {
        // read optical flow value
        shared_ptr < geometry::CNPoint2D > opticalFlowValues = wm->rawSensorData->getOpticalFlow(0);

        if (wm->rawSensorData->getOpticalFlow(10) == nullptr)
        {
            return ROTATE_ERR;
        }

        shared_ptr < geometry::CNPoint2D > opticalFlowValuesOld = wm->rawSensorData->getOpticalFlow(10);

        // if ball isn't rotating straight -> correct values in config values of dribbleFactor left and right
        // values should be:
        // -> high negativ y
        // -> x near 0
        double yDifference = opticalFlowValues->y - opticalFlowValuesOld->y;
        double xDifference = fabs(opticalFlowValues->x) - fabs(opticalFlowValuesOld->x);
        double yToleranceValue = 100;
        double xToleranceValue = 100;

        // decide if ball is rotating straight

        // check if ball if rotating with the correct speed, when the robot is holding the ball with speedNoBall (getBall)
        if (yDifference > -yToleranceValue)
        {
            // ball is rotating to slow
            return ROTATE_TOO_SLOW;
        }

        if (opticalFlowValues->x > 0)
        {
            if (opticalFlowValues->x > xToleranceValue)
            {
                //decrease speed of left actuator
                // -> increase DribbleFactorLeft
                return ROTATE_RIGHT;
            }
        }
        else
        {
            if (opticalFlowValues->x < xToleranceValue)
            {
                //decrease speed of right actuator
                // -> increase DribbleFactorRight
                return ROTATE_LEFT;
            }
        }

        return ROTATE_CORRECT;
    }

    void CalibrationTakeBall::correctWheelSpeed(int wheel)
    {
        if (wheel != ROTATE_RIGHT && wheel != ROTATE_LEFT)
        {
            cout << "CalibrationTakeBall::correctWheelSpeed(int wheel) -> wrong input!" << endl;
            return;
        }
        // check which wheel need to be corrected and safes it so we know in further iterations which wheel we need to fixed
        if (defectWheel == 0)
        {
            defectWheel = wheel;
        }

        // check if the defect wheel is too fast or to slow
        if (wheel == defectWheel)
        {
            dribbleFactorLeft = defectWheel == ROTATE_RIGHT ? dribbleFactorLeft + changingFactor : dribbleFactorLeft;
            dribbleFactorRight = defectWheel == ROTATE_LEFT ? dribbleFactorRight + changingFactor : dribbleFactorRight;
        }
        else
        {
            dribbleFactorLeft = defectWheel == ROTATE_RIGHT ? dribbleFactorLeft - changingFactor : dribbleFactorLeft;
            dribbleFactorRight = defectWheel == ROTATE_LEFT ? dribbleFactorRight - changingFactor : dribbleFactorRight;
        }

        changingFactor = changingFactor / 2;

    }

    void CalibrationTakeBall::readConfigParameters()
    {
        supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
        DribbleCalibrationContainer dcc;
        speedNoBall = dcc.readConfigParameter("Dribble.SpeedNoBall");
        slowTranslationWheelSpeed = dcc.readConfigParameter("Dribble.SlowTranslationWheelSpeed");
        minRotation = dcc.readConfigParameter("Dribble.MinRotation");
        // left and right are swapped!!!!
        dribbleFactorLeft = dcc.readConfigParameter("Dribble.DribbleFactorRight");
        dribbleFactorRight = dcc.readConfigParameter("Dribble.DribbleFactorLeft");

        // maybe put in config
        changingFactor = 1;
    }

    void CalibrationTakeBall::writeConfigParameters()
    {
        supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (speedNoBall), "Dribble.SpeedNoBall", NULL);
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (slowTranslationWheelSpeed),
                                 "Dribble.SlowTranslationWheelSpeed", NULL);
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (minRotation), "Dribble.MinRotation", NULL);
        // left and right are swapped!!
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (dribbleFactorLeft), "Dribble.DribbleFactorRight",
                                 NULL);
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (dribbleFactorRight), "Dribble.DribbleFactorLeft",
                                 NULL);

        (*sys)["Actuation"]->store();
    }
/*PROTECTED REGION END*/
} /* namespace alica */
