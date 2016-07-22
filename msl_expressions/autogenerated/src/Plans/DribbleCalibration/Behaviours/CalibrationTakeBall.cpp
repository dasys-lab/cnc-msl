using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalibrationTakeBall.h"

/*PROTECTED REGION ID(inccpp1469109429392) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include "container/CNPoint2D.h"
#include <msl_actuator_msgs/BallHandleCmd.h>
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
		/*PROTECTED REGION ID(run1469109429392) ENABLED START*/ //Add additional options here
		BallHandleCmd bhc;

		// check if robot has the ball
		if (wm->rawSensorData->getLightBarrier())
		{
			// check if ball is rotating correctly
			if (ballRotateCorrect)
			{
				// let ball continuously rotate with speedNoBall (should be by 4000)
				bhc.leftMotor = speedNoBall;
				bhc.rightMotor = speedNoBall;

				// read optical flow value
				shared_ptr<geometry::CNPoint2D> opticalFlowValues = wm->rawSensorData->getOpticalFlow(0);

				if (wm->rawSensorData->getOpticalFlow(10) == nullptr)
				{
					return;
				}

				shared_ptr<geometry::CNPoint2D> opticalFLowValuesOld = wm->rawSensorData->getOpticalFlow(10);

				// decide if ball is rotating straight
				if (checkBallRotation())
				{
					ballRotateCorrect = true;
					return;
				}
				// if ball isn't rotating straight -> correct values in config values of dribbleFactor left and right
				// values should be:
				// -> high negativ y
				// -> x near 0
				double yDifference = opticalFlowValues->y - opticalFLowValuesOld->y;
				double xDifference = fabs(opticalFlowValues->x) - fabs(opticalFLowValuesOld->x);
				double yToleranceValue = 100;
				double xToleranceValue = 100;

				if (yDifference < 0)
				{
					if (yDifference > -yToleranceValue)
					{
						speedNoBall = speedNoBall - 100;
						return;
					}
				}
				else
				{
					speedNoBall = -speedNoBall;
				}

				if (opticalFlowValues->x > 0)
				{
					if (opticalFlowValues->x > xToleranceValue)
					{
						//decrease speed of left actuator
						// -> increase DribbleFactorLeft
					}
				}
				else
				{
					if (opticalFlowValues->x < xToleranceValue)
					{
						//decrease speed of right actuator
						// -> increase DribbleFactorRight
					}
				}
			}

			// if the ball is rotating straight
			// check minRotation and slowTranslationWheelSpeed -> ball need to be in kicker but he may not rotating
		}
		else
		{
			MotionControl mc = dcc.getBall();
			send(mc);
		}
		this->setSuccess(true);
//        writeConfigParameters();
		/*PROTECTED REGION END*/
	}
	void CalibrationTakeBall::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1469109429392) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1469109429392) ENABLED START*/ //Add additional methods here
	bool CalibrationTakeBall::checkBallRotation()
	{
		return false;
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
	}

	void CalibrationTakeBall::writeConfigParameters()
	{
		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
		(*sys)["Actuation"]->set(boost::lexical_cast<std::string>(speedNoBall), "Dribble.SpeedNoBall", NULL);
		(*sys)["Actuation"]->set(boost::lexical_cast<std::string>(slowTranslationWheelSpeed),
									"Dribble.SlowTranslationWheelSpeed", NULL);
		(*sys)["Actuation"]->set(boost::lexical_cast<std::string>(minRotation), "Dribble.MinRotation", NULL);
		// left and right are swapped!!
		(*sys)["Actuation"]->set(boost::lexical_cast<std::string>(dribbleFactorLeft), "Dribble.DribbleFactorRight",
		NULL);
		(*sys)["Actuation"]->set(boost::lexical_cast<std::string>(dribbleFactorRight), "Dribble.DribbleFactorLeft",
		NULL);

		(*sys)["Actuation"]->store();
	}
/*PROTECTED REGION END*/
} /* namespace alica */
