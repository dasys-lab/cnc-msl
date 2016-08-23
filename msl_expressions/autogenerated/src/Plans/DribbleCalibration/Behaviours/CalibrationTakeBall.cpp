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

		adaptWheel = 0;
		operation = ADD;
		oldOperation = ADD;

		// for output
		queueFilled = false;

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
//		BallHandleCmd bhc;
		// check if robot has the ball
		if (wm->rawSensorData->getLightBarrier())
		{
			// check if ball is rotating correctly
			if (!this->ballRotateCorrect)
			{
				// let ball continuously rotate with speedNoBall (should be by 4000)
				if (!opQueueFilled())
				{
					return;
				}
				int ballRotation = checkBallRotation();

				if (ballRotation == ROTATE_ERR)
				{
					cout << "ROTATE_ERR" << endl;
					return;
				}
				else if (ballRotation == ROTATE_CORRECT)
				{
					cout << "ROTATE_CORRECT" << endl;
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
//			MotionControl mc = dcc.getBall();
//			send(mc);
			return;
		}
		cout << "successfully calibrated the ball taking!" << endl;
		this->setSuccess(true);

		/*PROTECTED REGION END*/
	}
	void CalibrationTakeBall::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1469109429392) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1469109429392) ENABLED START*/ //Add additional methods here
	bool CalibrationTakeBall::opQueueFilled()
	{
		// 10s of rotating the ball
		int queueSize = 285;

		if (wm->rawSensorData->getOpticalFlow(0) == nullptr)
		{
			cout << "no OpticalFLow signal!" << endl;
			this->setFailure(true);
			return false;
		}

		if (opQueue.size() >= queueSize)
		{
			return true;
		}
		if (!queueFilled)
		{
			cout << "filling optical flow queue!" << endl;
			queueFilled = true;
		}
		opQueue.push_back(wm->rawSensorData->getOpticalFlow(0));

		return false;
	}

	int CalibrationTakeBall::checkBallRotation()
	{

		int minX = 125;
		double maxY = 0.99999;

		// average x value should be on max (128)
		// average y value should be at 0 (between 0.99999 and -0.99999)
		// average qos (quality of service) should be existent ... normally between 30 and 40

		double xValue = dcc.getAverageOpticalFlowXValue(opQueue);
		double yValue = dcc.getAverageOpticalFlowYValue(opQueue);
		double qosValue = dcc.getAverageOpticalFlowQOSValue(opQueue);

		cout << "y: " << yValue << endl;

		if (qosValue == 0)
		{
			return ROTATE_ERR;
		}

//		if (xValue > minX && yValue < maxY && yValue > -maxY)
		if (yValue < maxY && yValue > -maxY)
		{
			return ROTATE_CORRECT;

		}
		else if (yValue > maxY)
		{
			// right wheel is too fast so the ball is rotating to the left
			return ROTATE_LEFT;

		}
		else if (yValue < -maxY)
		{
			// left wheel is too fast
			return ROTATE_RIGHT;
		}

		return ROTATE_ERR;
	}

	/**
	 * saves the currently changing wheel for further adaption
	 * if the ball is rotation right the left wheel need to be slowed
	 * if the ball is rotation left the right wheel need to be slowed
	 * if the ball is rotation to slow the dribbleFactor of both wheels need to be adapted
	 */
	void CalibrationTakeBall::correctWheelSpeed(int rotation)
	{
		if (rotation != ROTATE_RIGHT && rotation != ROTATE_LEFT && rotation != ROTATE_TOO_SLOW)
		{
			cout << "CalibrationTakeBall::correctWheelSpeed(int wheel) -> wrong input!" << endl;
			return;
		}
		if (rotation == ROTATE_TOO_SLOW)
		{
			// increase booth wheels speed (decrease dribbleFactor)
			cout << "ball is rotating too slow!" << endl;
			return;
		}
		// check which wheel need to be corrected and safes it so we know in further iterations which wheel we need to fixed
		if (adaptWheel == 0)
		{
			adaptWheel = rotation;
		}
		// check if the defect wheel is too fast or to slow
		if (rotation == adaptWheel)
		{
			dribbleFactorLeft = adaptWheel == ROTATE_RIGHT ? dribbleFactorLeft + changingFactor : dribbleFactorLeft;
			dribbleFactorRight = adaptWheel == ROTATE_LEFT ? dribbleFactorRight + changingFactor : dribbleFactorRight;
			operation = ADD;
		}
		else
		{
			dribbleFactorLeft = adaptWheel == ROTATE_RIGHT ? dribbleFactorLeft - changingFactor : dribbleFactorLeft;
			dribbleFactorRight = adaptWheel == ROTATE_LEFT ? dribbleFactorRight - changingFactor : dribbleFactorRight;
			operation = SUB;
		}
		cout << "operation: " << operation << endl;
		cout << "oldOperation: " << oldOperation << endl;
		if (operation != oldOperation)
		{
			changingFactor = changingFactor / 2;
			oldOperation = operation == ADD ? SUB : ADD;
		}
		else
		{
			oldOperation = operation;
		}
		opQueue.clear();
		queueFilled = false;
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
		changingFactor = 0.5;
	}

	void CalibrationTakeBall::writeConfigParameters()
	{
		cout << "writing config parameters" << endl;
		cout << "speedNoBall:               " << speedNoBall << endl;
		cout << "slowTranslationWheelSpeed: " << slowTranslationWheelSpeed << endl;
		cout << "minRotation:               " << minRotation << endl;
		cout << "dribbleFactorRight:        " << dribbleFactorLeft << endl;
		cout << "dribbleFlactorLeft:        " << dribbleFactorRight << endl;

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
