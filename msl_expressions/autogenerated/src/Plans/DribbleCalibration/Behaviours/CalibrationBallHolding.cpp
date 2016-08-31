using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalibrationBallHolding.h"

/*PROTECTED REGION ID(inccpp1469284294147) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include "container/CNPoint2D.h"
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1469284294147) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	CalibrationBallHolding::CalibrationBallHolding() :
			DomainBehaviour("CalibrationBallHolding")
	{
		/*PROTECTED REGION ID(con1469284294147) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	CalibrationBallHolding::~CalibrationBallHolding()
	{
		/*PROTECTED REGION ID(dcon1469284294147) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void CalibrationBallHolding::run(void* msg)
	{
		/*PROTECTED REGION ID(run1469284294147) ENABLED START*/ //Add additional options here
		if (wm->rawSensorData->getLightBarrier())
		{
			// if ball is in kicker

			// check optical flow before using ballIsRotating
			if (wm->rawSensorData->getOpticalFlow(0) == nullptr)
			{
				return;
			}

			// check if ball is not rotating
			if (ballIsRotating())
			{
				slowTranslationWheelSpeed = slowTranslationWheelSpeed - 50;
				ballWasRotating = true;

			}
			else
			{
				if (!ballWasRotating)
				{
					slowTranslationWheelSpeed = slowTranslationWheelSpeed + 50;
				}
				ballWasStanding = true;
			}

			if (this->minRotation > this->slowTranslationWheelSpeed)
			{
				this->minRotation = this->slowTranslationWheelSpeed;
			}
			writeConfigParameters();
		}
		else
		{
			MotionControl mc = dcc.getBall();
			send(mc);
		}

		if (ballWasRotating && ballWasStanding && !ballIsRotating())
		{
			cout << "finished ball holding calibration!" << endl;
			this->setSuccess(true);
		}
		/*PROTECTED REGION END*/
	}
	void CalibrationBallHolding::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1469284294147) ENABLED START*/ //Add additional options here
		cout << "starting ball holding calibration..." << endl;
		readConfigParameters();
		ballWasStanding = false;
		if (this->minRotation > this->slowTranslationWheelSpeed)
		{
			this->minRotation = this->slowTranslationWheelSpeed;
		}
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1469284294147) ENABLED START*/ //Add additional methods here
	bool CalibrationBallHolding::ballIsRotating()
	{
		shared_ptr<geometry::CNPoint2D> opticalFlow = wm->rawSensorData->getOpticalFlow(0);
		cout << "opticalFlow->x: " << opticalFlow->x << endl;
		return opticalFlow->x > 0 ? true : false;
	}

	void CalibrationBallHolding::readConfigParameters()
	{
		this->minRotation = dcc.readConfigParameter("Dribble.MinRotation");
		this->slowTranslationWheelSpeed = dcc.readConfigParameter("Dribble.SlowTranslationWheelSpeed");
	}

	void CalibrationBallHolding::writeConfigParameters()
	{
		cout << "writing config parameters" << endl;
		cout << "slowTranslationWheelSpeed: " << slowTranslationWheelSpeed << endl;
		cout << "minRotation: " << minRotation << endl;

		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
		(*sys)["Actuation"]->set(boost::lexical_cast<std::string>(slowTranslationWheelSpeed), "Dribble.SlowTranslationWheelSpeed", NULL);
		(*sys)["Actuation"]->set(boost::lexical_cast<std::string>(minRotation), "Dribble.MinRotation", NULL);
	}
/*PROTECTED REGION END*/
} /* namespace alica */
