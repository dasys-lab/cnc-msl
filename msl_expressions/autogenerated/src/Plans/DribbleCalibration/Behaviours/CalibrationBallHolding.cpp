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
		// TODO: remove when finished testing
		this->setSuccess(true);
		return;

		if (wm->rawSensorData->getLightBarrier())
		{
			// if ball is in kicker

			// check optical flow before using ballIsRotating
			if (wm->rawSensorData->getOpticalFlow(10) == nullptr)
			{
				return;
			}

			// check if ball is not rotating
			if (ballIsRotating())
			{
				this->slowTranslationWheelSpeed = this->slowTranslationWheelSpeed - 50;

				if (this->minRotation > this->slowTranslationWheelSpeed)
				{
					this->minRotation = this->slowTranslationWheelSpeed;
				}
				writeConfigParameters();
				return;
			}
		}
		else
		{
			MotionControl mc = dcc.getBall();
			send(mc);
		}

		this->setSuccess(true);
		/*PROTECTED REGION END*/
	}
	void CalibrationBallHolding::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1469284294147) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1469284294147) ENABLED START*/ //Add additional methods here
	bool CalibrationBallHolding::ballIsRotating()
	{
		double rotationTolerance = 10;

		shared_ptr<geometry::CNPoint2D> opticalFlow = wm->rawSensorData->getOpticalFlow(0);
		shared_ptr<geometry::CNPoint2D> opticalFlowOld = wm->rawSensorData->getOpticalFlow(10);

		double yDifference = opticalFlow->y - opticalFlowOld->y;

		return yDifference > rotationTolerance ? true : false;
	}

	void CalibrationBallHolding::readConfigParameters()
	{
		this->minRotation = dcc.readConfigParameter("Dribble.MinRotation");
		this->slowTranslationWheelSpeed = dcc.readConfigParameter("Dribble.SlowTranslationWheelSpeed");
	}

	void CalibrationBallHolding::writeConfigParameters()
	{

	}
/*PROTECTED REGION END*/
} /* namespace alica */
