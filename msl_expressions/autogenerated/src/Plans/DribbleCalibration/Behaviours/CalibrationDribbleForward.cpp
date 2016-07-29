using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalibrationDribbleForward.h"

/*PROTECTED REGION ID(inccpp1469116853584) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include "container/CNPoint2D.h"

#define DEBUG_CDF
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1469116853584) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	CalibrationDribbleForward::CalibrationDribbleForward() :
			DomainBehaviour("CalibrationDribbleForward")
	{
		/*PROTECTED REGION ID(con1469116853584) ENABLED START*/ //Add additional options here
		sections.reserve(SECTIONS_SIZE);
#ifdef DEBUG_CDF
		cout << "CalibrationDribbleForward: sections.size() = " << sections.size() << endl;
#endif

		moveCount = 0;
		correctRotationCount = 0;
		haveBallCount = 0;
		readConfigParameters();
		/*PROTECTED REGION END*/
	}
	CalibrationDribbleForward::~CalibrationDribbleForward()
	{
		/*PROTECTED REGION ID(dcon1469116853584) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void CalibrationDribbleForward::run(void* msg)
	{
		/*PROTECTED REGION ID(run1469116853584) ENABLED START*/ //Add additional options here
		MotionControl mc;

		if (wm->rawSensorData->getLightBarrier())
		{
			haveBallCount++;
			// if ball is in kicker
			// drive forward start slowly
			// start with 400 speed
			while (moveCount < SECTIONS_SIZE)
			{
				int translation = 400;
				dcc.move(dcc.DRIBBLE_FORWARD, (moveCount + 1) * translation);

				// use optical flow and light barrier data to analyze the the ball movement
				int ballMovement = checkBallRotation();

				// adapt actuatorSpeed at specific robotSpeed
				if (ballMovement == CORRECT)
				{
					correctRotationCount++;
				}
				else if (ballMovement == TOO_SLOW)
				{
					adaptWheelSpeed(TOO_SLOW);
					correctRotationCount = 0;
				}

				// if robot can handle ball at this speed -> increase the speed and repeat
				if (correctRotationCount > MIN_ROTATE_NUMBER)
				{
					moveCount++;
					correctRotationCount = 0;
				}
			}
		}
		else
		{
			// if we lose the ball -> our actuator are rotating to fast
			if (haveBallCount > 0)
			{
				adaptWheelSpeed(TOO_FAST);
				correctRotationCount = 0;
			}
			haveBallCount = 0;
			mc = dcc.getBall();
			send(mc);
		}
		writeConfigParameters();
		/*PROTECTED REGION END*/
	}
	void CalibrationDribbleForward::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1469116853584) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1469116853584) ENABLED START*/ //Add additional methods here
	void CalibrationDribbleForward::adaptWheelSpeed(int err)
	{
		if (err != TOO_FAST && err != TOO_SLOW)
		{
			cout << "CalibrationDribbleForward::correctWheelSpeed -> wrong input!" << endl;
			return;
		}

		if (err == TOO_FAST)
			sections[moveCount].actuatorSpeed = sections[moveCount].actuatorSpeed - changingFactor;

		if (err == TOO_SLOW)
			sections[moveCount].actuatorSpeed = sections[moveCount].actuatorSpeed + changingFactor;
	}

	int CalibrationDribbleForward::checkBallRotation()
	{
		// read optical flow value
		shared_ptr<geometry::CNPoint2D> opticalFlowValues = wm->rawSensorData->getOpticalFlow(0);

		if (wm->rawSensorData->getOpticalFlow(10) == nullptr)
		{
			return ROTATE_ERR;
		}

		shared_ptr<geometry::CNPoint2D> opticalFlowValuesOld = wm->rawSensorData->getOpticalFlow(10);

		double yDifference = opticalFlowValues->y - opticalFlowValuesOld->y;

		// TODO check optical flow

		// too slow or not moving
		// correct
		return false;
	}

	void CalibrationDribbleForward::fillSections(shared_ptr<vector<string> > speedsSections)
	{
		int i = 0;
		for (string subsection : *speedsSections)
		{
			sections[i].name = subsection.c_str();
			sections[i].robotSpeed = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(),
																		"robotSpeed", NULL);
			sections[i].actuatorSpeed = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(),
																		"actuatorSpeed", NULL);
			cout << "RobotSpeed: " << sections[i].robotSpeed << "actuatorSpeed: " << sections[i].actuatorSpeed << endl;
			i++;
		}

	}

	void CalibrationDribbleForward::createSections()
	{
		double speedFactor = MAX_SPEED / SECTIONS_SIZE;
		for (int i = 0; i < SECTIONS_SIZE; i++)
		{
			sections[i].name = "P" + std::to_string(i + 1);
			sections[i].robotSpeed = i * speedFactor;
			sections[i].actuatorSpeed = i * speedFactor;
		}
	}

	void CalibrationDribbleForward::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		shared_ptr<vector<string> > speedsSections = (*sc)["Actuation"]->getSections("ForwardDribbleSpeeds", NULL);

		if (speedsSections->size() == SECTIONS_SIZE)
		{
			fillSections(speedsSections);
		}
		else
		{
			createSections();
		}

		// maybe move to config file later
		changingFactor = 100;

	}

	void CalibrationDribbleForward::writeConfigParameters()
	{
		// as an example -> improve later!
//		subsection s1;
//		s1.name = "P9";
//		s1.actuatorSpeed = 300;
//		s1.robotSpeed = 400;
//		shared_ptr<vector<subsection>> sections;
//		sections->push_back(s1);

		dcc.writeConfigParameters(sections);
	}
/*PROTECTED REGION END*/
} /* namespace alica */
