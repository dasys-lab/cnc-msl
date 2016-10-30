using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalibrationDribbleForward.h"

/*PROTECTED REGION ID(inccpp1469116853584) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include "container/CNPoint2D.h"

/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1469116853584) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	CalibrationDribbleForward::CalibrationDribbleForward() :
			DomainBehaviour("CalibrationDribbleForward")
	{
		/*PROTECTED REGION ID(con1469116853584) ENABLED START*/ //Add additional options here
		increaseSpeed = false;
		correctRotationCount = 0;
		changingValue = 0;
		maxSpeed = 0;
		moveCount = 0;
		minRotation = 0;
		sectionSize = 0;
		minRotationNumber = 0;
		haveBallCount = 0;
		maxRotation = 0;
		getBallCount = 0;
		lastOpticalFlowValue = 1;
		haveBallWaitingDuration = 0;
		minCalibrationSpeed = 0;
		collectDataWaitingDuration = 0;
		changeDirections = false;
		alloAlignPoint = nullptr;
		query = nullptr;
		startTrans = 0;
		getBallFlag = true;
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
		msl::RobotMovement rm;
//		writeConfigParameters();
//        return;
//        if (wm->rawSensorData->getLightBarrier())
		if (1 == 1)
		{
			getBallFlag = true;
			// waiting so we definitely have the ball when we start with the calibration
			if (haveBallCount == 0 || (getBallCount > 0 && getBallCount < haveBallWaitingDuration))
			{
				haveBallCount++;
				getBallCount++;
				return;
			}
			else
			{
				getBallCount = 0;
			}
			haveBallCount++;
			// if ball is in kicker
			// drive forward start slowly
			// start with 400 speed
			if (moveCount < sectionSize)
			{
				// checking if we drive fast enough ... if we drive to slowly we won't be able to calibrate correctly
				if (sections[sectionSize - (moveCount + 1)].robotSpeed > (-minCalibrationSpeed))
				{
					cout << "sections[" << sectionSize - (moveCount + 1) << "].robotSpeed = "
							<< sections[sectionSize - (moveCount + 1)].robotSpeed << endl;
					cout << "robot Speed is too small to calibrate! Continue with next step..." << endl;
					moveCount++;
					haveBallCount = 0;
					getBallCount = 0;
					return;
				}

				// check if there is an obstacle
				if (!changeDirections)
					changeDirections = dcc.checkObstacles(dcc.Forward, 1000);

				// if there is an obstacle find a new way to drive and turn around
				if (changeDirections)
				{
					cout << "alloAlignPoint == nullptr : " << (alloAlignPoint == nullptr ? "true" : "false") << endl;
					auto me = wm->rawSensorData->getOwnPositionVision();

					if (alloAlignPoint == nullptr)
					{
						alloAlignPoint = dcc.calcNewAlignPoint()->egoToAllo(*me);
					}

					shared_ptr<geometry::CNPoint2D> newAlignPoint = alloAlignPoint->alloToEgo(*me);


					cout << "newAlignPoint: " << newAlignPoint->toString() << endl;
					cout << "newALignPoint->angleTo() = " << fabs(newAlignPoint->angleTo()) << endl;
					if (fabs(newAlignPoint->angleTo()) < (M_PI - query->angleTolerance))
					{

						// x and y my not be 0 otherwise the angleTo method will return 0
						newAlignPoint->x = (newAlignPoint->x == 0) ? (newAlignPoint->x = 1) : newAlignPoint->x;
						newAlignPoint->y = (newAlignPoint->y == 0) ? (newAlignPoint->y = 1) : newAlignPoint->y;

						query->egoAlignPoint = newAlignPoint;
						query->rotateAroundTheBall = false;
						query->angleTolerance = 0.2;

						cout << "aligning to new align point!" << endl;
						mc = rm.alignTo(query);
						send(mc);
						return;
					}
					else if (newAlignPoint != nullptr)
					{
						cout << "reset changeDirection flag and newAlignPoint" << endl;
						changeDirections = false;
						newAlignPoint = nullptr;
					}
				}
				else
				{
					mc = dcc.move(dcc.Forward, (moveCount + 1) * startTrans);
					send(mc);
				}

				// waiting some time till we can be sure to only collect correct values
				if (haveBallCount < (haveBallWaitingDuration + collectDataWaitingDuration))
				{
					return;
				}

				// use optical flow and light barrier data to analyze the ball movement
//                Rotation ballMovement = checkBallRotation();
				Rotation ballMovement = RotateErr;
				// adapt actuatorSpeed at specific robotSpeed
				if (ballMovement == Correct)
				{
					correctRotationCount++;
				}
				else if (ballMovement == TooSlow)
				{
					adaptWheelSpeed(TooSlow);
					correctRotationCount = 0;
				}

				// if robot can handle ball at this speed -> increase the speed and repeat
				if (correctRotationCount > minRotationNumber)
				{
					moveCount++;
					haveBallCount = 0;
					correctRotationCount = 0;
				}
			}
			else
			{
				cout << "finished dribble forward calibration!" << endl;
				this->setSuccess(true);
			}
		}
		else
		{
			if (getBallFlag)
			{
				cout << "getting ball" << endl;
			}

			// if we lose the ball -> our actuator are rotating to fast
			if (haveBallCount > 0 && !changeDirections)
			{
				adaptWheelSpeed(TooFast);
				correctRotationCount = 0;
			}
			haveBallCount = 0;
			mc = dcc.getBall();
			if (mc.motion.translation != NAN)
			{
				send(mc);
			}
			else
			{
				cerr << "motion command is NAN!" << endl;
			}
		}

		/*PROTECTED REGION END*/
	}
	void CalibrationDribbleForward::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1469116853584) ENABLED START*/ //Add additional options here
		cout << "starting dribble forward calibration..." << endl;
		readConfigParameters();
#ifdef DEBUG_DC
		cout << "CalibrationDribbleForward: sections.size() = " << sections.size() << endl;
#endif

		query = make_shared<msl::MovementQuery>();
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1469116853584) ENABLED START*/ //Add additional methods here
	/**
	 * @parm err can be TooFast or TooSlow
	 *
	 * The method will adapt the actuator speed in sections
	 */
	void CalibrationDribbleForward::adaptWheelSpeed(Rotation err)
	{
		if (err != TooFast && err != TooSlow)
		{
			cout << "CalibrationDribbleForward::correctWheelSpeed -> wrong input!" << endl;
			return;
		}
		cout << "moveCount = " << moveCount << endl;
		int counter = sectionSize - (moveCount + 1);
		if (err == TooFast)
			sections[counter].actuatorSpeed = sections[counter].actuatorSpeed + (changingValue * 20);

		if (err == TooSlow)
			sections[counter].actuatorSpeed = sections[counter].actuatorSpeed - changingValue;
		cout << "sections[" << counter << "].robotSpeed = " << sections[counter].robotSpeed << endl;
		cout << "sections[" << counter << "].actuatorSpeed = " << sections[counter].actuatorSpeed << endl;
		writeConfigParameters();
	}

	/**
	 * @return TooSlow if the ball isn't rotating (OpticalFlow->x = 0)
	 * @return Correct if the ball is rotating
	 */
	CalibrationDribbleForward::Rotation CalibrationDribbleForward::checkBallRotation()
	{
		// read optical flow value

		if (wm->rawSensorData->getOpticalFlow(0) == nullptr)
		{
			cout << "couldn't read optical flow value!" << endl;
			return RotateErr;
		}
		shared_ptr<geometry::CNPoint2D> opticalFlowValues = wm->rawSensorData->getOpticalFlow(0);

		cout << "opticalFlowValue x: " << opticalFlowValues->x;

		// too slow or not moving
		if (opticalFlowValues->x == 0 && lastOpticalFlowValue == 0)
		{
//        	opQueue->clear();
			cout << " -> Too Slow" << endl;
			return TooSlow;
		}

		lastOpticalFlowValue = opticalFlowValues->x;
		// correct
		cout << " -> Correct" << endl;
		return Correct;
	}

	/**
	 * fill class variable sections with config parameters
	 */
	void CalibrationDribbleForward::fillSections(shared_ptr<vector<string> > speedsSections)
	{
		int i = 0;
#ifdef DEBUG_DC
		cout << "speedSections size: " << speedsSections->size() << endl;
#endif
		for (string subsection : *speedsSections)
		{
			sections[i].name = subsection.c_str();
			sections[i].robotSpeed = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(),
																		"robotSpeed", NULL);
			sections[i].actuatorSpeed = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(),
																		"actuatorSpeed", NULL);
#ifdef DEBUG_DC
			cout << "Name: " << sections[i].name << " | RobotSpeed: " << sections[i].robotSpeed << " | ActuatorSpeed: "
			<< sections[i].actuatorSpeed << endl;
//			sections[i].robotSpeed += 100;
//			sections[i].actuatorSpeed += 100;
#endif
			i++;
		}

	}

	/**
	 * fills class variable sections with default parameters
	 */
	void CalibrationDribbleForward::createSections()
	{
		double speedFactor = maxSpeed / (sectionSize - 1);
		double rotationFactor = maxRotation / (sectionSize - 1);

		for (int i = 0; i < sectionSize; i++)
		{
			sections[i].name = "P" + std::to_string(i + 1);
			sections[i].robotSpeed = maxSpeed - i * speedFactor;
			// don't fall under the minRotation
			sections[i].actuatorSpeed =
					(maxRotation - i * rotationFactor) < minRotation ? minRotation : maxRotation - i * rotationFactor;
		}
	}

	void CalibrationDribbleForward::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		DribbleCalibrationContainer dcc;

		// own config parameters
		changingValue = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.ChangingValue",
		NULL);
		minRotationNumber = (*sc)["DribbleCalibration"]->get<double>(
				"DribbleCalibration.DribbleForward.MinRotationNumber", NULL);
		maxSpeed = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.MaxSpeed", NULL);
		maxRotation = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.MaxRotation", NULL);
		sectionSize = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.SectionSize", NULL);
		haveBallWaitingDuration = (*sc)["DribbleCalibration"]->get<double>(
				"DribbleCalibration.DribbleForward.HaveBallWaitingDuration", NULL);
		minCalibrationSpeed = (*sc)["DribbleCalibration"]->get<double>(
				"DribbleCalibration.DribbleForward.minCalibrationSpeed", NULL);
		collectDataWaitingDuration = (*sc)["DribbleCalibration"]->get<double>(
				"DribbleCalibration.DribbleForward.CollectDataWaitingDuration", NULL);
		startTrans = (*sc)["DribbleCalibration"]->get<int>("DribbleCalibration.DribbleForward.StartTranslation", NULL);

		// actuation config Params
		minRotation = dcc.readConfigParameter("Dribble.MinRotation");

		// sections
		vector<subsection> vec(sectionSize);
		sections = vec;
		shared_ptr<vector<string> > speedsSections = (*sc)["Actuation"]->getSections("ForwardDribbleSpeeds", NULL);

		if (speedsSections->size() == sectionSize)
		{
			fillSections(speedsSections);
		}
		else
		{
			// currently not used
			createSections();
		}
	}

	void CalibrationDribbleForward::writeConfigParameters()
	{
		dcc.writeConfigParameters(sections, "ForwardDribbleSpeeds");
	}
/*PROTECTED REGION END*/
} /* namespace alica */
