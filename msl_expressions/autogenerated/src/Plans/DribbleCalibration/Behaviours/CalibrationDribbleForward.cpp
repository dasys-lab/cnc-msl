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
//		writeConfigParameters();
//        return;
        if (wm->rawSensorData->getLightBarrier())
        {
	    // TODO: set in 60 in config file -> waiting duration for getting the ball
	    if (haveBallCount == 0 || (getBallCount > 0 && getBallCount < 60))
	    {
		haveBallCount++; 
		getBallCount++;
		return;
	    } else 
	    {
	    	getBallCount = 0;
	    }
            haveBallCount++;
            // if ball is in kicker
            // drive forward start slowly
            // start with 400 speed
            if (moveCount < sectionSize)
            {
//		cout << "moveCount < sectionSize" << endl;
                int translation = 400;
                mc = dcc.move(dcc.DribbleForward, (moveCount + 1) * translation);
		send(mc);
                // use optical flow and light barrier data to analyze the the ball movement
                Rotation ballMovement = checkBallRotation();

                // adapt actuatorSpeed at specific robotSpeed
                if (ballMovement == Correct)
                {
                    correctRotationCount++;
                }
                else if (ballMovement == TooSlow)
                {
                    adaptWheelSpeed (TooSlow);
                    correctRotationCount = 0;
                }

                // if robot can handle ball at this speed -> increase the speed and repeat
                if (correctRotationCount > minRotationNumber)
                {
                    moveCount++;
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
            // if we lose the ball -> our actuator are rotating to fast
            if (haveBallCount > 0)
            {
                adaptWheelSpeed (TooFast);
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
	
	getBallCount;
        moveCount = 0;
        correctRotationCount = 0;
        haveBallCount = 0;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1469116853584) ENABLED START*/ //Add additional methods here
    void CalibrationDribbleForward::adaptWheelSpeed(Rotation err)
    {
        if (err != TooFast && err != TooSlow)
        {
            cout << "CalibrationDribbleForward::correctWheelSpeed -> wrong input!" << endl;
            return;
        }

	int counter = sectionSize - (moveCount + 1);
        if (err == TooFast)
            sections[counter].actuatorSpeed = sections[counter].actuatorSpeed + (changingFactor * 10);

        if (err == TooSlow)
            sections[counter].actuatorSpeed = sections[counter].actuatorSpeed - changingFactor;
	cout << "sections[" << counter << "].actuatorSpeed = " << sections[counter].actuatorSpeed << endl;
	writeConfigParameters();
    }

    CalibrationDribbleForward::Rotation CalibrationDribbleForward::checkBallRotation()
    {
        // read optical flow value

        if (wm->rawSensorData->getOpticalFlow(0) == nullptr)
        {
            cout << "couldn't read optical flow value!" << endl;
            return RotateErr;
        }
        shared_ptr < geometry::CNPoint2D > opticalFlowValues = wm->rawSensorData->getOpticalFlow(0);

	cout << "opticalFlowValue x: " << opticalFlowValues->x;

        // too slow or not moving
        if (opticalFlowValues->x == 0)
        {
//        	opQueue->clear();
	    cout << " -> Too Slow" << endl;
            return TooSlow;
        }

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
        changingFactor = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.ChangingFactor",
                                                                  NULL);
        minRotationNumber = (*sc)["DribbleCalibration"]->get<double>(
                "DribbleCalibration.DribbleForward.MinRotationNumber", NULL);
        maxSpeed = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.MaxSpeed", NULL);
        maxRotation = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.MaxRotation", NULL);
        sectionSize = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.SectionSize", NULL);

        // actuation config Params
        minRotation = dcc.readConfigParameter("Dribble.MinRotation");

        // sections 
        vector < subsection > vec(sectionSize);
        sections = vec;
        shared_ptr < vector<string> > speedsSections = (*sc)["Actuation"]->getSections("ForwardDribbleSpeeds", NULL);

        if (speedsSections->size() == sectionSize)
        {
            fillSections (speedsSections);
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
