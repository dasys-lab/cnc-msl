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
        query = nullptr;
        startTrans = 0;
        endTrans = 0;
        speedIter = 0;
        moveCount = 0;
        getBallCount = 0;
        getBallFlag = true;
        haveBallCount = 0;
        haveBallWaitingDuration = 0;
//		correctRotationCount = 0;
        collectDataWaitingDuration = 0;
        orthoDriveFactor = 0;
        minHaveBallIter = 0;
        minHaveBallParamPoint = 0;
        maxHaveBallParamPoint = 0;
        changingValue = 0;

        // output
        moveLeftFlag = true;
        moveRightFlag = true;

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
        // check DribbleControl code and maybe add a new parameter for the orthogonal calculation
        MotionControl mc;
        RobotMovement rm;

        // if ball is in kicker
//		if (wm->rawSensorData->getLightBarrier(0) && (moveCount < speedIter))
        if ((moveCount < speedIter))
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

            // drive to the left
#ifdef DEBUG_DC
            cout << "haveBallCount = " << haveBallCount << " minHaveBallIter = " << minHaveBallIter << endl;
#endif

            if (haveBallCount < (minHaveBallIter / 2))
            {
            	// output
                if (moveLeftFlag) {cout << "move left..." << endl; moveLeftFlag = false;}

                mc = dcc.move(dcc.Left, (moveCount + 1) * startTrans);
            }
            else
            {
            	// output
            	if (moveRightFlag) {cout << "move right..." << endl; moveRightFlag = false;}

                mc = dcc.move(dcc.Right, (moveCount + 1) * startTrans);
            }

            // checking for error values
            if (mc.motion.translation == NAN)
            {
                cerr << "motion command == NAN" << endl;
            }
            else
            {
                send(mc);
            }

            // waiting some time till we can be sure to only collect correct values
            if (haveBallCount < (haveBallWaitingDuration + collectDataWaitingDuration))
            {
                return;
            }

            // if the robot could hold the ball for a long time -> adapt othoDriveFactor and remember the Point
            if (haveBallCount >= minHaveBallIter)
            {
                moveCount++;
                cout << "moveCount = " << moveCount << endl;

                haveBallCount = 0;

                if (minHaveBallParamPoint == 0)
                {
                    minHaveBallParamPoint = orthoDriveFactor;
                }
                else
                {
                    maxHaveBallParamPoint = orthoDriveFactor;
                }

                adaptParam();
            }

            // check sensor data -> optical flow should say, that the ball isn't moving
            // correct data if the robot is loosing the ball
            // drive to the right
            // check sensor data -> optical flow should say, that the ball isn't moving
            // correct data if the robot is loosing the ball

        }
        else if (moveCount > speedIter)
        {
            // choose correct value
            orthoDriveFactor = (minHaveBallParamPoint + maxHaveBallParamPoint) / 2;
            writeConfigParameters();
            cout << "the best config value for orthogonal driving is " << orthoDriveFactor << endl;
            this->setSuccess(true);
            return;
        }
        else
        {
            if (getBallFlag)
            {
                cout << "getting ball" << endl;
            }

            // we need to adapt our weel Speed!
            if (haveBallCount > 0)
            {
//				adaptWheelSpeed (TooFast);
//				correctRotationCount = 0;
                adaptParam();
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
    void CalibrationDribbleOrthogonal::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1469196321064) ENABLED START*/ //Add additional options here
        cout << "starting dribble orthogonal calibration..." << endl;
        query = make_shared<MovementQuery>();
        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1469196321064) ENABLED START*/ //Add additional methods here
    void CalibrationDribbleOrthogonal::adaptParam()
    {
        orthoDriveFactor = orthoDriveFactor + changingValue;
        writeConfigParameters();
    }

    CalibrationDribbleOrthogonal::Rotation CalibrationDribbleOrthogonal::checkBallRotation()
    {
        return RotateErr;
    }

    /**
     * writes to etc/<Robot>/Actuation.conf -> OrthoDriveFactor
     */
    void CalibrationDribbleOrthogonal::writeConfigParameters()
    {
        supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
        cout << "writing OrthoDriveFractor value: " << orthoDriveFactor << endl;
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (orthoDriveFactor), "Dribble.OrthoDriveFactor",
                                 NULL);
    }

    void CalibrationDribbleOrthogonal::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

//		orthoDriveFactor = dcc.readConfigParameter("Dribble.OrthoDriveFactor");
        startTrans = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleOrthogonal.StartTranslation",
                                                              NULL);
        endTrans = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleOrthogonal.EndTranslation",
                                                            NULL);
        speedIter = floor(endTrans / startTrans);
        haveBallWaitingDuration = (*sc)["DribbleCalibration"]->get<double>(
                "DribbleCalibration.Default.HaveBallWaitingDuration", NULL);
        collectDataWaitingDuration = (*sc)["DribbleCalibration"]->get<int>(
                "DribbleCalibration.DribbleOrthogonal.EndTranslation", NULL);
        minHaveBallIter = (*sc)["DribbleCalibration"]->get<int>("DribbleCalibration.DribbleOrthogonal.MinHaveBallIter",
                                                                NULL);
        changingValue = (*sc)["DribbleCalibration"]->get<int>("DribbleCalibration.DribbleOrthogonal.ChangingValue",
                                                              NULL);

    }
/*PROTECTED REGION END*/
} /* namespace alica */
