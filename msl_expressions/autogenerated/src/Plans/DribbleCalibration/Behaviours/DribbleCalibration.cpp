using namespace std;
#include "Plans/DribbleCalibration/Behaviours/DribbleCalibration.h"

/*PROTECTED REGION ID(inccpp1482339434271) ENABLED START*/ //Add additional includes here
#include <RawSensorData.h>
#include <Ball.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <SystemConfig.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1482339434271) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleCalibration::DribbleCalibration() :
            DomainBehaviour("DribbleCalibration")
    {
        /*PROTECTED REGION ID(con1482339434271) ENABLED START*/ //Add additional options here
        dribbleForward = false;
        dribbleBackward = false;
        param = DribbleCalibrationContainer::Param::ErrParm;
        startTrans = 0;
        endTrans = 0;
        speedIter = 0;
        moveCount = 0;
        getBallCount = 0;
        getBallFlag = true;
        haveBallCount = 0;
        haveBallWaitingDuration = 0;
        collectDataWaitingDuration = 0;
        minHaveBallIter = 0;
        minHaveBallParamPoint = 0;
        maxHaveBallParamPoint = 0;
        /*PROTECTED REGION END*/
    }
    DribbleCalibration::~DribbleCalibration()
    {
        /*PROTECTED REGION ID(dcon1482339434271) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleCalibration::run(void* msg)
    {
        /*PROTECTED REGION ID(run1482339434271) ENABLED START*/ //Add additional options here
        // check DribbleControl code and maybe add a new parameter for the orthogonal calculation
        MotionControl mc;
        RobotMovement rm;
//		MovementContainer moveCont;

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
            cout << "DribbleCalibration::run() haveBallCount = " << haveBallCount << " minHaveBallIter = " << minHaveBallIter << endl;
#endif

            // translation may not be higher than endTrans
            int tran = ((moveCount + 1) * startTrans) < endTrans ? ((moveCount + 1) * startTrans) : endTrans;

            mc = dcc.paramToMove(param, tran);

            // checking for error values
            if (mc.motion.translation == NAN)
            {
                cerr << "motion command == NAN" << endl;
                return;
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

            // if the robot could hold the ball for a long time -> adapt parameter and remember the Point
            if (haveBallCount >= minHaveBallIter)
            {
                moveCount++;
                cout << "Could hold the ball long enough. Increasing speed to" << (moveCount + 1) * startTrans << "..."
                        << endl;

                haveBallCount = 0;

//				if (minHaveBallParamPoint == 0)
//				{
//					cout << "setting minimum parameter value to " << orthoDriveFactor << "..." << endl;
//					minHaveBallParamPoint = orthoDriveFactor;
//				}
//				else
//				{
//					cout << "setting maximum parameter value to " << orthoDriveFactor << "..." << endl;
//					maxHaveBallParamPoint = orthoDriveFactor;
//				}
//
//				adaptParam();
            }

        }
        else if (moveCount >= speedIter)
        {
            // end
            // choose correct value
        	dcc.writeConfigParameres(param);
//			cout << "Collected enough data. Depending on this the best configuration value for orthogonal driving is "
//					<< orthoDriveFactor << endl;
            cout << "finished calibration" << endl;
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
//				adaptParam();
            	dcc.adaptParam(param);
            }

            haveBallCount = 0;
            mc = moveCont.getBall();
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
    void DribbleCalibration::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1482339434271) ENABLED START*/ //Add additional options here
//        MovementContainer moveCont;
        readConfigParameters();

        bool success = true;
        string tmp;
        try
        {
            // dribble forward
            success &= getParameter("DribbleForward", tmp);
            if (success)
            {
                std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
                istringstream(tmp) >> std::boolalpha >> dribbleForward;
                !dribbleForward ? : param = DribbleCalibrationContainer::Param::DribbleForwardParm;
            }

            // dribble backward
            success &= getParameter("DribbleBackward", tmp);
            if (success)
            {
                std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
                istringstream(tmp) >> std::boolalpha >> dribbleBackward;
                !dribbleBackward ? : param = DribbleCalibrationContainer::Param::DribbleBackwardParm;
            }
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "DC: Parameter does not exist" << endl;
        }

        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1482339434271) ENABLED START*/ //Add additional methods here
    void DribbleCalibration::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

        startTrans = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.StartTranslation", NULL);
        endTrans = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.EndTranslation", NULL);
        speedIter = floor(endTrans / startTrans);

        haveBallWaitingDuration = (*sc)["DribbleCalibration"]->get<double>(
                "DribbleCalibration.Default.HaveBallWaitingDuration", NULL);

        collectDataWaitingDuration = (*sc)["DribbleCalibration"]->get<int>("DribbleCalibration.Default.EndTranslation",
                                                                           NULL);
        minHaveBallIter = (*sc)["DribbleCalibration"]->get<int>("DribbleCalibration.Default.MinHaveBallIter", NULL);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
