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
        runBehaviour = false;
        changingValue = 0;
        queueSize = 0;
        oldOperation = Add;
        queueFilled = false;
        operation = Add;
        minRotation = 0;
        adaptWheel = 0;
        ballRotateCorrect = false;
        ballHoldCorrect = false;
        slowTranslationWheelSpeed = 0;
        dribbleFactorRight = 0;
        dribbleFactorLeft = 0;
        speedNoBall = 0;
        opQueue = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
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
        if (!runBehaviour)
        {
            cout << "skipping TakeBall behaviour..." << endl;
            this->setSuccess(true);
            return;
        }
        // check if robot has the ball
        if (wm->rawSensorData->getLightBarrier())
        {
            // check if ball is rotating correctly
            if (!this->ballRotateCorrect)
            {
                // let ball continuously rotate with speedNoBall (should be by 4000)
                if (!dcc.fillOpticalFlowQueue(queueSize, opQueue))
                {
                    return;
                }
                Rotation ballRotation = checkBallRotation();

                if (ballRotation == RotateErr)
                {
                    cout << "ROTATE_ERR" << endl;
                    return;
                }
                else if (ballRotation == RotateCorrect)
                {
                    cout << "ROTATE_CORRECT" << endl;
                    this->ballRotateCorrect = true;
                }
                else if (ballRotation == RotateLeft)
                {
                    // ROTATE_LEFT means that the right wheel is spinning too fast so we need to correct the right wheel
                    correctWheelSpeed (RotateLeft);
                    writeConfigParameters();
                }
                else if (ballRotation == RotateRight)
                {
                    // ROTATE_RIGHT means that the left wheel is spinning too fast so we need to correct the left wheel
                    correctWheelSpeed (RotateRight);
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
        cout << "starting dribble calibration..." << endl;
        changingValue = 0;
        queueSize = 0;
        oldOperation = Add;
        queueFilled = false;
        operation = Add;
        minRotation = 0;
        adaptWheel = 0;
        ballRotateCorrect = false;
        ballHoldCorrect = false;
        slowTranslationWheelSpeed = 0;
        dribbleFactorRight = 0;
        dribbleFactorLeft = 0;
        speedNoBall = 0;
        readConfigParameters();
        cout << "try to synchronize the actuators..." << endl;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1469109429392) ENABLED START*/ //Add additional methods here
    CalibrationTakeBall::Rotation CalibrationTakeBall::checkBallRotation()
    {

        int minX = 125;
        double maxY = 0.99999;

        // average x value should be on max (128)
        // average y value should be at 0 (between 0.99999 and -0.99999)
        // average qos (quality of service) should be existent ... normally between 30 and 40

        double xValue = dcc.getAverageOpticalFlowXValue(opQueue);
        double yValue = dcc.getAverageOpticalFlowYValue(opQueue);
        //		double qosValue = dcc.getAverageOpticalFlowQOSValue(opQueue);

        cout << "checkBallRotation x: " << xValue << endl;
        cout << "checkBallRotation y: " << yValue << endl;

        //		if (qosValue == 0)
        //		{
        //			return ROTATE_ERR;
        //		}

        //		if (xValue > minX && yValue < maxY && yValue > -maxY)
        if (yValue < maxY && yValue > -maxY)
        {
            return RotateCorrect;

        }
        else if (yValue > maxY)
        {
            // right wheel is too fast so the ball is rotating to the left
            return RotateLeft;

        }
        else if (yValue < -maxY)
        {
            // left wheel is too fast
            return RotateRight;
        }

        return RotateErr;
    }

    /**
     * saves the currently changing wheel for further adaption
     * if the ball is rotation right the left wheel need to be slowed
     * if the ball is rotation left the right wheel need to be slowed
     * if the ball is rotation to slow the dribbleFactor of both wheels need to be adapted
     */
    void CalibrationTakeBall::correctWheelSpeed(Rotation rotation)
    {
        if (rotation != RotateRight && rotation != RotateLeft && rotation != RotateTooSlow)
        {
            cout << "CalibrationTakeBall::correctWheelSpeed(int wheel) -> wrong input!" << endl;
            return;
        }
        if (rotation == RotateTooSlow)
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
        cout << "changinFactor: " << changingValue << endl;
        // check if the defect wheel is too fast or to slow
        if (rotation == adaptWheel)
        {
            dribbleFactorLeft = adaptWheel == RotateRight ? dribbleFactorLeft + changingValue : dribbleFactorLeft;
            dribbleFactorRight = adaptWheel == RotateLeft ? dribbleFactorRight + changingValue : dribbleFactorRight;
            operation = Add;
        }
        else
        {
            dribbleFactorLeft = adaptWheel == RotateRight ? dribbleFactorLeft - changingValue : dribbleFactorLeft;
            dribbleFactorRight = adaptWheel == RotateLeft ? dribbleFactorRight - changingValue : dribbleFactorRight;
            operation = Sub;
        }
        if (operation != oldOperation)
        {
            changingValue = changingValue / 2;
            oldOperation = operation == Add ? Sub : Add;
        }
        else
        {
            oldOperation = operation;
        }
        opQueue->clear();
        queueFilled = false;
    }

    void CalibrationTakeBall::readConfigParameters()
    {
        supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
        msl::DribbleCalibrationContainer dcc;
        speedNoBall = (*sys)["Actuation"]->get<double>("Dribble.SpeedNoBall", NULL);
        slowTranslationWheelSpeed = (*sys)["Actuation"]->get<double>("Dribble.SlowTranslationWheelSpeed", NULL);
        minRotation = (*sys)["Actuation"]->get<double>("Dribble.MinRotation", NULL);
        dribbleFactorRight = (*sys)["Actuation"]->get<double>("Dribble.DribbleFactorLeft", NULL); // left or right?
        dribbleFactorLeft = (*sys)["Actuation"]->get<double>("Dribble.DribbleFactorRight", NULL);

        // maybe put in config
        changingValue = (*sys)["DribbleCalibration"]->get<double>("DribbleCalibration.TakeBall.ChangingValue", NULL);
        queueSize = (*sys)["DribbleCalibration"]->get<int>("DribbleCalibration.TakeBall.QueueSize", NULL);

        runBehaviour = (*sys)["DribbleCalibration"]->get<int>("DribbleCalibration.Run.TakeBall", NULL);
    }

    void CalibrationTakeBall::writeConfigParameters()
    {
        cout << "writing config parameters" << endl;
        cout << "speedNoBall:               " << speedNoBall << endl;
        cout << "slowTranslationWheelSpeed: " << slowTranslationWheelSpeed << endl;
        cout << "minRotation:               " << minRotation << endl;
        cout << "dribbleFactorRight:        " << dribbleFactorRight << endl;
        cout << "dribbleFlactorLeft:        " << dribbleFactorLeft << endl;

        supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (speedNoBall), "Dribble.SpeedNoBall", NULL);
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (slowTranslationWheelSpeed),
                                 "Dribble.SlowTranslationWheelSpeed", NULL);
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (minRotation), "Dribble.MinRotation", NULL);
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (dribbleFactorRight), "Dribble.DribbleFactorLeft",
                                 NULL);
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (dribbleFactorLeft), "Dribble.DribbleFactorRight",
                                 NULL);

        (*sys)["Actuation"]->store();
    }

/*PROTECTED REGION END*/
} /* namespace alica */
