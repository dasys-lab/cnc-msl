using namespace std;
#include "Plans/Robotcheck/RobotTest.h"

/*PROTECTED REGION ID(inccpp1456756113767) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <msl_actuator_msgs/KickControl.h>
#include <msl_actuator_msgs/ShovelSelectCmd.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1456756113767) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    RobotTest::RobotTest() :
            DomainBehaviour("RobotTest")
    {
        /*PROTECTED REGION ID(con1456756113767) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    RobotTest::~RobotTest()
    {
        /*PROTECTED REGION ID(dcon1456756113767) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void RobotTest::run(void* msg)
    {
        /*PROTECTED REGION ID(run1456756113767) ENABLED START*/ //Add additional options here
        auto me = wm->rawSensorData->getOwnPositionVision();

        // testing motion ================================================================

        if (driveForward && !finDriveForward)
        {
            out = outPut("driveForward", out);
            setParms(false);
            driveForward = true;
            driveForward = translationRotationRobot(-300, true, 3000);
            if (!driveForward)
            {
                readConfigParms();
                finDriveForward = true;
                out = true;
            }
        }
        if (driveBack && !finDriveBack)
        {
            out = outPut("driveBack", out);
            setParms(false);
            driveBack = true;
            driveBack = translationRotationRobot(300, true, 3000);
            if (!driveBack)
            {
                out = true;
                readConfigParms();
                finDriveBack = true;
            }
        }
        if (rotateForward && !finRotateForward)
        {
            out = outPut("rotateForward", out);
            setParms(false);
            rotateForward = true;
            rotateForward = translationRotationRobot(-2, false, rotationTime);
            if (!rotateForward)
            {
                out = true;
                readConfigParms();
                finRotateForward = true;
            }
        }
        if (rotateBack && !finRotateBack)
        {
            out = outPut("rotateBack", out);
            setParms(false);
            rotateBack = true;
            rotateBack = translationRotationRobot(2, false, rotationTime);
            if (!rotateBack)
            {
                out = true;
                readConfigParms();
                finRotateBack = true;
            }
        }

        // testing actuator ==============================================================

        if (actuatorPushLeft && !finActuatorPushLeft)
        {
            out = outPut("actuator -> pushing left", out);
            setParms(false);
            actuatorPushLeft = true;
            actuatorPushLeft = actuatorRobot(3000, 3000, false);
            if (!actuatorPushLeft)
            {
                out = true;
                readConfigParms();
                finActuatorPushLeft = true;
            }
        }

        if (actuatorPushRight && !finActuatorPushRight)
        {
            out = outPut("actuator -> pushing right", out);
            setParms(false);
            actuatorPushRight = true;
            actuatorPushRight = actuatorRobot(3000, 3000, true);
            if (!actuatorPushRight)
            {
                out = true;
                readConfigParms();
                finActuatorPushRight = true;
            }
        }

        if (actuatorPullLeft && !finActuatorPullLeft)
        {
            out = outPut("actuator -> pulling left", out);
            setParms(false);
            actuatorPullLeft = true;
            actuatorPullLeft = actuatorRobot(3000, -3000, false);
            if (!actuatorPullLeft)
            {
                out = true;
                readConfigParms();
                finActuatorPullLeft = true;
            }
        }

        if (actuatorPullRight && !finActuatorPullRight)
        {
            out = outPut("actuator -> pulling right", out);
            setParms(false);
            actuatorPullRight = true;
            actuatorPullRight = actuatorRobot(3000, -3000, true);
            if (!actuatorPullRight)
            {
                out = true;
                readConfigParms();
                finActuatorPullRight = true;
            }
        }

        // light barrier ==============================================================

        if (lightBarrier && !finLightBarrier)
        {
            out = outPut("lightBarrier", out);
            setParms(false);
            lightBarrier = true;
            lightBarrier = lightBarrierRobot();
            if (!lightBarrier)
            {
                out = true;
                readConfigParms();
                finLightBarrier = true;
            }
        }

        // optical flow ===============================================================

        if (opticalFlow && !finOpticalFlow)
        {
            out = outPut("opticalFlow", out);
            setParms(false);
            opticalFlow = true;
            opticalFlow = opticalFlowRobot();
            if (!opticalFlow)
            {
                out = true;
                readConfigParms();
                finOpticalFlow = true;
            }
        }

        // IMU ========================================================================

        if (imu && !finImu)
        {
            out = outPut("imu", out);
            setParms(false);
            imu = true;
            imu = imuRobot();
            if (!imu)
            {
                out = true;
                readConfigParms();
                finImu = true;
            }
        }

        // shovel select ==============================================================

        if (shovelSelectLow && !finShovelSelectLow)
        {
            out = outPut("shovelSelectLow", out);
            setParms(false);
            shovelSelectLow = true;
            shovelSelectLow = shovelSelectRobot(true, 3000);
            if (!shovelSelectLow)
            {
                out = true;
                readConfigParms();
                finShovelSelectLow = true;
            }
        }

        if (shovelSelectHigh && !finShovelSelectHigh)
        {
            out = outPut("shovelSelectHigh", out);
            setParms(false);
            shovelSelectHigh = true;
            shovelSelectHigh = shovelSelectRobot(false, 3000);
            if (!shovelSelectHigh)
            {
                out = true;
                readConfigParms();
                finShovelSelectHigh = true;
            }
        }

        // testing kicker ================================================================

        if (kicker && !finKicker)
        {
            out = outPut("kicker", out);
            kicker = kickerRobot(kickPower);
            if (!kicker)
            {
                readConfigParms();
                finKicker = true;
                out = true;
            }
        }

        if (finished())
        {
            if (repeat)
            {
                cout << "restart test..." << endl;
                initialiseParameters();
            }
            else
            {
                cout << "finished testing" << endl;
                this->setSuccess(true);
            }

        }
        /*PROTECTED REGION END*/
    }
    void RobotTest::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1456756113767) ENABLED START*/ //Add additional options here
        move = 0;
        out = true;
        setFinParmsFalse();
        readConfigParms();

        cout << "\nstart testing ..." << endl;
        if (!startAll)
        {

            cout << "test is manually configured!\n"
                    "starting the first test!" << endl;

        }

        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1456756113767) ENABLED START*/ //Add additional methods here
    bool RobotTest::translationRotationRobot(int movement, bool trans, int duration)
    {
        msl_actuator_msgs::MotionControl mc;
        if (trans)
        {
            if (move < (30 * duration) / 1000 && trans)
            {
                mc.motion.translation = movement;
            }
            else
            {
                move = 0;
                return false;
            }
        }
        else
        {
            if (move < (30 * duration) / 1000 && !trans)
            {
                mc.motion.rotation = movement;
            }
            else
            {
                move = 0;
                return false;
            }
        }
        move++;
        send(mc);

        return true;
    }

    bool RobotTest::kickerRobot(int power)
    {
        msl_actuator_msgs::KickControl kc;
        kc.enabled = true;
        if (power > 0)
        {
            kc.power = power;
            send(kc);
            return false;

        }
        else
        {
            cerr << "Kick power may not be negative!" << endl;
            kc.power = 0;
        }
        return true;
    }

    bool RobotTest::actuatorRobot(int duration, int power, bool right)
    {
        msl_actuator_msgs::BallHandleCmd bhc;
        if (move < (30 * duration) / 1000)
        {
            if (power < 10000 && power > -10000)
            {
                if (right)
                {
                    bhc.leftMotor = power;
                }
                else
                {
                    bhc.rightMotor = power;
                }
            }
            else
            {
                bhc.leftMotor = 0;
                bhc.rightMotor = 0;
                cerr << "Rotation speed may only be in range of -100 to 100!" << endl;
            }
        }
        else
        {
            move = 0;
            return false;
        }

        move++;
        bhc.enabled = true;
        send(bhc);
        return true;
    }

    bool RobotTest::lightBarrierRobot()
    {
        auto lbi = wm->rawSensorData->getLightBarrier();

        bool static lb_old = lbi;
        if (lb_old != lbi)
        {
            lb_old = lbi;
            cout << "toggle light barrier!" << endl;
            move++;
        }
        if (move > 5)
        {
            move = 0;
            return false;
        }
        return true;
    }

    bool RobotTest::opticalFlowRobot()
    {
        auto of = wm->rawSensorData->getOpticalFlow();
        if (of != nullptr)
        {
            cout << "receive data from optical flow!" << endl;
            return false;
        }
        else
        {
            cerr << "No data from optical flow!" << endl;
            return false;
        }
        return true;
    }

    bool RobotTest::imuRobot()
    {
        cout << "IMU currently not included in world model" << endl;
        return false;
    }

    bool RobotTest::shovelSelectRobot(bool pass, int duration)
    {
        msl_actuator_msgs::ShovelSelectCmd sc;
        if (move < (30 * duration) / 1000)
        {
            sc.passing = pass;
            send(sc);
        }
        else
        {
            move = 0;
            return false;
        }
        move++;
        return true;
    }

//	bool RobotTest::readConfig(string parm)
//	{
//
//		string s = ("Robotcheck.Default." + parm);
//		char* p = s.c_str();
//		cout << "s = " << s << endl;
//		return (*sc)["Robotcheck"]->get<bool>(s, NULL);
//
//	}

    void RobotTest::readConfigParms()
    {
        driveForward = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.driveForward", NULL);
        driveBack = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.driveBack", NULL);
        rotateForward = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.rotateForward", NULL);
        rotateBack = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.rotateBack", NULL);
        kicker = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.kicker", NULL);
        actuatorPushRight = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.actuatorPushRight", NULL);
        actuatorPushLeft = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.actuatorPushLeft", NULL);
        actuatorPullRight = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.actuatorPullRight", NULL);
        actuatorPullLeft = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.actuatorPullLeft", NULL);
        lightBarrier = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.lightBarrier", NULL);
        opticalFlow = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.opticalFlow", NULL);
        imu = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.imu", NULL);
        shovelSelectLow = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.shovelSelectLow", NULL);
        shovelSelectHigh = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Default.shovelSelectHigh", NULL);
        kickPower = (*sc)["Robotcheck"]->get<double>("Robotcheck.Default.kickPower", NULL);
        startAll = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Global.startAll", NULL);
        repeat = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Global.repeat", NULL);
        rotationTime = (*sc)["Robotcheck"]->get<bool>("Robotcheck.Robot.rotationTime", NULL);

        if (startAll)
        {
            setParms(true);
        }
    }

    void RobotTest::setParms(bool b)
    {
        driveForward = b;
        driveBack = b;
        rotateBack = b;
        rotateForward = b;
        kicker = b;
        actuatorPushRight = b;
        actuatorPushLeft = b;
        actuatorPullRight = b;
        actuatorPullLeft = b;
        opticalFlow = b;
        imu = b;
        shovelSelectLow = b;
        shovelSelectHigh = b;
        lightBarrier = b;
    }

    void RobotTest::setFinParmsFalse()
    {
        finDriveForward = false;
        finDriveBack = false;
        finRotateForward = false;
        finRotateBack = false;
        finKicker = false;
        finActuatorPushLeft = false;
        finActuatorPushRight = false;
        finActuatorPullLeft = false;
        finActuatorPullRight = false;
        finLightBarrier = false;
        finOpticalFlow = false;
        finImu = false;
        finShovelSelectLow = false;
        finShovelSelectHigh = false;
    }

    bool RobotTest::outPut(string s, bool t)
    {
        if (t)
        {
            cout << "testing: " << s << endl;
        }
        return false;
    }

    bool RobotTest::finished()
    {
//		controllOutput();
        return (finDriveForward == driveForward && finDriveBack == driveBack && finRotateForward == rotateForward
                && finRotateBack == rotateBack && finKicker == kicker && finActuatorPushLeft == actuatorPushLeft
                && finActuatorPushRight == actuatorPushRight && finActuatorPullLeft == actuatorPullLeft
                && finActuatorPullRight == actuatorPullRight && finLightBarrier == lightBarrier
                && finOpticalFlow == opticalFlow && finImu == imu && finShovelSelectLow == shovelSelectLow
                && finShovelSelectHigh == shovelSelectHigh);
    }

    void RobotTest::controllOutput()
    {
        cout << driveForward << driveBack << rotateForward << rotateBack << actuatorPushLeft << actuatorPushRight
                << actuatorPullLeft << actuatorPullRight << kicker << lightBarrier << opticalFlow << imu
                << shovelSelectLow << shovelSelectHigh << " | " << finDriveForward << finDriveBack << finRotateForward
                << finRotateBack << finActuatorPushLeft << finActuatorPushRight << finActuatorPullLeft
                << finActuatorPullRight << finLightBarrier << finOpticalFlow << finImu << finShovelSelectLow
                << finShovelSelectHigh << finKicker << endl;

    }
/*PROTECTED REGION END*/
} /* namespace alica */
