using namespace std;
#include "Plans/Robotcheck/RobotTest.h"

/*PROTECTED REGION ID(inccpp1456756113767) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
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
        auto me = wm->rawSensorData.getOwnPositionVision();

        if (!driveForward)
        {
            driveForward = translationRotationRobot(300, true, 3000);
        }
        if (!driveBack)
        {
            driveBack = translationRotationRobot(-300, true, 3000);
        }
        if (!rotateForward)
        {
            rotateForward = translationRotationRobot(5, false, 3000);
        }
        if (!rotateBack)
        {
            rotateBack = translationRotationRobot(-5, false, 3000);
        }

        /*PROTECTED REGION END*/
    }
    void RobotTest::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1456756113767) ENABLED START*/ //Add additional options here
        move = 0;

        driveForward = false;
        driveBack = false;
        rotateBack = false;
        rotateForward = false;

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
                return true;
            }
        }
        else
        {
            if (move < (30 * duration) / 1000 && trans)
            {
                mc.motion.rotation = movement;
            }
            else
            {
                move = 0;
                return true;
            }
        }
        move++;
        send(mc);

        return false;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
