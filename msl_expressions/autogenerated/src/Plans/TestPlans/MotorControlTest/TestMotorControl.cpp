using namespace std;
#include "Plans/TestPlans/MotorControlTest/TestMotorControl.h"

/*PROTECTED REGION ID(inccpp1482163964536) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/MotionControl.h"
#include "RawSensorData.h"
#include "MSLWorldModel.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1482163964536) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    TestMotorControl::TestMotorControl() :
            DomainBehaviour("TestMotorControl")
    {
        /*PROTECTED REGION ID(con1482163964536) ENABLED START*/ //Add additional options here
        this->sc = supplementary::SystemConfig::getInstance();
        /*PROTECTED REGION END*/
    }
    TestMotorControl::~TestMotorControl()
    {
        /*PROTECTED REGION ID(dcon1482163964536) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void TestMotorControl::run(void* msg)
    {
        /*PROTECTED REGION ID(run1482163964536) ENABLED START*/ //Add additional options here

    	if (count > 300)
    	{
        msl_actuator_msgs::MotionControl motorMsg;
        motorMsg.motion.translation = testSpeed;
        motorMsg.motion.angle = 0;
        motorMsg.motion.rotation = 0;

        cout << "TestMotorControl::run motor msg angle: " << motorMsg.motion.angle << " rotation: "
                << motorMsg.motion.rotation << endl;
        send(motorMsg);
        count++;
    	}

    	if (count == 300)
    	{

    		(*sc)["MotorControlTest"]->set<string>(to_string(testSpeed+100),"MotorControlTest.testSpeed");
    	}
        /*PROTECTED REGION END*/
    }
    void TestMotorControl::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1482163964536) ENABLED START*/ //Add additional options here

        testSpeed = (*sc)["MotorControlTest"]->get<int>("MotorControlTest.testSpeed", NULL);

        count = 0;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1482163964536) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
