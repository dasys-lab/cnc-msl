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
        if (terminated)
        {
            return;
        }

        if (count == 0)
        {
            cout << "TestMotorControl::run started on first round " <<  endl;
            start = wm->rawSensorData->getOwnPositionMotion();
            if (start == NULL)
                return;
            goal->x = start->x + relGoalX;
            goal->y = start->y + relGoalY;
            goal->theta = start->theta + relGoalRot;
        }

        goalPointer = goal - wm->rawSensorData->getOwnPositionMotion();
        angleDistance = goal->theta - wm->rawSensorData->getOwnPositionMotion()->theta;
        goalDistance = sqrt(
                goalPointer->x * goalPointer->x + goalPointer->y * goalPointer->y
                        + angleDistance * angleDistance * 500);

        cout << "TestMotorControl::run old Distance: " << oldGoalDistance << " new Distance: " << goalDistance << " x "
                << goalPointer->x << " y " << goalPointer->y << " goal " << goal->x << " " << goal->y << endl;

        if (goalDistance > oldGoalDistance + 10)
        {
            if (!terminated)
            {
                cout << "TestMotorControl::run Reached closest point to target at " << count << " with "
                        << goalPointer->x << "," << goalPointer->y << "," << angleDistance << endl;
                msl_actuator_msgs::MotionControl motorMsg;
                motorMsg.motion.angle = 0;
                motorMsg.motion.rotation = 0;
                motorMsg.motion.translation = 0;
                send(motorMsg);
                terminated = true;
            }
            return;
        }
        oldGoalDistance = goalDistance;

        if (count >= abortTime * 30)
        {
            if (!terminated)
            {
                cerr << "TestMotorControl::run Missed target, terminated by time out" << endl;
                msl_actuator_msgs::MotionControl motorMsg;
                motorMsg.motion.angle = 0;
                motorMsg.motion.rotation = 0;
                motorMsg.motion.translation = 0;
                send(motorMsg);
                terminated = true;
            }
            return;
        }

        count++;

        msl_actuator_msgs::MotionControl motorMsg;
        motorMsg.motion.translation = testSpeed;

        if (straight)
        {
            motorMsg.motion.angle = atan2(relGoalY, relGoalX) - wm->rawSensorData->getOwnPositionMotion()->theta
                    + start->theta;
            motorMsg.motion.rotation = relGoalRot * testSpeed / sqrt(relGoalX * relGoalX + relGoalY * relGoalY);
        }

        else
        {
            motorMsg.motion.angle = atan2(relGoalY, relGoalX) - (relGoalRot / 2);
            motorMsg.motion.rotation = testSpeed * 2 * sin(relGoalRot / 2)
                    / sqrt(relGoalX * relGoalX + relGoalY * relGoalY);
        }
	cout<<"TestMotorControl::run motor msg angle: "<<motorMsg.motion.angle<<" rotation: "<<motorMsg.motion.rotation << endl;
        send(motorMsg);

        /*PROTECTED REGION END*/
    }
    void TestMotorControl::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1482163964536) ENABLED START*/ //Add additional options here
        relGoalX = (*sc)["MotorControlTest"]->get<double>("MotorControlTest.GoalX", NULL);
        relGoalY = (*sc)["MotorControlTest"]->get<double>("MotorControlTest.GoalY", NULL);
        relGoalRot = (*sc)["MotorControlTest"]->get<double>("MotorControlTest.GoalRot", NULL) * M_PI;

        while (relGoalRot > M_PI)
        {
            relGoalRot -= 2 * M_PI;
        }
        while (relGoalRot <= -M_PI)
        {
            relGoalRot += 2 * M_PI;
        }

        straight = (*sc)["MotorControlTest"]->get<bool>("MotorControlTest.straight", NULL);
        testSpeed = (*sc)["MotorControlTest"]->get<int>("MotorControlTest.testSpeed", NULL);

        abortTime = (*sc)["MotorControlTest"]->get<double>("MotorControlTest.abortTime", NULL)
                * sqrt(relGoalX * relGoalX + relGoalY * relGoalY) / testSpeed;

        count = 0;
        terminated = false;
        goal = make_shared<geometry::CNPosition>();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1482163964536) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
