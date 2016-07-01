#ifndef RobotTest_H_
#define RobotTest_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1456756113767) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
/*PROTECTED REGION END*/
namespace alica
{
    class RobotTest : public DomainBehaviour
    {
    public:
        RobotTest();
        virtual ~RobotTest();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1456756113767) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1456756113767) ENABLED START*/ //Add additional protected methods here
        int move;

        bool driveForward;
        bool driveBack;
        bool rotateForward;
        bool rotateBack;
        bool kicker;
        // forward = push
        // back = pull
        bool actuatorPushRight;
        bool actuatorPushLeft;
        bool actuatorPullRight;
        bool actuatorPullLeft;
        bool lightBarrier;
        bool opticalFlow;
        bool imu;
        bool shovelSelectLow;
        bool shovelSelectHigh;
        double kickPower;
        bool startAll;

        bool finDriveForward;
        bool finDriveBack;
        bool finRotateForward;
        bool finRotateBack;
        bool finActuatorPushLeft;
        bool finActuatorPushRight;
        bool finActuatorPullLeft;
        bool finActuatorPullRight;
        bool finKicker;
        bool finLightBarrier;
        bool finOpticalFlow;
        bool finImu;
        bool finShovelSelectLow;
        bool finShovelSelectHigh;

        int rotationTime;
        bool repeat;
        bool out;

        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1456756113767) ENABLED START*/ //Add additional private methods here
        bool translationRotationRobot(int movement, bool trans, int duration);
        bool kickerRobot(int power);
        bool actuatorRobot(int duration, int power, bool right);
        bool lightBarrierRobot();
        bool opticalFlowRobot();
        bool imuRobot();
        bool shovelSelectRobot(bool pass, int duration);
        void printGlasses();
//		bool readConfig(string pram);
        void readConfigParms();
        void setParms(bool b);
        void setFinParmsFalse();
        bool outPut(string s, bool t);
        bool finished();
        void controllOutput();
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* RobotTest_H_ */
