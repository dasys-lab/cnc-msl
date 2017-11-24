using namespace std;
#include "Plans/Calibration/TestRotationRotation.h"

/*PROTECTED REGION ID(inccpp1492620499435) ENABLED START*/ //Add additional includes here
#include <MSLWorldModel.h>
#include <Game.h>
#include <RawSensorData.h>
#include <math.h>
#include <ctime>
#include <SystemConfig.h>
#include <FileSystem.h>
#include "ConsoleCommandHelper.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1492620499435) ENABLED START*/ //initialise static variables here    
    double circularDiff(double a, double b)
    {
        // DEFINITELY SELF DOCUMENTING CODE
        // (maybe not)
        // TODO

        double diff = a - b;
        if (abs(diff) > M_PI)
        {
            diff = 2 * M_PI - diff;
            while (diff > M_PI)
            {
                diff -= 2 * M_PI;
            }
            diff *= -1;
        }

        return diff;
    }

    /*PROTECTED REGION END*/
    TestRotationRotation::TestRotationRotation() :
            DomainBehaviour("TestRotationRotation")
    {
        /*PROTECTED REGION ID(con1492620499435) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    TestRotationRotation::~TestRotationRotation()
    {
        /*PROTECTED REGION ID(dcon1492620499435) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void TestRotationRotation::run(void* msg)
    {
        /*PROTECTED REGION ID(run1492620499435) ENABLED START*/ //Add additional options here
        if (this->isSuccess())
        {
            return;
        }

        int limit = 50;
        if (inited < limit)
        {
            inited++;
            return;
        }
        else if (inited == limit)
        {
            //TODO BUG KAI
//            initialBearing = wm->rawOdometry->position.angle;
            initialBearing = 0.0;
            inited++;
        }

        msl_actuator_msgs::MotionControl mc;
        double rotationSpeed = 0.5;
        mc.motion.rotation = rotationSpeed;
        send(mc);

//        double currentBearing = wm->rawOdometry->position.angle;
        //TODO BUG KAI
        double currentBearing = 0.0;
        double cd = circularDiff(initialBearing, currentBearing);
        if (cd > 3)
        {
            halfwayDone = true;
            cout << "HALFWAY DONE!!!" << endl;
        }

        cout << cd << endl;

        if (halfwayDone && cd < 0)
        {
            this->setSuccess(true);
        }
        /*PROTECTED REGION END*/
    }
    void TestRotationRotation::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1492620499435) ENABLED START*/ //Add additional options here
        inited = 0;
        halfwayDone = false;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1492620499435) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
