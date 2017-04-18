using namespace std;
#include "Plans/Behaviours/SpinSlowly.h"

/*PROTECTED REGION ID(inccpp1435159253296) ENABLED START*/ //Add additional includes here
#include <RawSensorData.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1435159253296) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    SpinSlowly::SpinSlowly() :
            DomainBehaviour("SpinSlowly")
    {
        /*PROTECTED REGION ID(con1435159253296) ENABLED START*/ //Add additional options here
        center = geometry::CNPointAllo(0, 0);
        /*PROTECTED REGION END*/
    }
    SpinSlowly::~SpinSlowly()
    {
        /*PROTECTED REGION ID(dcon1435159253296) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void SpinSlowly::run(void* msg)
    {
        /*PROTECTED REGION ID(run1435159253296) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData->getOwnPositionMotionBuffer().getLastValidContent();

        if (!ownPos)
        {
            return;
        }

        if (startAngle == 999)
        {
            startAngle = ownPos->theta;
        }

        alpha = ownPos->theta;

        msl_actuator_msgs::MotionControl mc;

        cout << "angle, epsilon: " << abs(startAngle - alpha) << ", " << epsilon << endl;
        cout << "counter: " << counter << endl;

        if (abs(startAngle - alpha) < epsilon && counter > 90)
        {
            this->setSuccess(true);
        }
        else
        {
            counter++;
            mc.motion.rotation = M_PI / 8;
        }

        send(mc);

        /*PROTECTED REGION END*/
    }
    void SpinSlowly::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1435159253296) ENABLED START*/ //Add additional options here
        alpha = 0;
        startAngle = 999;
        counter = 0;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1435159253296) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
