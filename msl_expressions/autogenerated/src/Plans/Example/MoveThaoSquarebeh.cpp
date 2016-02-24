using namespace std;
#include "Plans/Example/MoveThaoSquarebeh.h"

/*PROTECTED REGION ID(inccpp1450269578569) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1450269578569) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    MoveThaoSquarebeh::MoveThaoSquarebeh() :
            DomainBehaviour("MoveThaoSquarebeh")
    {
        /*PROTECTED REGION ID(con1450269578569) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    MoveThaoSquarebeh::~MoveThaoSquarebeh()
    {
        /*PROTECTED REGION ID(dcon1450269578569) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void MoveThaoSquarebeh::run(void* msg)
    {
        /*PROTECTED REGION ID(run1450269578569) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos2 = wm->rawSensorData.getOwnPositionVision();

        if (ownPos2 == nullptr)
        {
            return;
        }
        geometry::CNPoint2D alloTarget;

        alloTarget.x = 4000;

        alloTarget.y = 4000;
        //auto egoTarget = alloTarget.alloToEgo(*ownPos);

        msl_actuator_msgs::MotionControl mc;

        mc.motion.angle = (M_PI / 4);
        ;

        //((((alloTarget.y) - (ownPos->y)) / ((alloTarget.x) - (ownPos->x)) * 3.14159) / 180);
        mc.motion.rotation = 0;
        mc.motion.translation = 1000;

        send(mc);

        //    if (egoTarget->length() < 250)
        //     {
        //        count++;
        //     }

        //    send(mc);
        /*PROTECTED REGION END*/
    }
    void MoveThaoSquarebeh::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450269578569) ENABLED START*/ //Add additional options here
        count = 0;
        ownPos = wm->rawSensorData.getOwnPositionVision();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1450269578569) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
