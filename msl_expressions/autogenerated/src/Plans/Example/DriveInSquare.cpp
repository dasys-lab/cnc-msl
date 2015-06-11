using namespace std;
#include "Plans/Example/DriveInSquare.h"

/*PROTECTED REGION ID(inccpp1433939613017) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1433939613017) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DriveInSquare::DriveInSquare() :
            DomainBehaviour("DriveInSquare")
    {
        /*PROTECTED REGION ID(con1433939613017) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    DriveInSquare::~DriveInSquare()
    {
        /*PROTECTED REGION ID(dcon1433939613017) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DriveInSquare::run(void* msg)
    {
        /*PROTECTED REGION ID(run1433939613017) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision();

        if (ownPos == nullptr)
        {
            return;
        }
        geometry::CNPoint2D alloTarget;

        if (count % 4 == 0)
        {
            alloTarget.x = 0;
            alloTarget.y = 0;
        }
        else if (count % 4 == 1)
        {
            alloTarget.x = 2000;
            alloTarget.y = 0;
        }
        else if (count % 4 == 2)
        {
            alloTarget.x = 2000;
            alloTarget.y = 2000;
        }
        else
        {
            alloTarget.x = 0;
            alloTarget.y = 2000;
        }

        auto egoTarget = alloTarget.alloToEgo(*ownPos);

        MotionControl mc;
        mc = RobotMovement::moveToPointCarefully(egoTarget, make_shared < geometry::CNPoint2D > (-1000.0, 0.0), 0);

        if (egoTarget->length() < 250)
        {
            count++;
        }

        send(mc);

        /*PROTECTED REGION END*/
    }
    void DriveInSquare::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1433939613017) ENABLED START*/ //Add additional options here
        count = 0;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1433939613017) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
