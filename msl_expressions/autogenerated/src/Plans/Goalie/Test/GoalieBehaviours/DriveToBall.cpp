using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/DriveToBall.h"

/*PROTECTED REGION ID(inccpp1447863493623) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1447863493623) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DriveToBall::DriveToBall() :
            DomainBehaviour("DriveToBall")
    {
        /*PROTECTED REGION ID(con1447863493623) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    DriveToBall::~DriveToBall()
    {
        /*PROTECTED REGION ID(dcon1447863493623) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DriveToBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1447863493623) ENABLED START*/ //Add additional options here
        cout << "### DriveToBall ###" << endl;
        me = wm->rawSensorData.getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoTarget = wm->ball.getEgoBallPosition();
        goalMid = MSLFootballField::posOwnGoalMid();
        mc = RobotMovement::moveToPointFast(egoTarget, egoTarget, 100, 0);

        if (egoTarget == nullptr)
        {
            cout << "egoTarget null!" << endl;
            return;
        }
        else if (me == nullptr)
        {
            cout << "me null!" << endl;
            return;
        }
        double distance = me->distanceTo(egoTarget);
        if (distance < 100)
        {
            this->success = true;
        }
        else
        {
            send (mc);
        }
        cout << "### DriveToBall ###" << endl;
        /*PROTECTED REGION END*/
    }
    void DriveToBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1447863493623) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1447863493623) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
