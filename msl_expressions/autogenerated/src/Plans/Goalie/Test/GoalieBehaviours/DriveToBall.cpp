using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/DriveToBall.h"

/*PROTECTED REGION ID(inccpp1447863493623) ENABLED START*/ //Add additional includes here
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
        /*cout << "inside drive to ball" << endl;
         auto egoTarget = make_shared<geometry::CNPoint2D>(targetX, targetY);
         mc = RobotMovement::moveToPointFast(egoTarget, goalMid, 100, 0);
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
