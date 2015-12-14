using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/DriveToGoal.h"

/*PROTECTED REGION ID(inccpp1447863424939) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1447863424939) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DriveToGoal::DriveToGoal() :
            DomainBehaviour("DriveToGoal")
    {
        /*PROTECTED REGION ID(con1447863424939) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    DriveToGoal::~DriveToGoal()
    {
        /*PROTECTED REGION ID(dcon1447863424939) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DriveToGoal::run(void* msg)
    {
        /*PROTECTED REGION ID(run1447863424939) ENABLED START*/ //Add additional options here
        auto me = wm->rawSensorData.getOwnPositionMotion();
        if (!me.operator bool())
        {
            return;
        }

        msl_actuator_msgs::MotionControl mc;
        auto egoX = MSLFootballField::posOwnGoalMid()->alloToEgo(*me)->x;
        auto egoY = MSLFootballField::posOwnGoalMid()->alloToEgo(*me)->y;
        shared_ptr < geometry::CNPoint2D > fieldCenterTarget = MSLFootballField::posCenterMarker()->alloToEgo(*me);

        mc = RobotMovement::moveToPointFast(make_shared < geometry::CNPoint2D > (egoX - 100, egoY), fieldCenterTarget,
                                            100, 0);

        send(mc);

        /*PROTECTED REGION END*/
    }
    void DriveToGoal::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1447863424939) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1447863424939) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
