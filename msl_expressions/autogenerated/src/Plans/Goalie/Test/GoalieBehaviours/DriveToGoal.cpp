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
        shared_ptr < geometry::CNPosition > me = wm->rawSensorData.getOwnPositionVision();

        msl_actuator_msgs::MotionControl mc;
        if (me == nullptr)
        {
            mc.motion.angle = 0;
            mc.motion.rotation = 0;
            mc.motion.translation = 0;

            cout << " Stop!" << endl;
            cout << "### DriveToGoal ###\n" << endl;
        }
        else
        {

            double egoX = MSLFootballField::posOwnGoalMid()->alloToEgo(*me)->x;
            double egoY = MSLFootballField::posOwnGoalMid()->alloToEgo(*me)->y;
            shared_ptr < geometry::CNPoint2D > fieldCenterTarget = MSLFootballField::posCenterMarker()->alloToEgo(*me);

            cout << " Driving to goal" << endl;
            //mc = RobotMovement::moveToPointFast(make_shared < geometry::CNPoint2D > (egoX - 100, egoY), fieldCenterTarget,
            //                                  100, 0);
            mc = RobotMovement::moveToPointCarefully(make_shared < geometry::CNPoint2D > (egoX - 100, egoY),
                                                     fieldCenterTarget, 100, 0);
            cout << "### DriveToGoal ###\n" << endl;
        }
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
