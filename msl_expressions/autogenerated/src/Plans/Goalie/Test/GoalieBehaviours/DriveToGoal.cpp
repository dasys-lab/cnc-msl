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
        simulating = (*this->sc)["Behaviour"]->get<int>("Goalie.Simulating", NULL);
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
        cout << "### DriveToGoal ###" << endl;
        shared_ptr < geometry::CNPosition > me;
        double alloTargetX, alloTargetY;

        me = wm->rawSensorData.getOwnPositionVision();

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
            /*if (simulating < 0)
             {
             alloTargetX = MSLFootballField::posOwnGoalMid()->x - 100;
             alloTargetY = MSLFootballField::posOwnGoalMid()->y;
             }
             else
             {
             alloTargetX = MSLFootballField::posOppGoalMid()->x + 100;
             alloTargetY = MSLFootballField::posOppGoalMid()->y;

             }*/

            alloTargetX = MSLFootballField::getInstance()->posOwnGoalMid()->x - 100;
            alloTargetY = MSLFootballField::getInstance()->posOwnGoalMid()->y;

            alloTarget = make_shared < geometry::CNPoint2D > (alloTargetX, alloTargetY);
            alloFieldCenterAlignPoint = MSLFootballField::getInstance()->posCenterMarker();

            cout << " Driving to goal" << endl;
            mc = RobotMovement::moveToPointCarefully(alloTarget->alloToEgo(*me),
                                                     alloFieldCenterAlignPoint->alloToEgo(*me), 100, 0);

            if (me->distanceTo(alloTarget) <= 100)
            {
                this->success = true;
            }
            else
            {
                cout << "Distance left: " << me->distanceTo(alloTarget) << endl;
                send (mc);
            }
            cout << "### DriveToGoal ###\n" << endl;
        }
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
