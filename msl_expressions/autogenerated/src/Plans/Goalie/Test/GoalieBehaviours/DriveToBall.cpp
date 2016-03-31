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
        simulating = (*this->sc)["Behaviour"]->get<int>("Goalie.Simulating", NULL);
        /*PROTECTED REGION END*/
    }
    void DriveToBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1447863493623) ENABLED START*/ //Add additional options here
    	// TODO: DELETE THIS BECAUSE NOT USED IN GoalieDefault PLAN!!!

        /*cout << "### DriveToBall ###" << endl;
        me = wm->rawSensorData.getOwnPositionVision();
        alloFieldCenter = MSLFootballField::posCenterMarker();
        shared_ptr < geometry::CNPoint2D > alloBall = wm->ball.getAlloBallPosition();

        int goalieHalfSize;
        if (simulating > 0)
            goalieHalfSize = 410; // goalie size in simulator
        else if (simulating < 0)
            goalieHalfSize = 315; // 630mm/2 + 140mm = 445mm

        if (alloBall == nullptr)
        {
            cout << "alloTarget null!" << endl;
            return;
        }
        else if (me == nullptr)
        {
            cout << "me null!" << endl;
            return;
        }

        alloTarget = alloBall;
        shared_ptr < geometry::CNPoint2D > alloAlignPoint = MSLFootballField::posCenterMarker();

        //egoAlignPoint = make_shared<geometry::CNPoint2D>(-me->x, me->y);
        cout << "ownPosition : " << me->toString();
        cout << "alignPoint  : " << alloAlignPoint->egoToAllo(*me)->toString();
        cout << "targetPoint : " << alloTarget->toString();
        cout << "ballPosition: " << alloBall->toString();

        mc = RobotMovement::moveToPointCarefully(alloTarget->alloToEgo(*me), alloAlignPoint->alloToEgo(*me), 100);
        send (mc);

        cout << "### DriveToBall ###\n" << endl;*/
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
