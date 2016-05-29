using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/DriveToGoal.h"

/*PROTECTED REGION ID(inccpp1447863424939) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <MSLWorldModel.h>
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
        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        if (ownPos == nullptr)
        {
        	return;
        }

		auto alloTarget = make_shared<geometry::CNPoint2D>(wm->field->posOwnGoalMid()->x - 100,  wm->field->posOwnGoalMid()->y);
		if (ownPos->distanceTo(alloTarget) <= 100)
		{
			this->setSuccess(true);
			return;
		}

		auto mc = msl::RobotMovement::moveToPointCarefully(alloTarget->alloToEgo(*ownPos),
													  this->wm->field->posCenterMarker()->alloToEgo(*ownPos),
													  100, 0);
		send (mc);
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
