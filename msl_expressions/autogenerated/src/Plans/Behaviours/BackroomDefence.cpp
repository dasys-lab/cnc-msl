#include "Plans/Behaviours/BackroomDefence.h"

/*PROTECTED REGION ID(inccpp1454507752863) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1454507752863) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    BackroomDefence::BackroomDefence() :
            DomainBehaviour("BackroomDefence")
    {
        /*PROTECTED REGION ID(con1454507752863) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    BackroomDefence::~BackroomDefence()
    {
        /*PROTECTED REGION ID(dcon1454507752863) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void BackroomDefence::run(void* msg)
    {
        /*PROTECTED REGION ID(run1454507752863) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto alloBallPos = wm->ball->getPositionAllo();

        if (!ownPos || !alloBallPos)
        {
            return;
        }

        // assume goalie is in the middle of the goal
        auto ownGoalMid = wm->field->posOwnGoalMid();

        auto goaltoball = *alloBallPos - ownGoalMid;
        auto defenderPos = ownGoalMid + (goaltoball.normalize()) * min(4300.0, goaltoball.length() - 1750.0);

        defenderPos = wm->field->mapOutOfOwnPenalty(defenderPos, goaltoball);

        query->egoDestinationPoint = defenderPos->alloToEgo(*ownPos);
        query->egoAlignPoint = alloBallPos->alloToEgo(*ownPos);
        query->snapDistance = 1000;
        query->velocityMode = msl::MovementQuery::Velocity::FAST;
        msl_actuator_msgs::MotionControl mc = rm.moveToPoint(query);

        send(mc);
        /*PROTECTED REGION END*/
    }
    void BackroomDefence::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1454507752863) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1454507752863) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
