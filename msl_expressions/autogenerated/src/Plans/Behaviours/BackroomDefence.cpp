using namespace std;
#include "Plans/Behaviours/BackroomDefence.h"

/*PROTECTED REGION ID(inccpp1454507752863) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
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
        auto me = wm->rawSensorData.getOwnPositionVision();
        auto alloBallPos = wm->ball.getAlloBallPosition();
        //auto goaliePos = wm->robots.teammates.getTeamMatePosition(1, 0);
        auto field = msl::MSLFootballField::getInstance();
        shared_ptr < geometry::CNPoint2D > goalPos;

        if (!me || !alloBallPos)
        {
            return;
        }

        /*if (goaliePos)
         {
         goalPos = goaliePos->getPoint();
         }
         else
         {*/
        // assume goalie is in the middle of the goal
        goalPos = field->posOwnGoalMid();
        //}

        auto goaltoball = alloBallPos - goalPos;
        auto defenderRange = goalPos + (goaltoball->normalize()) * min(4300.0, goaltoball->length() - 1750.0);
        if (defenderRange->x
                < -(msl::MSLFootballField::FieldLength / 2) + msl::MSLFootballField::PenaltyAreaLength + 100)
        {
            defenderRange->x = -(msl::MSLFootballField::FieldLength / 2) + msl::MSLFootballField::PenaltyAreaLength
                    + 100;
        }

        /*
         if (alloBallPos->y <= 0)
         {

         }
         else
         {

         }
         */
        MotionControl mc = msl::RobotMovement::moveToPointFast(defenderRange->alloToEgo(*me),
                                                               alloBallPos->alloToEgo(*me), 100, nullptr);

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
