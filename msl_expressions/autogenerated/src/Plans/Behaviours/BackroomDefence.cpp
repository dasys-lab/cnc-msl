using namespace std;
#include "Plans/Behaviours/BackroomDefence.h"

/*PROTECTED REGION ID(inccpp1454507752863) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1454507752863) ENABLED START*/ //initialise static variables here
    double static DESTINATION_DISTANCE = 300;
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
        auto ballPos = wm->ball.getEgoBallPosition();
        auto goaliePos = wm->robots.getTeamMatePosition(1, 0);
        auto field = msl::MSLFootballField::getInstance();
        shared_ptr < geometry::CNPoint2D > goalPos;

        if (!me || !ballPos)
        {
            return;
        }

        if (goaliePos)
        {
            goalPos = goaliePos->getPoint();
        }
        else
        {
            // assume goalie is in the middle of the goal
            goalPos = field->posOwnGoalMid();
        }

        double robotX = 0.25 * field->FieldLength;
        double goalX = goalPos->x;
        double goalY = goalPos->y;
        double ballX = ballPos->x;
        double ballY = ballPos->y;

        double factor = (robotX - goalX) / (ballX - goalX);
        double robotY = goalY + factor * (ballY - goalY);

        auto intersectPos = make_shared < geometry::CNPoint2D > (robotX, robotY);
        MotionControl mc = msl::RobotMovement::moveToPointFast(intersectPos->alloToEgo(*me), ballPos, 100, nullptr);

        if (me->distanceTo(intersectPos) < DESTINATION_DISTANCE)
        {
            // robot is too close to intersection point, move away from it
            mc.motion.translation *= -1;
        }

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
