using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

/*PROTECTED REGION ID(inccpp1447863466691) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <cmath>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1447863466691) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    WatchBall::WatchBall() :
            DomainBehaviour("WatchBall")
    {
        /*PROTECTED REGION ID(con1447863466691) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    WatchBall::~WatchBall()
    {
        /*PROTECTED REGION ID(dcon1447863466691) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void WatchBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1447863466691) ENABLED START*/ //Add additional options here
        msl_actuator_msgs::MotionControl mc;

        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();
        shared_ptr < geometry::CNPoint2D > goalMid = MSLFootballField::posOwnGoalMid();
        shared_ptr < geometry::CNPosition > me = wm->rawSensorData.getOwnPositionVision();

        double targetX = goalMid->alloToEgo(*me)->x - 100;
        double targetY = egoBallPos->y;
        double penaltyLength = MSLFootballField::PenaltyAreaLength;
        double goalWidth = MSLFootballField::GoalAreaWidth;

        // TODO: Set y-borders, armlength away from goal posts and let goalie stop once reached

        //cout << "targetX: " << targetX << endl;
        //cout << "targetY: " << targetY<< endl;

        mc = RobotMovement::moveToPointFast(make_shared < geometry::CNPoint2D > (targetX, targetY), goalMid, 100, 0);
        send(mc);

        /*PROTECTED REGION END*/
    }
    void WatchBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
