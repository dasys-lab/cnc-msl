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
        cout << "### WatchBall ###" << endl;

        shared_ptr < geometry::CNPosition > me = wm->rawSensorData.getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > goalMid = MSLFootballField::posOwnGoalMid();
        msl_actuator_msgs::MotionControl mc;

        double targetX = MSLFootballField::posOwnGoalMid()->egoToAllo(*me)->x - 100;
        double targetY = wm->ball.getEgoBallPosition()->y;

        cout << " Watching ball" << endl;
        double leftGoalPost = MSLFootballField::posLeftOwnGoalPost()->alloToEgo(*me)->y;
        double rightGoalPost = MSLFootballField::posRightOwnGoalPost()->alloToEgo(*me)->y;

        int centerToArmDist = 445; 	// 630mm/2 + 140mm = 445mm
        int ballRadius = 105; 		// Umfang 68cm => Radius 10.8225cm
        //int puffer = ballRadius;
        int puffer = ballRadius + centerToArmDist;

        // drive closer to goal if arms have been shot in the previous 4 seconds
        /*if(!shotInPrevFourSeconds) {
			puffer += centerToArmDist;
        }*/

        // ball position is outside of goalposts
        if (targetY <= leftGoalPost || (targetY > leftGoalPost && targetY <= leftGoalPost + puffer))
        {
            cout << "  - y: " << targetY << endl;
            targetY = leftGoalPost + puffer;
        }
        else if (targetY >= rightGoalPost || (targetY < rightGoalPost && targetY >= rightGoalPost - puffer))
        {
            cout << "  - y: " << targetY << endl;
            targetY = rightGoalPost - puffer;
        }
        // ball position is between goalposts
        /*else if (leftGoalPost + puffer < targetY)
		{
        	targetY = leftGoalPost - puffer;
		}
        else if(rightGoalPost - puffer > targetY)
        {
        	targetY = rightGoalPost + puffer;
        }*/

        auto egoTarget = make_shared < geometry::CNPoint2D > (targetX, targetY);
        mc = RobotMovement::moveToPointFast(egoTarget, goalMid, 100, 0);
        //mc = RobotMovement::moveToPointCarefully(egoTarget, goalMid, 100, 0);
        cout << "### WatchBall ###\n" << endl;

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
