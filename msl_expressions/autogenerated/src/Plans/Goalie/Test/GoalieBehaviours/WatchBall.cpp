using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

/*PROTECTED REGION ID(inccpp1447863466691) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <cmath>
/*PROTECTED REGION END*/
namespace alica
{
<<<<<<< HEAD
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

		shared_ptr<geometry::CNPosition> me = wm->rawSensorData.getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> goalMid = MSLFootballField::posOwnGoalMid();
		msl_actuator_msgs::MotionControl mc;

		double targetX = MSLFootballField::posOwnGoalMid()->egoToAllo(*me)->x - 100;
		double targetY = wm->ball.getEgoBallPosition()->y;

		cout << " Watching ball" << endl;
		double leftGoalPost = MSLFootballField::posLeftOwnGoalPost()->alloToEgo(*me)->y;
		double rightGoalPost = MSLFootballField::posRightOwnGoalPost()->alloToEgo(*me)->y;

		// TODO: armlength when extended and balldiameter/2
		double puffer = 100 + 200;

		if (targetY < leftGoalPost)
		{
			cout << "  - y: " << targetY << endl;
			targetY = leftGoalPost + puffer;
		}
		else if (targetY > rightGoalPost)
		{
			cout << "  - y: " << targetY << endl;
			targetY = rightGoalPost - puffer;
		}

		shared_ptr<geometry::CNPosition> egoTarget = make_shared<geometry::CNPoint2D>(targetX, targetY);
		mc = RobotMovement::moveToPointFast(egoTarget, goalMid, 100, 0);
		cout << "### WatchBall ###\n" << endl;

		send(mc);
		/*PROTECTED REGION END*/
	}
	void WatchBall::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
=======
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
>>>>>>> 897ba8acbda3dad27447f4ab08472d5168819832
/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
