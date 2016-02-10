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

		me = wm->rawSensorData.getOwnPositionVision();
		if(me == nullptr) {
			cout << "me is null" << endl;
			return;
		}
		goalMid = MSLFootballField::posOwnGoalMid();

		double targetX = goalMid->egoToAllo(*me)->x - 100;
		double targetY = wm->ball.getEgoBallPosition()->y;

		double leftGoalPost = MSLFootballField::posLeftOwnGoalPost()->alloToEgo(*me)->y;
		double rightGoalPost = MSLFootballField::posRightOwnGoalPost()->alloToEgo(*me)->y;

		int goalieHalfSize = 315; // 630mm/2 + 140mm = 445mm
		int extendedArmWidth = 140;
		int ballRadius = (int) wm->ball.getBallDiameter()/2; // Umfang 68cm => Radius 10.8225cm
		int puffer = ballRadius + goalieHalfSize;
		int shotInPrevFourSec = 0;

		// TODO:
		//		call subscriber, which gets a timestamp from last Goalie Kick.
		//		if >=4sec => shotInFourSec = 0 else shotInFourSec = 1;

		if (shotInPrevFourSec)
		{
			// drives 445mm (centerToArmDist) closer to goal because arms have been shot in the previous 4 seconds
			puffer += extendedArmWidth;
		}

		if (targetY <= leftGoalPost || (targetY > leftGoalPost && targetY <= leftGoalPost + puffer))
		{
			targetY = leftGoalPost + puffer;
		}
		else if (targetY >= rightGoalPost || (targetY < rightGoalPost && targetY >= rightGoalPost - puffer))
		{
			targetY = rightGoalPost - puffer;
		}
		// else, ballY is between goalposts and not closer than ballRadius + robot's CenterToArmDistance away from goal posts

		auto egoTarget = make_shared<geometry::CNPoint2D>(targetX, targetY);
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
/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
