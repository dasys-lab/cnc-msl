using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

/*PROTECTED REGION ID(inccpp1447863466691) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <cmath>
#include <vector>
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
		shared_ptr<geometry::CNPosition> me = wm->rawSensorData.getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> alloBall = wm->ball.getAlloBallPosition();
		;
		shared_ptr<geometry::CNVelocity2D> egoBallVel;

		alloFieldCenter = MSLFootballField::posCenterMarker();

		if (SIMULATING > 0)
			alloGoalMid = MSLFootballField::posOppGoalMid();
		else
			alloGoalMid = MSLFootballField::posOwnGoalMid();

		if (me == nullptr || alloGoalMid == nullptr)
		{
			cout << "me: " << me->toString() << ", goalMid: " << alloGoalMid->toString() << endl;
			return;
		}

		if (oldAlloTarget == nullptr)
			oldAlloTarget = alloGoalMid;

		if (oldAlloAlignPoint == nullptr)
			oldAlloAlignPoint = alloBall;

		if (alloBall == nullptr)
		{
			cout << "Goalie can't see ball! Moving to OldTarget (init with GoalMid)" << endl;
			mc = RobotMovement::moveToPointFast(oldAlloTarget->alloToEgo(*me), alloFieldCenter->alloToEgo(*me), 100, 0);
			send(mc);
		}
		else
		{
			egoBallVel = wm->ball.getEgoBallVelocity();
			int goalieHalfSize = 315; // 630mm/2 + 140mm = 445mm
			//int extendedArmWidth = 140;
			int ballRadius = (int)wm->ball.getBallDiameter() / 2; // Umfang 68cm => Radius 10.8225cm

			//int puffer = ballRadius + goalieHalfSize + extendedArmWidth;
			int puffer = ballRadius + goalieHalfSize;

			shared_ptr<geometry::CNPoint2D> alloTarget = make_shared<geometry::CNPoint2D>(0.0, 0.0);
			bool success = calcGoalImpactY(alloTarget, nPoints, puffer);
			shared_ptr<geometry::CNPoint2D> alloAlignPoint = alloBall;

			// fill buffer
			ballPosBuffer[currentIndex] = alloBall;
			shared_ptr<geometry::CNPoint2D> ballPos = wm->ball.getAlloBallPosition();

			if (success == true && egoBallVel != nullptr && abs(egoBallVel->x) > 100 && abs(egoBallVel->y) > 100)
			{
				if (alloAlignPoint->distanceTo(me) < 1000)
					alloAlignPoint = oldAlloAlignPoint;
				else
					oldAlloAlignPoint = alloAlignPoint;
			}
			else
			{
				alloTarget->y = oldAlloTarget->y;
			}

			alloTarget->x = alloGoalMid->x + (100 * SIMULATING);
			oldAlloTarget = alloTarget;
			mc = RobotMovement::moveToPointFast(alloTarget->alloToEgo(*me), alloAlignPoint->alloToEgo(*me), 100, 0);
			send(mc);
			cout << "### WatchBall ###\n" << endl;
		}
		currentIndex = (currentIndex + 1) % RING_BUFFER_SIZE;
		/*PROTECTED REGION END*/
	}
	void WatchBall::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
	bool WatchBall::calcGoalImpactY(shared_ptr<geometry::CNPoint2D> &alloTarget, int nPoints, int puffer)
	{

		double _slope, _yInt;
		double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

		for (int i = modRingBuffer(currentIndex - nPoints); i < modRingBuffer(currentIndex); i = modRingBuffer(i + 1))
		{
			if (ballPosBuffer[i] == nullptr)
			{
				// need more points to
				cout << "calcGoal failed! [ballPosBuffer]" << endl;
				return false;
			}
			sumX += ballPosBuffer[i]->x;
			sumY += ballPosBuffer[i]->y;
			sumXY += ballPosBuffer[i]->y * ballPosBuffer[i]->x;
			sumX2 += ballPosBuffer[i]->x * ballPosBuffer[i]->x;
		}

		double xMean = sumX / nPoints;
		double yMean = sumY / nPoints;
		double denominator = sumX2 - sumX * xMean;

		// You can tune the eps (1e-7) below for your specific task
		if (std::fabs(denominator) < 1e-3)
		{
			// Fail: it seems a vertical line
			cout << "calcGoal failed! [vertical Line]" << endl;
			return false;
		}

		_slope = (sumXY - sumX * yMean) / denominator;
		_yInt = yMean - _slope * xMean;

		double targetY = _slope * alloGoalMid->x + _yInt;

		double leftGoalPostY = MSLFootballField::posLeftOwnGoalPost()->y;
		double rightGoalPostY = MSLFootballField::posRightOwnGoalPost()->y;

		if (targetY < rightGoalPostY + puffer)
		{
			// calculated ballPos is close to or right side, outside of right GoalPost
			targetY = rightGoalPostY + puffer;
			//cout << "RightGoalPostY: " << targetY << endl;
		}
		else if (targetY > leftGoalPostY - puffer)
		{
			// calculated ballPos is close to or left side, outside of left GoalPost
			targetY = leftGoalPostY - puffer;
			//cout << "LeftGoalPostY: " << targetY << endl;
		}

		/*if (targetY <= (rightGoalPostY - 2*puffer) || (targetY > rightGoalPostY && targetY <= rightGoalPostY + puffer))
		 {
		 targetY = rightGoalPostY + puffer;
		 }
		 else if (targetY >= (leftGoalPostY + 2*puffer) || (targetY < leftGoalPostY && targetY >= leftGoalPostY - puffer))
		 {
		 targetY = leftGoalPostY - puffer;
		 }
		 // else, ballY is between goalposts and not closer than ballRadius + robot's CenterToArmDistance away from goal posts

		 /*if (targetY < -1000)
		 {
		 targetY = -1000 + puffer;
		 }
		 else if (targetY > 1000)
		 {
		 targetY = 1000 - puffer;
		 }*/

		alloTarget->y = targetY;

		cout << "calcGoal successfull!" << endl;
		return true;
	}

	int WatchBall::modRingBuffer(int k)
	{
		return ((k %= RING_BUFFER_SIZE) < 0) ? k + RING_BUFFER_SIZE : k;
	}
/*PROTECTED REGION END*/
} /* namespace alica */
