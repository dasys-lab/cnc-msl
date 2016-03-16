using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

/*PROTECTED REGION ID(inccpp1447863466691) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <cmath>
#include <vector>
#include <string>
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1447863466691) ENABLED START*/ //initialise static variables here
	const string WatchBall::LEFT = "LEFT";
	const string WatchBall::MID = "MID";
	const string WatchBall::RIGHT = "RIGHT";
	/*
	 *
	 *			 _______________________________
	 *			|								|
	 *			| <---------- 2000mm ---------> |
	 *			|   ._.		   ._.		  ._.	|
	 *			| \/   \/	 \/   \/    \/   \/ |
	 *	 _______| |__0__| _ _|__1__|_ _ |__2__| |__________
	 *
	 *			  \__ __/
	 *				 V
	 *			140+630+140
	 *			   910mm
	 */
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
		//cout << "### WatchBall ###" << endl;
		me = wm->rawSensorData.getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> alloBall = wm->ball.getAlloBallPosition();

		if (me == nullptr)
		{
			cout << "me is null" << endl;
			return;
		}

		if (SIMULATING > 0)
			alloGoalMid = MSLFootballField::posOppGoalMid();
		else
			alloGoalMid = MSLFootballField::posOwnGoalMid();

		alloGoalLeft = make_shared<geometry::CNPoint2D>(alloGoalMid->x, alloGoalMid->y + GOALIE_SIZE / 2 * SIMULATING);
		alloGoalRight = make_shared<geometry::CNPoint2D>(alloGoalMid->x, alloGoalMid->y - GOALIE_SIZE / 2 * SIMULATING);

		if (targetPosBuffer[targetIndex] == nullptr)
			targetPosBuffer[targetIndex] = alloGoalMid;

		if (alloBall == nullptr)
		{
			cout << "Goalie can't see ball! Moving to OldTarget (init with GoalMid)" << endl;
			auto prevTarget = targetPosBuffer[modRingBuffer(targetIndex - 1, TARGET_BUFFER_SIZE)];
			if (prevTarget == nullptr)
			{
				prevTarget = alloGoalMid;
			}
			sendMC(fitTargetY(prevTarget->y));
			return;
		}
		else
		{
			if (abs(alloBall->x) > abs(alloGoalMid->x) + 50)
			{
				cout << "Ball is behind goal line" << endl;
				sendMC(MID);
				return;
			}
			else
				moveInsideGoal(alloBall, me);
		}
		ballIndex = modRingBuffer(ballIndex + 1, BALL_BUFFER_SIZE);
		targetIndex = modRingBuffer(targetIndex + 1, TARGET_BUFFER_SIZE);
		//cout << "### WatchBall ###\n" << endl;
		/*PROTECTED REGION END*/
	}
	void WatchBall::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
	void WatchBall::moveInsideGoal(shared_ptr<geometry::CNPoint2D> alloBall, shared_ptr<geometry::CNPosition> me)
	{
		ballPosBuffer[ballIndex] = alloBall;
		auto alloTrgt = calcGoalImpactY(TARGET_BUFFER_SIZE);
		if (alloTrgt != nullptr && alloBall != nullptr)
		{
			cout << "calcY  : " << alloTrgt->y << endl;
			cout << "actualY: " << alloBall->y << endl;
		}
		auto prevTarget = targetPosBuffer[modRingBuffer(targetIndex - 1, TARGET_BUFFER_SIZE)];

		if (alloTrgt == nullptr)
		{
			cout << "alloTarget NULL" << endl;
			if (prevTarget != nullptr)
				alloTrgt = prevTarget;
			else
				cout << "prevTarget NULL" << endl;
			alloTrgt = alloGoalMid;
		}
		string targetPos = fitTargetY(alloTrgt->y);

		//cout << "currentBall: " << ballPosBuffer[ballIndex]->toString() << endl;
		//cout << "actualTarget: " << alloTrgt->toString();
		sendMC(targetPos);
		/*mc = RobotMovement::moveToPointFast(alloTrgt->alloToEgo(*me), alloAlignPt->alloToEgo(*me), 100, 0);
		 send(mc);*/
	}

	shared_ptr<geometry::CNPoint2D> WatchBall::calcGoalImpactY(int nPoints)
	{
		double _slope, _yInt;
		double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

		int prePrevIndex = modRingBuffer(targetIndex - 2, TARGET_BUFFER_SIZE);
		int prevIndex = modRingBuffer(targetIndex - 1, TARGET_BUFFER_SIZE);

		auto prePrevTarget = targetPosBuffer[prePrevIndex];
		auto prevTarget = targetPosBuffer[prevIndex];
		auto currentTarget = prevTarget;

		for (int i = modRingBuffer(ballIndex - nPoints, BALL_BUFFER_SIZE);
				i < modRingBuffer(ballIndex, BALL_BUFFER_SIZE); i = modRingBuffer(i + 1, BALL_BUFFER_SIZE))
		{
			if (ballPosBuffer[i] == nullptr)
			{
				// need more points to
				cout << "calcGoal failed! [ballPosBuffer]" << endl;
				//return prevTarget;
			}
			sumX += ballPosBuffer[i]->x;
			sumY += ballPosBuffer[i]->y;
			sumXY += ballPosBuffer[i]->y * ballPosBuffer[i]->x;
			sumX2 += ballPosBuffer[i]->x * ballPosBuffer[i]->x;
		}

		double xMean = sumX / nPoints;
		double yMean = sumY / nPoints;
		double denominator = sumX2 - sumX * xMean;

		// You can tune the eps (1e-7)
		if (std::fabs(denominator) < 1e-3)
		{
			// Fail: it seems a vertical line
			//cout << "calcGoal failed! [vertical Line ]" << endl;
			return prevTarget;
		}

		_slope = (sumXY - sumX * yMean) / denominator;
		_yInt = yMean - _slope * xMean;
		double calcTargetY = _slope * alloGoalMid->x + _yInt;

		if (prevTarget != nullptr && prePrevTarget != nullptr)
		{
			int factor = 1.3;
			double meanY = (prePrevTarget->y + prevTarget->y * factor + calcTargetY) / 3;
			string targetPos = fitTargetY(meanY);
			if (targetPos == LEFT)
				currentTarget->y = alloGoalLeft->y;
			else if (targetPos == RIGHT)
				currentTarget->y = alloGoalRight->y;
			else
				currentTarget->y = alloGoalMid->y;
		}
		targetPosBuffer[targetIndex] = make_shared<geometry::CNPoint2D>(alloGoalMid->x, calcTargetY);
		//cout << "calcTargetY: " << calcTargetY << endl;
		//cout << "BallPos: " << ballPosBuffer[ballIndex]->toString();
		return currentTarget;
	}

	string WatchBall::fitTargetY(double targetY)
	{
		int buffer = 225;
		//cout << "leftCond : " << targetY * SIMULATING << ">" << alloGoalLeft->y - buffer << endl;
		//cout << "rightCond: " << targetY * SIMULATING << "<" << alloGoalRight->y + buffer << endl;
		//cout << "before fitTargetY: " << targetY << endl;

		if (targetY * SIMULATING > alloGoalLeft->y - buffer)
		{
			//targetY = alloGoalLeft->y;
			//cout << "left" << endl;
			return LEFT;
		}
		else if (targetY * SIMULATING < alloGoalRight->y + buffer)
		{
			//targetY = alloGoalRight->y;
			//cout << "right" << endl;
			return RIGHT;
		}
		else
		{
			//targetY = alloGoalMid->y;
			//cout << "mid" << endl;
			return MID;
		}
		//cout << "after fitTargetY: " << targetY << "\n" << endl;
	}

	void WatchBall::sendMC(string targetPos)
	{
		shared_ptr<geometry::CNPoint2D> egoALignPoint = alloAlignPt->alloToEgo(*me);
		cout << "TARGET: ";
		if (targetPos == LEFT)
		{
			mc = RobotMovement::moveToPointFast(alloGoalLeft->alloToEgo(*me), egoALignPoint, 100, 0, true);
			cout << alloGoalLeft->y << " left" << endl;
		}
		else if (targetPos == RIGHT)
		{
			mc = RobotMovement::moveToPointFast(alloGoalRight->alloToEgo(*me), egoALignPoint, 100, 0, true);
			cout << alloGoalRight->y << " right" << endl;
		}
		else
		{
			mc = RobotMovement::moveToPointFast(alloGoalMid->alloToEgo(*me), egoALignPoint, 100, 0, true);
			cout << alloGoalMid->y << " mid" << endl;
		}
		send(mc);
	}

	int WatchBall::modRingBuffer(int k, int bufferSize)
	{
		return ((k %= bufferSize) < 0) ? k + bufferSize : k;
	}
/*PROTECTED REGION END*/
} /* namespace alica */
