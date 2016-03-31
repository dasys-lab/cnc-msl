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
	 *			110+720+110
	 *			   920mm
	 */
	/*PROTECTED REGION END*/
	WatchBall::WatchBall() :
			DomainBehaviour("WatchBall")
	{
		/*PROTECTED REGION ID(con1447863466691) ENABLED START*/ //Add additional options here
		simulating = (*this->sc)["Behaviour"]->get<int>("Goalie.Simulating", NULL);
		maxVariance = (*this->sc)["Behaviour"]->get<int>("Goalie.MaxVariance", NULL);
		goalieSize = (*this->sc)["Behaviour"]->get<int>("Goalie.GoalieSize", NULL);
		nrOfPositions = (*this->sc)["Behaviour"]->get<int>("Goalie.NrOfPositions", NULL);
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
		shared_ptr<geometry::CNPoint2D> alloBall = wm->ball.getAlloBallPosition();

		if (me == nullptr)
		{
			cout << "me is null" << endl;
			return;
		}

		alloGoalMid = MSLFootballField::getInstance()->posOwnGoalMid();
		alloGoalLeft = make_shared<geometry::CNPoint2D>(
				alloGoalMid->x, MSLFootballField::getInstance()->posLeftOwnGoalPost()->y - goalieSize / 2);
		alloGoalRight = make_shared<geometry::CNPoint2D>(
				alloGoalMid->x, MSLFootballField::getInstance()->posRightOwnGoalPost()->y + goalieSize / 2);

		if (alloBall == nullptr || abs(alloBall->x) > abs(alloGoalMid->x) + 50)
		{
			cout << "Goalie can't see ball! Moving to GoalMid" << endl;
			mc = RobotMovement::moveGoalie(prevTarget, alloFieldCntr, SNAP_DIST);
			send(mc);
			return;
		}
		else
		{
			watchBall();
		}
		//cout << "### WatchBall ###\n" << endl;
		/*PROTECTED REGION END*/
	}
	void WatchBall::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ //Add additional options here
		prevTarget = MSLFootballField::getInstance()->posOwnGoalMid();
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
	void WatchBall::watchBall()
	{
		std::vector<shared_ptr<geometry::CNPoint2D>> ballPositions;
		for (int i = 0; i < nrOfPositions; i++)
		{
			auto currentBall = wm->ball.getVisionBallPosition(i);
			if (currentBall)
			{
				ballPositions.push_back(currentBall->egoToAllo(*me));
			}
		}

		shared_ptr<geometry::CNPoint2D> alloTarget;
		if (ballPositions.size() > 0)
		{
			alloTarget = calcGoalImpactY(ballPositions);
			prevTarget = alloTarget;
		}
		else
		{
			alloTarget = prevTarget;
		}

		mc = RobotMovement::moveGoalie(alloTarget, alloFieldCntr, SNAP_DIST);
		send(mc);
	}

	shared_ptr<geometry::CNPoint2D> WatchBall::calcGoalImpactY(
			std::vector<shared_ptr<geometry::CNPoint2D>>& ballPositions)
	{
		double _slope, _yInt;
		double sumXY = 0, sumX2 = 0, sumX2Y2 = 0;
		shared_ptr<geometry::CNPoint2D> avgBall = make_shared<geometry::CNPoint2D>(0.0, 0.0);
		int nPoints = 0;

		for (int i = 0; i < ballPositions.size(); i++)
		{
			auto currentBall = ballPositions.at(i);

			shared_ptr<geometry::CNPoint2D> ppprevBall;
			shared_ptr<geometry::CNPoint2D> pprevBall;
			shared_ptr<geometry::CNPoint2D> prevBall;

			if (i > 2)
			{
				ppprevBall = ballPositions.at(i - 3);
				pprevBall = ballPositions.at(i - 2);
				prevBall = ballPositions.at(i - 1);
			}

			if (prevBall != nullptr && pprevBall != nullptr && ppprevBall != nullptr)
			{
				// check if function is continual
				double diffppp = ppprevBall->y - pprevBall->y;
				double diffpp = pprevBall->y - prevBall->y;
				double diffp = prevBall->y - currentBall->y;
				int buffer = 100;

				if (diffppp >= diffpp)
				{
					if (!(diffpp >= diffp || diffpp + buffer >= diffp || diffpp - buffer >= diffp))
					{
						cout << "[WatchBall] corner detected! cond1" << endl;
						break;
					}
				}
				else
				{
					if (!(diffpp <= diffp || diffpp + buffer <= diffp || diffpp - buffer <= diffp))
					{
						cout << "[WatchBall] corner detected! cond2" << endl;
						break;
					}
				}
			}
//			cout << "[WatchBall] currentBall: " << currentBall->toString() << endl;

			avgBall->x += currentBall->x;
			avgBall->y += currentBall->y;

			sumXY = sumXY + (currentBall->y * currentBall->x);
			sumX2 = sumX2 + (currentBall->x * currentBall->x);
			sumX2Y2 = sumX2Y2 + (currentBall->x * currentBall->x + currentBall->y * currentBall->y);

			nPoints = nPoints + 1;
		}

		double sumX = avgBall->x;
		double sumY = avgBall->y;
		avgBall = avgBall / nPoints;
		double denom = 0;
		double nomi = 0;

		double calcTargetY;
		double variance = (sumX2Y2 + nPoints * ((avgBall->x * avgBall->x) + (avgBall->y * avgBall->y))
				- 2 * ((avgBall->x * sumX) + (avgBall->y * sumY))) / nPoints;
		cout << "[WatchBall] Variance: " << variance << endl;
		if (nPoints > 1 && variance > maxVariance)
		{
			for (int i = 0; i < nPoints; i++)
			{
				auto curBall = ballPositions.at(i);
				nomi = nomi + ((curBall->x - avgBall->x) * (curBall->y - avgBall->y));
				denom = denom + ((curBall->x - avgBall->x) * (curBall->x - avgBall->x));
			}

			if (denom < 1e-3)
			{
				cout << "[WatchBall] prevTarget, cause no hitPoint " << endl;
				return prevTarget;
			}

			_slope = nomi / denom;
			_yInt = avgBall->y - _slope * avgBall->x;

			calcTargetY = _slope * alloGoalMid->x + _yInt;
			cout << "[WatchBall] calcTargetY   : " << calcTargetY << endl;
		}
		else
		{
			cout << "[WatchBall] noRegression ball y is " << ballPositions.at(0)->y << endl;
			calcTargetY = ballPositions.at(0)->y;
		}

		calcTargetY = fitTargetY(calcTargetY);
//		cout << "[WatchBall] ballPosX      : " << ballPositions.at(0)->x << endl;
//		cout << "[WatchBall] ballPosY      : " << ballPositions.at(0)->y << endl;
		return make_shared<geometry::CNPoint2D>(alloGoalMid->x, calcTargetY);
	}

	double WatchBall::fitTargetY(double targetY)
	{

		if (targetY > alloGoalLeft->y)
		{
			return alloGoalLeft->y;
		}
		else if (targetY < alloGoalRight->y)
		{
			return alloGoalRight->y;
		}
		else
		{
			return targetY;
		}
	}

	int WatchBall::modRingBuffer(int k, int bufferSize)
	{
		return ((k %= bufferSize) < 0) ? k + bufferSize : k;
	}
/*PROTECTED REGION END*/
} /* namespace alica */
