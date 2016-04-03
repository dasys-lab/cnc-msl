using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

/*PROTECTED REGION ID(inccpp1447863466691) ENABLED START*/ //Add additional includes here
#include <cmath>
#include <vector>
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
		maxVariance = (*this->sc)["Behaviour"]->get<int>("Goalie.MaxVariance", NULL);
		goalieSize = (*this->sc)["Behaviour"]->get<int>("Goalie.GoalieSize", NULL);
		nrOfPositions = (*this->sc)["Behaviour"]->get<int>("Goalie.NrOfPositions", NULL);
		pFactor = (*this->sc)["Behaviour"]->get<double>("Goalie.pFactor", NULL);
		dFactor = (*this->sc)["Behaviour"]->get<double>("Goalie.dFactor", NULL);
		snapDistance = (*this->sc)["Behaviour"]->get<int>("Goalie.SnapDistance", NULL);
		alignMaxVel = (*sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
		fastRotation = fastRotation = (*sc)["Drive"]->get<double>("Drive.Fast.RotateP", NULL);
		ballPositions = new RingBuffer<geometry::CNPoint2D>(nrOfPositions);
		this->field = MSLFootballField::getInstance();
		auto tempMid = alloGoalMid = field->posOwnGoalMid();
		alloGoalMid = make_shared<geometry::CNPoint2D>(tempMid->x, tempMid->y);
		alloGoalLeft = make_shared<geometry::CNPoint2D>(alloGoalMid->x,
														field->posLeftOwnGoalPost()->y - goalieSize / 2);
		alloGoalRight = make_shared<geometry::CNPoint2D>(alloGoalMid->x,
															field->posRightOwnGoalPost()->y + goalieSize / 2);
		prevTargetDist = 0;
		/*PROTECTED REGION END*/
	}
	WatchBall::~WatchBall()
	{
		/*PROTECTED REGION ID(dcon1447863466691) ENABLED START*/ //Add additional options here
		delete ballPositions;
		/*PROTECTED REGION END*/
	}
	void WatchBall::run(void* msg)
	{
		/*PROTECTED REGION ID(run1447863466691) ENABLED START*/ //Add additional options here
//		cout << "####### WatchBall #######" << endl;
		ownPos = wm->rawSensorData.getOwnPositionVision();
		if (ownPos == nullptr)
		{
			cout << "[WatchBall]: ownPos is null" << endl;
			return;
		}

		shared_ptr<geometry::CNPoint2D> alloBall = wm->ball.getAlloBallPosition();
		// > abs(alloGoalMid->x) + 50
		if (alloBall == nullptr || abs(alloBall->x) > abs(alloGoalMid->x) + 50)
		{
			cout << "[WatchBall]: Goalie can't see ball! Moving to prevTarget" << endl;
			moveGoalie(prevTarget, nullptr);
			return;
		}

		this->ballPositions->add(alloBall);
		observeBall(alloBall->alloToEgo(*ownPos));

		/*PROTECTED REGION END*/
	}
	void WatchBall::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ //Add additional options here
		prevTarget = MSLFootballField::getInstance()->posOwnGoalMid();
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
	void WatchBall::observeBall(shared_ptr<geometry::CNPoint2D> egoBall)
	{
		shared_ptr<geometry::CNPoint2D> alloTarget;
		if (ballPositions->getSize() > 0)
		{
			double targetY = calcGoalImpactY();
			targetY = fitTargetY(targetY);
			alloTarget = make_shared<geometry::CNPoint2D>(alloGoalMid->x, targetY);
			prevTarget = alloTarget;
		}
		else
		{
			alloTarget = prevTarget;
		}

		//cout << "[WatchBall] alloBall:" << wm->ball.getAlloBallPosition()->toString();
		moveGoalie(alloTarget, egoBall);
	}

	void WatchBall::moveGoalie(shared_ptr<geometry::CNPoint2D> alloTarget, shared_ptr<geometry::CNPoint2D> egoBall)
	{
		//alloTarget->x = -msl::MSLFootballField::FieldLength/2;
		auto egoTarget = alloTarget->alloToEgo(*ownPos);
		cout << alloTarget->toString() << endl;
		cout << egoTarget->toString() << endl;
		mc.motion.angle = egoTarget->angleTo();
		mc.motion.rotation = alloFieldCntr->alloToEgo(*ownPos)->rotate(M_PI)->angleTo() * fastRotation;

		if (egoBall != nullptr && egoBall->egoToAllo(*ownPos) != nullptr && egoBall->egoToAllo(*ownPos)->x > 1000)
		{
			cout << "[WatchBall] Ball in opp side, goalie moves with half translation" << endl;
			pFactor = pFactor / 2;
		}

		// + (neuDist - altDist) * dfactor
		if (egoTarget->length() > snapDistance)
		{
			mc.motion.translation = std::min(
					alignMaxVel, (egoTarget->length() * pFactor) + ((egoTarget->length() - prevTargetDist) * dFactor));
			//cout << "[WatchBall] targetDistance: " << egoTarget->length() << endl;
			cout << "[WatchBall] TRANSLATION: " << mc.motion.translation << endl;
			cout << endl;
		}
		else
		{
			mc.motion.translation = 0;
			//cout << "[WatchBall] arrived at target!" << endl;
		}

		prevTargetDist = egoTarget->length();
		send(mc);
	}

	double WatchBall::calcGoalImpactY()
	{
		double _slope, _yInt;
		double sumXY = 0, sumX2 = 0, sumX2Y2 = 0;
		shared_ptr<geometry::CNPoint2D> avgBall = make_shared<geometry::CNPoint2D>(0.0, 0.0);
		int nPoints = 0;
		for (int i = 0; i < ballPositions->getSize(); i++)
		{
			auto currentBall = ballPositions->getLast(i);
			shared_ptr<geometry::CNPoint2D> ppprevBall;
			shared_ptr<geometry::CNPoint2D> pprevBall;
			shared_ptr<geometry::CNPoint2D> prevBall;
			if (i > 2)
			{
				ppprevBall = ballPositions->getLast(i - 3);
				pprevBall = ballPositions->getLast(i - 2);
				prevBall = ballPositions->getLast(i - 1);
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
						//cout << "[WatchBall] corner detected! cond1" << endl;
						break;
					}
				}
				else
				{
					if (!(diffpp <= diffp || diffpp + buffer <= diffp || diffpp - buffer <= diffp))
					{
						//cout << "[WatchBall] corner detected! cond2" << endl;
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
		if (nPoints > 1 && variance > maxVariance)
		{
			cout << "[WatchBall] LinearRegression: Variance: " << variance << endl;
			for (int i = 0; i < nPoints; i++)
			{
				auto curBall = ballPositions->getLast(i);
				nomi = nomi + ((curBall->x - avgBall->x) * (curBall->y - avgBall->y));
				denom = denom + ((curBall->x - avgBall->x) * (curBall->x - avgBall->x));
			}
			if (denom < 1e-3)
			{
				cout << "[WatchBall] LinearRegression: prevTarget, cause no hitPoint " << endl;
				return prevTarget->y;
			}
			_slope = nomi / denom;
			_yInt = avgBall->y - _slope * avgBall->x;
			calcTargetY = _slope * alloGoalMid->x + _yInt;
			cout << "[WatchBall] LinearRegression: calcTargetY   : " << calcTargetY << endl;
		}
		else
		{
			// TODO: use this as soon as Goalie Vision detects Obstacles better!
			auto obstacles = wm->obstacles.getAlloObstaclePoints();
			shared_ptr<geometry::CNPoint2D> closestObstacle; // = make_shared<geometry::CNPoint2D>(0.0, 0.0);
			double minDistBallObs = 20000;
			for (auto currentObs : *obstacles)
			{
				//cout << "[WatchBall] " << currentObs->toString();

				double currentDistBallObs = currentObs->distanceTo(ballPositions->getLast(0));
				if (currentObs->distanceTo(ownPos) < ballPositions->getLast(0)->distanceTo(ownPos)
						|| currentDistBallObs > 1000)
				{
					continue;
				}
				if (currentDistBallObs < minDistBallObs)
				{
					closestObstacle = currentObs;
					minDistBallObs = currentDistBallObs;
				}
			}

			if (closestObstacle != nullptr)
			{
				//cout << "[WatchBall] Obstacle Variance: " << variance << endl;
				_slope = (closestObstacle->y - ballPositions->getLast(0)->y)
						/ (closestObstacle->x - ballPositions->getLast(0)->x);
				_yInt = ballPositions->getLast(0)->y - _slope * ballPositions->getLast(0)->x;
				calcTargetY = _slope * alloGoalMid->x + _yInt;
			}
			else
			{
				cout << "[WatchBall] BallY Variance: " << variance << endl;
				calcTargetY = ballPositions->getLast(0)->y;
			}
		}
		return calcTargetY;
	}

	double WatchBall::fitTargetY(double targetY)
	{

		if (targetY > alloGoalLeft->y)
		{
			//cout << "[WatchBall] fitTarget left: " << alloGoalLeft->y << endl;
			return alloGoalLeft->y;
		}
		else if (targetY < alloGoalRight->y)
		{
			//cout << "[WatchBall] fitTarget right: " << alloGoalRight->y << endl;
			return alloGoalRight->y;
		}
		else
		{
			//cout << "[WatchBall] fitTarget else: " << targetY << endl;
			return targetY;
		}
	}
/*PROTECTED REGION END*/
} /* namespace alica */
