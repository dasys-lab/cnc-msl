using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

/*PROTECTED REGION ID(inccpp1447863466691) ENABLED START*/ //Add additional includes here
#include <cmath>
#include <vector>
#include <RawSensorData.h>
#include <Ball.h>
#include <obstaclehandler/Obstacles.h>
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
	 *				pos		   pos		  pos
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
		alignTowardsBall = (*this->sc)["Behaviour"]->get<bool>("Goalie.AlignTowardsBall", NULL);
		rotationLimit = (*this->sc)["Behaviour"]->get<double>("Goalie.RotationLimit", NULL);
		maxVariance = (*this->sc)["Behaviour"]->get<int>("Goalie.MaxVariance", NULL);
		goalieSize = (*this->sc)["Behaviour"]->get<int>("Goalie.GoalieSize", NULL);
		nrOfPositions = (*this->sc)["Behaviour"]->get<int>("Goalie.NrOfPositions", NULL);

		pTrans = (*this->sc)["Behaviour"]->get<double>("Goalie.pTrans", NULL);
		dTrans = (*this->sc)["Behaviour"]->get<double>("Goalie.dTrans", NULL);
		pRot = (*this->sc)["Behaviour"]->get<double>("Goalie.pRot", NULL);
		dRot = (*this->sc)["Behaviour"]->get<double>("Goalie.dRot", NULL);
		lastRotErr = 0;
		prevTargetDist = 0;

		snapDistance = (*this->sc)["Behaviour"]->get<int>("Goalie.SnapDistance", NULL);
		alignMaxVel = (*sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
		ballPositions = new RingBuffer<geometry::CNPoint2D>(nrOfPositions);
		auto tempMid = wm->field->posOwnGoalMid();
		alloGoalMid = make_shared<geometry::CNPoint2D>(tempMid->x, tempMid->y);
		alloGoalLeft = make_shared<geometry::CNPoint2D>(alloGoalMid->x,
														wm->field->posLeftOwnGoalPost()->y - goalieSize / 2 + 375);
		alloGoalRight = make_shared<geometry::CNPoint2D>(alloGoalMid->x,
															wm->field->posRightOwnGoalPost()->y + goalieSize / 2);
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
		ownPos = wm->rawSensorData->getOwnPositionVision();
		if (ownPos == nullptr)
		{
			mc.motion.translation = 0;
			mc.motion.angle = 0;
			mc.motion.rotation = 0;
			send(mc);
			cout << "[WatchBall]: ownPos is null" << endl;
			return;
		}

		shared_ptr<geometry::CNPoint2D> alloBall = wm->ball->getAlloBallPosition();

		// TODO: Keep?
		if (alloBall != nullptr && alloBall->x > -250)
		{
			mc.motion.translation = 0;
			shared_ptr<geometry::CNPoint2D> alignPoint = make_shared<geometry::CNPoint2D>(-ownPos->x, ownPos->y); // align to mirrored ownPos
			mc.motion.rotation = alignPoint->alloToEgo(*ownPos)->rotate(M_PI)->angleTo();
			send(mc);
			cout << "[WatchBall] ball is far away from goal! BallX: " << alloBall->x << endl;
			return;
		}
		// TODO: Keep?

		if (alloBall == nullptr || abs(alloBall->x) > abs(alloGoalMid->x) + 50)
		{

			/*
			 * Goalie drives to last known target and rotates towards mirrored own position
			 */

			cout << "[WatchBall]: Goalie can't see ball! Moving to prevTarget" << endl;
			mc.motion.angle = prevTarget->alloToEgo(*ownPos)->angleTo();
			rotate(make_shared<geometry::CNPoint2D>(-ownPos->x, ownPos->y));
			mc.motion.translation = std::min(
					alignMaxVel,
					(prevTarget->alloToEgo(*ownPos)->length() * pTrans)
							+ ((prevTarget->alloToEgo(*ownPos)->length() - prevTargetDist) * dTrans));
			send(mc);
			return;
		}

		this->ballPositions->add(alloBall);

		/*
		 * Calculate target position on goal line
		 */

		shared_ptr<geometry::CNPoint2D> alloTarget;
		double targetY;
		if (ballPositions->getSize() > 0)
		{
			targetY = calcGoalImpactY();
			targetY = fitTargetY(targetY);
			alloTarget = make_shared<geometry::CNPoint2D>(alloGoalMid->x + 200, targetY);
			prevTarget = alloTarget;
		}
		else
		{
			alloTarget = prevTarget;
			targetY = alloTarget->y;
		}

		auto egoBall = alloBall->alloToEgo(*ownPos);
		auto egoTarget = alloTarget->alloToEgo(*ownPos);

		/*
		 * Goalie drives to target
		 */

		mc.motion.angle = egoTarget->angleTo();
		rotate(alloBall);

		if (egoTarget->length() > snapDistance)
		{
			auto tempPFactor = pTrans;
			if (egoBall != nullptr && alloBall->x > alloFieldCntr->x + 1000)
			{
				// todo: remove
//				cout << "[WatchBall] Ball in opp side, goalie moves with half translation" << endl;
				tempPFactor = pTrans / 2;
			}

			double translation = (egoTarget->length() * tempPFactor)
					+ ((egoTarget->length() - prevTargetDist) * dTrans);
			mc.motion.translation = std::min(alignMaxVel, translation);
		}
		else
		{
			/*
			 * Goalie arrived at target!
			 */

			mc.motion.translation = 0;
		}

		send(mc);

		/*PROTECTED REGION END*/
	}
	void WatchBall::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ //Add additional options here
		prevTarget = wm->field->posOwnGoalMid();
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
	double WatchBall::calcGoalImpactY()
	{
		double _slope, _yInt;
		double sumXY = 0, sumX2 = 0, sumX2Y2 = 0;
		shared_ptr<geometry::CNPoint2D> avgBall = make_shared<geometry::CNPoint2D>(0.0, 0.0);
		int nPoints = 0;
		for (int i = 0; i < ballPositions->getSize(); i++)
		{
			auto currentBall = ballPositions->getLast(i);
			// p=prev, pp=prePrev, ppp=prePrePrev
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
//						cout << "[WatchBall] corner detected! cond1" << endl;
                        break;
                    }
                }
                else
                {
                    if (!(diffpp <= diffp || diffpp + buffer <= diffp || diffpp - buffer <= diffp))
                    {
//						cout << "[WatchBall] corner detected! cond2" << endl;
						break;
					}
				}
			}

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

		// TODO: Keep?
		std::shared_ptr<geometry::CNVelocity2D> alloBallVel = wm->ball->getEgoBallVelocity();

		if (alloBallVel != nullptr)
		{
			alloBallVel = alloBallVel->egoToAllo(*ownPos);
		}

		if (alloBallVel != nullptr && alloBallVel->x < -1000)
		{
			cout << "[WatchBall] alloBallVelX: " << alloBallVel->x << endl;
			calcTargetY = prevTarget->y;
		} // TODO: Keep?
		else if (nPoints > 1 && variance > maxVariance)
		{
			/*
			 * Ball is moving, so that its variance is greater than maxVariance
			 */
			cout << "[WatchBall] -LinearRegression- Variance   : " << variance << endl;
			for (int i = 0; i < nPoints; i++)
			{
				auto curBall = ballPositions->getLast(i);
				nomi = nomi + ((curBall->x - avgBall->x) * (curBall->y - avgBall->y));
				denom = denom + ((curBall->x - avgBall->x) * (curBall->x - avgBall->x));
			}
			if (denom < 1e-3)
			{
				// return prev, cause denom no valid value
				return prevTarget->y;
			}
			_slope = nomi / denom;
			_yInt = avgBall->y - _slope * avgBall->x;
			calcTargetY = _slope * alloGoalMid->x + _yInt;
//			cout << "[WatchBall] -LinearRegression- calcTargetY: " << calcTargetY << endl;
		}
		else
		{
			// TODO: use this when Goalie Vision detects Obstacles better?!
//            auto obstacles = wm->obstacles->getAlloObstaclePoints();
//            shared_ptr < geometry::CNPoint2D > closestObstacle; // = make_shared<geometry::CNPoint2D>(0.0, 0.0);
//            double minDistBallObs = 20000;
//            for (auto currentObs : *obstacles)
//            {
//                double currentDistBallObs = currentObs->distanceTo(ballPositions->getLast(0));
//                if (currentObs->distanceTo(ownPos) < ballPositions->getLast(0)->distanceTo(ownPos)
//                        || currentDistBallObs > 1000)
//                {
//                    continue;
//                }
//                if (currentDistBallObs < minDistBallObs)
//                {
//                    closestObstacle = currentObs;
//                    minDistBallObs = currentDistBallObs;
//                }
//            }
//
//            if (closestObstacle != nullptr)
//            {
//
//			/*
//			 * Goalie drives to calculated Impact point by using obstacle in front of ball and ball position itself
//			 */

//				cout << "[WatchBall] -Obstacle- Variance " << variance << endl;
//                _slope = (closestObstacle->y - ballPositions->getLast(0)->y)
//                        / (closestObstacle->x - ballPositions->getLast(0)->x);
//                _yInt = ballPositions->getLast(0)->y - _slope * ballPositions->getLast(0)->x;
//                calcTargetY = _slope * alloGoalMid->x + _yInt;
//				cout << "[WatchBall] -Obstacle- calcTargetY : " << calcTargetY << endl;
//            }
//            else
//            {
//
//			/*
//			 * Goalie drives to ball x position, meaning he just follows the ball
//			 */

			cout << "[WatchBall] -BallY- Variance   : " << variance << endl;
			calcTargetY = ballPositions->getLast(0)->y;
			//cout << "[WatchBall] -BallY- calcTargetY: " << calcTargetY << endl;
		}
		return calcTargetY;
	}

    double WatchBall::fitTargetY(double targetY)
    {

        if (targetY > alloGoalLeft->y)
        {
//			cout << "[WatchBall] fitTarget left: " << alloGoalLeft->y << endl;
            return alloGoalLeft->y;
        }
        else if (targetY < alloGoalRight->y)
        {
//			cout << "[WatchBall] fitTarget right: " << alloGoalRight->y << endl;
            return alloGoalRight->y;
        }
        else
        {
//			cout << "[WatchBall] fitTarget else: " << targetY << endl;
            return targetY;
        }
    }

    void WatchBall::rotate(shared_ptr<geometry::CNPoint2D> alloTarget)
    {
        shared_ptr < geometry::CNPoint2D > alignPoint;
        double ballAngle = ownPos->getPoint()->angleToPoint(alloTarget) / M_PI * 180;

        if (alignTowardsBall == true)
        {
            double radRotLim = rotationLimit * M_PI / 180.0;
            // todo: only allow smaller rotationLimit when close to posts?
            if (ballAngle <= -rotationLimit)
            {
                alignPoint = make_shared < geometry::CNPoint2D
                        > (ownPos->x + 1000, -((tan(radRotLim) * 1000) - ownPos->y));
            }
            else if (ballAngle >= rotationLimit)
            {
                alignPoint = make_shared < geometry::CNPoint2D
                        > (ownPos->x + 1000, (tan(radRotLim) * 1000) + ownPos->y);
            }
            else
            {
                alignPoint = alloTarget;
            }
        }
        else
        {
            alignPoint = alloFieldCntr;
        }

        double angleErr = alignPoint->alloToEgo(*ownPos)->rotate(M_PI)->angleTo();
        mc.motion.rotation = pRot * angleErr + dRot * geometry::normalizeAngle(angleErr - lastRotErr);
        lastRotErr = angleErr;

//		double angleAlignPoint = ownPos->getPoint()->angleToPoint(alignPoint) / M_PI * 180;
//		cout << "[WatchBall] alignPAngle: " << angleAlignPoint << endl;
//		cout << "[WatchBall] ballAngle  : " << ballAngle << endl;
//		cout << "[WatchBall] rotationLim: " << rotationLimit << endl;
//		cout << "[WatchBall] alloBall   : " << alloBall->x << " " << alloBall->y << endl;
//		cout << "[WatchBall] alloAPoint : " << alignPoint->x << " " << alignPoint->y << endl;
//		cout << "[WatchBall] alloOwnPos : " << ownPos->x << " " << ownPos->y << endl;
//		cout << "[WatchBall] rotation   : " << mc.motion.rotation << endl;
//		cout << "[WatchBall] theta      :" << ownPos->theta / M_PI * 180 << endl << endl;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
