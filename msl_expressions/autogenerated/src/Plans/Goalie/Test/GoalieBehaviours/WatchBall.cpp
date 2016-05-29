using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

/*PROTECTED REGION ID(inccpp1447863466691) ENABLED START*/ //Add additional includes here
#include <cmath>
#include <vector>
#include <SystemConfig.h>
#include <MSLWorldModel.h>
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
    	this->alignTowardsBall = (*this->sc)["Behaviour"]->get<bool>("Goalie.AlignTowardsBall", NULL);
        this->rotationLimit = (*this->sc)["Behaviour"]->get<double>("Goalie.RotationLimit", NULL);
        this->maxVariance = (*this->sc)["Behaviour"]->get<int>("Goalie.MaxVariance", NULL);
        this->goalieSize = (*this->sc)["Behaviour"]->get<int>("Goalie.GoalieSize", NULL);
        this->nrOfPositions = (*this->sc)["Behaviour"]->get<int>("Goalie.NrOfPositions", NULL);

        this->pTrans = (*this->sc)["Behaviour"]->get<double>("Goalie.pTrans", NULL);
        this->dTrans = (*this->sc)["Behaviour"]->get<double>("Goalie.dTrans", NULL);
        this->pRot = (*this->sc)["Behaviour"]->get<double>("Goalie.pRot", NULL);
        this->dRot = (*this->sc)["Behaviour"]->get<double>("Goalie.dRot", NULL);
        this->lastRotErr = 0;
        this->prevTargetDist = 0;

        this->snapDistance = (*this->sc)["Behaviour"]->get<int>("Goalie.SnapDistance", NULL);
        this->alignMaxVel = (*sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
        this->ballPositions = new RingBuffer<geometry::CNPoint2D>(nrOfPositions);
        this->alloFieldCntr = wm->field->posCenterMarker();
        this->alloAlignPt = alloFieldCntr;
        auto tempMid = alloGoalMid = wm->field->posOwnGoalMid();
        this->alloGoalMid = make_shared < geometry::CNPoint2D > (tempMid->x, tempMid->y);
        this->alloGoalLeft = make_shared < geometry::CNPoint2D
                > (alloGoalMid->x, wm->field->posLeftOwnGoalPost()->y - goalieSize / 2);
        this->alloGoalRight = make_shared < geometry::CNPoint2D
                > (alloGoalMid->x, wm->field->posRightOwnGoalPost()->y + goalieSize / 2);
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
            return;
        }

        shared_ptr < geometry::CNPoint2D > alloBall = wm->ball->getAlloBallPosition();
        if (alloBall == nullptr || abs(alloBall->x) > abs(alloGoalMid->x) + 50)
        {

            /*
             * Goalie drives to last known target and rotates towards mirrored own position
             */
            mc.motion.angle = prevTarget->alloToEgo(*ownPos)->angleTo();
            rotate(make_shared < geometry::CNPoint2D > (-ownPos->x, ownPos->y));
            mc.motion.translation = std::min(
                    alignMaxVel,
                    (prevTarget->alloToEgo(*ownPos)->length() * pTrans)
                            + ((prevTarget->alloToEgo(*ownPos)->length() - prevTargetDist) * dTrans));
            send (mc);
            return;
        }

        this->ballPositions->add(alloBall);

        /*
         * Calculate target position on goal line
         */

        shared_ptr < geometry::CNPoint2D > alloTarget;
        double targetY;
        if (ballPositions->getSize() > 0)
        {
            targetY = calcGoalImpactY();
            targetY = fitTargetY(targetY);
            alloTarget = make_shared < geometry::CNPoint2D > (alloGoalMid->x, targetY);
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
        rotate (alloBall);

        if (egoTarget->length() > snapDistance)
        {
            auto tempPFactor = pTrans;
            if (egoBall != nullptr && egoBall->egoToAllo(*ownPos) != nullptr
                    && egoBall->egoToAllo(*ownPos)->x > alloFieldCntr->x + 1000)
            {
                tempPFactor = pTrans / 2;
            }
            else
            {
                tempPFactor = pTrans;
            }
            mc.motion.translation = std::min(
                    alignMaxVel,
                    (egoTarget->length() * tempPFactor) + ((egoTarget->length() - prevTargetDist) * dTrans));
        }
        else
        {
            mc.motion.translation = 0;
        }

        send (mc);
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
        shared_ptr < geometry::CNPoint2D > avgBall = make_shared < geometry::CNPoint2D > (0.0, 0.0);
        int nPoints = 0;
        for (int i = 0; i < ballPositions->getSize(); i++)
        {
            auto currentBall = ballPositions->getLast(i);
            // p=prev, pp=prePrev, ppp=prePrePrev
            shared_ptr < geometry::CNPoint2D > ppprevBall;
            shared_ptr < geometry::CNPoint2D > pprevBall;
            shared_ptr < geometry::CNPoint2D > prevBall;
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
                        break;
                    }
                }
                else
                {
                    if (!(diffpp <= diffp || diffpp + buffer <= diffp || diffpp - buffer <= diffp))
                    {
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
        if (nPoints > 1 && variance > maxVariance)
        {
            for (int i = 0; i < nPoints; i++)
            {
                auto curBall = ballPositions->getLast(i);
                nomi = nomi + ((curBall->x - avgBall->x) * (curBall->y - avgBall->y));
                denom = denom + ((curBall->x - avgBall->x) * (curBall->x - avgBall->x));
            }
            if (denom < 1e-3)
            {
                return prevTarget->y;
            }
            _slope = nomi / denom;
            _yInt = avgBall->y - _slope * avgBall->x;
            calcTargetY = _slope * alloGoalMid->x + _yInt;
        }
        else
        {
            // TODO: use this when Goalie Vision detects Obstacles better?!
            auto obstacles = wm->obstacles->getAlloObstaclePoints();
            shared_ptr < geometry::CNPoint2D > closestObstacle; // = make_shared<geometry::CNPoint2D>(0.0, 0.0);
            double minDistBallObs = 20000;
            for (auto currentObs : *obstacles)
            {
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
                _slope = (closestObstacle->y - ballPositions->getLast(0)->y)
                        / (closestObstacle->x - ballPositions->getLast(0)->x);
                _yInt = ballPositions->getLast(0)->y - _slope * ballPositions->getLast(0)->x;
                calcTargetY = _slope * alloGoalMid->x + _yInt;
            }
            else
            {
                calcTargetY = ballPositions->getLast(0)->y;
            }
        }
        return calcTargetY;
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
    }
/*PROTECTED REGION END*/
} /* namespace alica */
