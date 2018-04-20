using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

/*PROTECTED REGION ID(inccpp1447863466691) ENABLED START*/ // Add additional includes here
#include <Ball.h>
#include <RawSensorData.h>
#include <cmath>
#include <obstaclehandler/Obstacles.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <vector>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1447863466691) ENABLED START*/
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
WatchBall::WatchBall()
    : DomainBehaviour("WatchBall")
{
    /*PROTECTED REGION ID(con1447863466691) ENABLED START*/ // Add additional options here
    goalieSize = (*this->sc)["Behaviour"]->get<int>("Goalie.GoalieSize", NULL);
    snapDistance = (*this->sc)["Behaviour"]->get<int>("Goalie.SnapDistance", NULL);
    maxVariance = (*this->sc)["Behaviour"]->get<int>("Goalie.MaxVariance", NULL);

     query = make_shared<msl::MovementQuery>();

    const auto nrOfPositions = (*this->sc)["Behaviour"]->get<int>("Goalie.NrOfPositions", NULL);
    ballPositions = new RingBuffer<geometry::CNPoint2D>(nrOfPositions);

    const auto tempMid = wm->field->posOwnGoalMid();
    alloGoalMid = make_shared<geometry::CNPoint2D>(tempMid->x, tempMid->y);
    alloGoalLeft = make_shared<geometry::CNPoint2D>(alloGoalMid->x, wm->field->posLeftOwnGoalPost()->y - goalieSize / 2);
    alloGoalRight = make_shared<geometry::CNPoint2D>(alloGoalMid->x, wm->field->posRightOwnGoalPost()->y + goalieSize / 2);
    /*PROTECTED REGION END*/
}
WatchBall::~WatchBall()
{
    /*PROTECTED REGION ID(dcon1447863466691) ENABLED START*/ // Add additional options here
    delete ballPositions;
    /*PROTECTED REGION END*/
}
void WatchBall::run(void *msg)
{
    /*PROTECTED REGION ID(run1447863466691) ENABLED START*/ // Add additional options here
    ownPos = wm->rawSensorData->getOwnPositionVision();
    msl_actuator_msgs::MotionControl mc;

    // Stop Robot if own position is unknown
    if (ownPos == nullptr)
    {
        mc.motion.translation = 0;
        mc.motion.angle = 0;
        mc.motion.rotation = 0;
        send(mc);
        cout << "[WatchBall]: ownPos is null" << endl;
        return;
    }

    // TODO: Fix and uncomment
    // updateGoalPosition();

	// Special cases that depend on the ball position are following:
    shared_ptr<geometry::CNPoint2D> alloBall = wm->ball->getAlloBallPosition();

    // If ball is not seen or the ball is further away than the goal mid point is.
    // TODO: Keep?
    if (alloBall == nullptr || abs(alloBall->x) > abs(alloGoalMid->x) + 50)
    {
        // Goalie drives to last known target and rotates towards mirrored own position

        cout << "[WatchBall]: Goalie can't see ball! Moving to GoalMid" << endl;

	query->egoDestinationPoint = alloGoalMid->alloToEgo(*ownPos);
		if (alloBall != nullptr) {
			query->egoAlignPoint = alloBall->alloToEgo(*ownPos);
		}

        mc = robot->robotMovement->moveToPoint(query);

        send(mc);
        return;
    }

	// If ball is seen in the opponent half, stop the robot but rotate towards the opponents half.
    // TODO: Keep. What would be the alternative?
    if (alloBall != nullptr && alloBall->x > -250)
    {
        mc.motion.translation = 0;
        shared_ptr<geometry::CNPoint2D> alignPoint = make_shared<geometry::CNPoint2D>(-ownPos->x, ownPos->y); // align to mirrored ownPos
        mc.motion.rotation = alignPoint->alloToEgo(*ownPos)->rotate(M_PI)->angleTo();
        send(mc);
        cout << "[WatchBall] ball is far away from goal! BallX: " << alloBall->x << endl;
        return;
    }

	// From now on the balls postion alloBall is valid and in our half, so act accordingly.
    // Add ball position to ring buffer.
    this->ballPositions->add(alloBall);

    // Calculate target position on goal line.
	
	// Lambda that returns one prediction of the goal impact or nullptr if
	// a prediction is not possible.
	auto calculateTarget = [&]() -> shared_ptr<geometry::CNPoint2D> {
		// Not enough valid ball positions to predict goal impact
		if (ballPositions->getSize() < 0) {
			return nullptr;
		}

		double targetY = calcGoalImpactY(); // TODO: Parameterize
		// Limit or clamp targetY to goal area
		targetY = fitTargetY(targetY);
		const double targetX = alloGoalMid->x + 200;
		std::cout << "targetY: " << targetY << std::endl;

	        return make_shared<geometry::CNPoint2D>(targetX, targetY);
	};

	auto alloTarget = calculateTarget();
	// If alloTarget was not calculated, stop robot?
	// TODO: Eventually return prevTarget or alloGoalMid to be ready when ball is coming.
	if (alloTarget == nullptr) {
		alloTarget = alloGoalMid;
	}

	// Finaly if a goal impact can be calculated drive to the calculated impact:
    // TODO: Think about replacing the code with existing drive config.
	auto egoBall = alloBall->alloToEgo(*ownPos);
    auto egoTarget = alloTarget->alloToEgo(*ownPos);

	// Stop translation if already at target.
	if (egoTarget->length() <= snapDistance) {
		mc.motion.translation = 0;
		send(mc);
		return;
	}

	query->egoDestinationPoint = egoTarget;
	query->egoAlignPoint = egoBall;
	mc = robot->robotMovement->moveToPoint(query);

	// Add goal posts as obstacles
	auto additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
	additionalPoints->push_back(alloGoalLeft);
	additionalPoints->push_back(alloGoalRight);
	query->additionalPoints = additionalPoints;
	// TODO: Test if good
	query->blockOwnPenaltyArea = true;

	mc.motion.translation *= 2; // Foxy, move faster!
	// TODO: Probably remove as soon as motion is fixed
	// Clamp translation because of motion failure
	if (mc.motion.translation > 1500) {
		mc.motion.translation = 1500;
	}

    send(mc);

    /*PROTECTED REGION END*/
}
void WatchBall::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ // Add additional options here
    prevTarget = wm->field->posOwnGoalMid();
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ // Add additional methods here
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
    double variance = (sumX2Y2 + nPoints * ((avgBall->x * avgBall->x) + (avgBall->y * avgBall->y)) - 2 * ((avgBall->x * sumX) + (avgBall->y * sumY))) / nPoints;

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
        // cout << "[WatchBall] -LinearRegression- Variance   : " << variance << endl;
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
        cout << "[WatchBall] -BallY- Variance   : " << variance << endl;
        calcTargetY = ballPositions->getLast(0)->y;
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

void WatchBall::updateGoalPosition()
{
    shared_ptr<geometry::CNPoint2D> laserDetectedEgoGoalMid = wm->rawSensorData->getEgoGoalMid();

    if (laserDetectedEgoGoalMid)
    {
        alloGoalMid = laserDetectedEgoGoalMid;
    }
    else
    {
        alloGoalMid = wm->field->posOwnGoalMid();
    }
    if (alloGoalMid == nullptr || wm->field->posLeftOwnGoalPost() == nullptr)  {
        cout << "Can't determine goal mid using scanner, alloGoalMid == nullptr" << endl;
        return;
    }
    alloGoalLeft = make_shared<geometry::CNPoint2D>(alloGoalMid->x, wm->field->posLeftOwnGoalPost()->y - goalieSize / 2);
    alloGoalRight = make_shared<geometry::CNPoint2D>(alloGoalMid->x, wm->field->posRightOwnGoalPost()->y + goalieSize / 2);
}
/*PROTECTED REGION END*/
} /* namespace alica */
