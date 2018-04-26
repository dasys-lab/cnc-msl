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

	// Read in config values.
    goalieSize = (*this->sc)["Behaviour"]->get<int>("Goalie.GoalieSize", NULL);
    snapDistance = (*this->sc)["Behaviour"]->get<int>("Goalie.SnapDistance", NULL);
    ballMovingThreshold = (*this->sc)["Behaviour"]->get<double>("Goalie.BallMovingThreshold", NULL);
    const auto maxRotationDeg = (*this->sc)["Behaviour"]->tryGet<double>(45.0, "Goalie.MaxRotation", NULL);
    maxRotationRad = maxRotationDeg * (M_PI/180.0);

	// Read required goal dimensions.
    goalWidth = this->wm->field->getGoalWidth();
    const auto tempMid = wm->field->posOwnGoalMid();
    alloGoalMid = make_shared<geometry::CNPoint2D>(tempMid->x, tempMid->y);
    alloGoalLeft = make_shared<geometry::CNPoint2D>(alloGoalMid->x, wm->field->posLeftOwnGoalPost()->y - goalieSize / 2);
    alloGoalRight = make_shared<geometry::CNPoint2D>(alloGoalMid->x, wm->field->posRightOwnGoalPost()->y + goalieSize / 2);

	// Create query for movement.
    query = make_shared<msl::MovementQuery>();
    /*PROTECTED REGION END*/
}
WatchBall::~WatchBall()
{
    /*PROTECTED REGION ID(dcon1447863466691) ENABLED START*/ // Add additional options here
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

    updateGoalPosition();

	// Special cases that depend on the ball position are following:
    alloBall = wm->ball->getAlloBallPosition();

    // If ball is not seen or the ball is further away than the goal mid point is.
    if (alloBall == nullptr // Ball is not seen
		|| abs(alloBall->x) > abs(alloGoalMid->x) + 50 // Ball is behind goal
		|| alloBall->x > -250) // Ball is in opponent half
    {
        // Goalie drives to last known target and rotates towards mirrored own position
        cout << "[WatchBall]: Special case: Moving to GoalMid" << endl;
        mc = driveAndAlignTo(alloGoalMid, mirroredOwnPos());
        send(mc);
        return;
    }

    // Calculate target position on goal line.
	
	// Lambda that returns one prediction of the goal impact or nullptr if
	// a prediction is not possible.
	auto calculateTarget = [&]() -> shared_ptr<geometry::CNPoint2D> {
		double targetY = 0;

		if (ballIsMoving()) {
			targetY = calcGoalImpactY();
		} else {
			// if ball is not moving, do stuff
			targetY = alloBall->y;
		}

		// Limit or clamp targetY to goal area
		targetY = fitTargetY(targetY);
		const double targetX = alloGoalMid->x + 200;
		if (targetY != 0) {
			std::cout << "targetY: " << targetY << std::endl;
		}

		return make_shared<geometry::CNPoint2D>(targetX, targetY);
	};


	auto alloTarget = calculateTarget();

	// If alloTarget was not calculated, move to the goal mid.
	if (alloTarget == nullptr) {
		alloTarget = alloGoalMid;
	}

	auto offset = std::make_shared<geometry::CNPoint2D>(200, 0);

	// Finaly if a goal impact can be calculated drive to the calculated impact
	mc = faster(driveAndAlignTo(alloTarget + offset, alloBall));

	// Clamp rotation to 45°
	mc.motion.rotation = clampRotation(
			mc.motion.translation, ownPos->theta, maxRotationRad);

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
	// Retrieve ball velocity vector
	auto ballVec = wm->ball->getEgoBallVelocity();
	if (ballVec == nullptr) {
		return 0; // TODO: Indicate error
	}

	auto ballSpeed = ballVec->length();

	// Calculation itself
	const auto bvx = -ballVec->x; // ball velocity x
	const auto bvy = -ballVec->y;
	const auto bpx = alloBall->x; // ball position x
	const auto bpy = alloBall->y;
	const auto glx = alloGoalMid->x;

	if (bvx >= 0) {
		puts("Ball not moving to our direction");
		return 0;
	}

	const auto y = bpy + ((glx-bpx)/bvx) * bvy;

	const double threshold = 1000.0;

	static int c = 0;
	if (c++ == 5) {
		printf("ballVec: %f %f, ballSpeed: %f, y: %f\n",
				ballVec->x, ballVec->y, ballSpeed, y);
		c = 0;
		if (ballSpeed >= threshold) {
			puts("Ball IS moving!");
		} else {
			puts("Ball NOT moving!");
		}
	}

	if (ballSpeed < threshold)
	{
		return 0;
	}


	return y;
}

bool WatchBall::ballIsMoving()
{
	// Retrieve ball velocity vector
	auto ballVec = wm->ball->getEgoBallVelocity();
	if (ballVec == nullptr) {
		return 0; // TODO: Indicate error
	}
	auto ballSpeed = ballVec->length();
	return ballSpeed >= ballMovingThreshold;
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

    if (!laserDetectedEgoGoalMid)
    {
        alloGoalMid = wm->field->posOwnGoalMid();
    	return;
    }

    alloGoalMid = laserDetectedEgoGoalMid;

    if (alloGoalMid == nullptr || wm->field->posLeftOwnGoalPost() == nullptr)  {
        cout << "Can't determine goal mid using scanner, alloGoalMid == nullptr" << endl;
        return;
    }

    alloGoalLeft = make_shared<geometry::CNPoint2D>(
    		alloGoalMid->x, wm->field->posLeftOwnGoalPost()->y - goalieSize / 2);
    alloGoalRight = make_shared<geometry::CNPoint2D>(
    		alloGoalMid->x, wm->field->posRightOwnGoalPost()->y + goalieSize / 2);
}

double WatchBall::clampRotation(double mcRotation, double ownTheta, double maxRot)
{
	if (ownTheta > maxRot && mcRotation > 0) {
		return 0;
	}

	if (ownTheta < -(maxRot) && mcRotation < 0) {
		return 0;
	}

	return mcRotation;
}

msl_actuator_msgs::MotionControl WatchBall::driveAndAlignTo(
	shared_ptr<geometry::CNPoint2D> targetAllo,
	shared_ptr<geometry::CNPoint2D> alloAlginPoint
	)
{
	msl_actuator_msgs::MotionControl mc;

	auto egoAlignPoint = alloAlginPoint->alloToEgo(*ownPos);
	auto egoTarget = targetAllo->alloToEgo(*ownPos);

	// Build Query
	query->egoDestinationPoint = egoTarget;
	query->egoAlignPoint = egoAlignPoint;
	query->snapDistance = snapDistance; // TODO: Test if rotation still works
	// Add goal posts as obstacles
	auto additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
	additionalPoints->push_back(alloGoalLeft);
	additionalPoints->push_back(alloGoalRight);
	query->additionalPoints = additionalPoints;
	// TODO: Test if good
	query->blockOwnPenaltyArea = true;
	mc = robot->robotMovement->moveToPoint(query);

	return mc;
}

MotionControl WatchBall::faster(MotionControl in)
{
	in.motion.translation *= 2; // Foxy, move faster!
	// TODO: Probably remove as soon as motion is fixed
	// Clamp translation because of motion failure
	if (in.motion.translation > 1500) {
		in.motion.translation = 1500;
	}
	return in;
}

shared_ptr<geometry::CNPoint2D> WatchBall::mirroredOwnPos() {
	auto mirrored = std::make_shared<geometry::CNPoint2D>(
			ownPos->x+1000, ownPos->y);
	return mirrored;
}

/*PROTECTED REGION END*/
} /* namespace alica */
