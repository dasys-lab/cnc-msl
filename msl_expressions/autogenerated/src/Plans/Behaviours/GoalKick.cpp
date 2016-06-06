using namespace std;
#include "Plans/Behaviours/GoalKick.h"

/*PROTECTED REGION ID(inccpp1415205565589) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "robotmovement/RobotMovement.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <obstaclehandler/Obstacles.h>
#include <pathplanner/PathPlanner.h>
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1415205565589) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	GoalKick::GoalKick() :
			DomainBehaviour("GoalKick")
	{
		/*PROTECTED REGION ID(con1415205565589) ENABLED START*/ //Add additional options here
		alloLeftAimPoint = nullptr;
		alloMidAimPoint = nullptr;
		alloRightAimPoint = nullptr;
		alloAimPoint = nullptr;
		angleTolerance = 0.05;
		minKickPower = 1500.0;
		alignMaxVel = (*sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
		alignToPointRapidMaxRotation = 2 * M_PI;
		lastRotErrorWithBallRapid = 0;
		/*PROTECTED REGION END*/
	}
	GoalKick::~GoalKick()
	{
		/*PROTECTED REGION ID(dcon1415205565589) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void GoalKick::run(void* msg)
	{
		/*PROTECTED REGION ID(run1415205565589) ENABLED START*/ //Add additional options here
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball->getEgoBallPosition();

		if (ownPos == nullptr || egoBallPos == nullptr)
		{
			return;
		}

		msl_actuator_msgs::BallHandleCmd bhc;
		bhc.leftMotor = (int8_t)-70;
		bhc.rightMotor = (int8_t)-70;
		send(bhc);

		alloAimPoint = nullptr;

		auto obs = wm->obstacles->getEgoVisionObstacles();
		bool leftBlocked = false;
		bool midBlocked = false;
		bool rightBlocked = false;
		for (int i = 0; i < obs->size(); i++)
		{
			if (leftBlocked && midBlocked && rightBlocked)
			{
				break;
			}
			if (wm->pathPlanner->corridorCheckBall(
					make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y), alloLeftAimPoint,
					make_shared<geometry::CNPoint2D>(obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPos)))
			{
				leftBlocked = true;
			}
			if (wm->pathPlanner->corridorCheckBall(
					make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y), alloMidAimPoint,
					make_shared<geometry::CNPoint2D>(obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPos)))
			{
				midBlocked = true;
			}
			if (wm->pathPlanner->corridorCheckBall(
					make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y), alloRightAimPoint,
					make_shared<geometry::CNPoint2D>(obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPos)))
			{
				rightBlocked = true;
			}

		}
		if (!leftBlocked && alloAimPoint == nullptr)
		{
			cout << "aiming left" << endl;
			alloAimPoint = alloLeftAimPoint;
		}
		if (!midBlocked && alloAimPoint == nullptr)
		{
			cout << "aiming mid" << endl;
			alloAimPoint = alloMidAimPoint;
		}
		if (!rightBlocked && alloAimPoint == nullptr)
		{
			cout << "aiming right" << endl;
			alloAimPoint = alloRightAimPoint;
		}
		if (leftBlocked && midBlocked && rightBlocked && alloAimPoint == nullptr)
		{
			this->setFailure(true);
		}
		if (alloAimPoint != nullptr)
		{
			auto egoAimPoint = alloAimPoint->alloToEgo(*ownPos);
			msl_actuator_msgs::MotionControl mc = rapidAlignToPointWithBall(
					egoAimPoint->rotate(M_PI), egoBallPos, this->angleTolerance, this->angleTolerance);

			if (fabs(geometry::deltaAngle(egoAimPoint->angleTo(), M_PI)) > this->angleTolerance)
			{
				cout << "angle: " << fabs(geometry::deltaAngle(egoAimPoint->angleTo(), M_PI)) << endl;
				send(mc);
			}
			else
			{
				cout << "kicking" << endl;
				msl_actuator_msgs::KickControl kc;
				kc.enabled = true;
				kc.kicker = egoBallPos->angleTo();
				kc.power = min(minKickPower, egoAimPoint->length());
				send(kc);
				this->setSuccess(true);
			}
		}
		/*PROTECTED REGION END*/
	}
	void GoalKick::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1415205565589) ENABLED START*/ //Add additional options here
		alloLeftAimPoint = make_shared<geometry::CNPoint2D>(
				wm->field->getFieldLength() / 2 + 250,
				wm->field->posLeftOppGoalPost()->y - wm->ball->getBallDiameter() * 1.5);
		alloMidAimPoint = make_shared<geometry::CNPoint2D>(wm->field->getFieldLength() / 2 + 250, 0);
		alloRightAimPoint = make_shared<geometry::CNPoint2D>(
				wm->field->getFieldLength() / 2 + 250,
				wm->field->posRightOppGoalPost()->y + wm->ball->getBallDiameter() * 1.5);
		alloAimPoint = nullptr;
		angleTolerance = 0.075;
		minKickPower = 1500.0;
		/*PROTECTED REGION END*/
	}

	// this method was copied from the old RobotMovement
	msl_actuator_msgs::MotionControl GoalKick::rapidAlignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
																			shared_ptr<geometry::CNPoint2D> egoBallPos,
																			double angleTolerance,
																			double ballAngleTolerance)
	{
		msl_actuator_msgs::MotionControl mc;
		double egoTargetAngle = egoAlignPoint->angleTo();
		double egoBallAngle = egoBallPos->angleTo();
		double deltaTargetAngle = geometry::deltaAngle(egoTargetAngle, M_PI);
		double deltaBallAngle = geometry::deltaAngle(egoBallAngle, M_PI);

		if (fabs(deltaBallAngle) < ballAngleTolerance && fabs(deltaTargetAngle) < angleTolerance)
		{
			mc.motion.angle = 0;
			mc.motion.rotation = 0;
			mc.motion.translation = 0;
		}
		else
		{
			if (egoAlignPoint->angleTo() > M_PI / 2)
			{
				mc.motion.rotation = alignToPointRapidMaxRotation;
			}
			else if (egoAlignPoint->angleTo() < -M_PI / 2)
			{
				mc.motion.rotation = -alignToPointRapidMaxRotation;
			}
			else
			{
				double clausenValue = 0.0;
				for (int i = 1; i < 10; i++)
				{
					clausenValue += sin(i * egoAlignPoint->angleTo()) / pow(i, 2);
				}
				mc.motion.rotation = egoAlignPoint->angleTo() * abs(clausenValue) * 8;

			}
			//			mc.motion.rotation = -(deltaTargetAngle * defaultRotateP
			//					+ (deltaTargetAngle - lastRotError) * alignToPointpRot);
			//			mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1)
			//					* min(alignToPointMaxRotation, max(fabs(mc.motion.rotation), alignToPointMinRotation));

			lastRotErrorWithBallRapid = deltaTargetAngle;

			// crate the motion orthogonal to the ball
			shared_ptr<geometry::CNPoint2D> driveTo = egoBallPos->rotate(-M_PI / 2.0);
			driveTo = driveTo * mc.motion.rotation;

			// add the motion towards the ball
			driveTo = driveTo + egoBallPos->normalize() * 10;

			mc.motion.angle = driveTo->angleTo();
			mc.motion.translation = min(alignMaxVel, driveTo->length());
		}
		return mc;
	}
/*PROTECTED REGION ID(methods1415205565589) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
