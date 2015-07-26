using namespace std;
#include "Plans/Behaviours/GoalKick.h"

/*PROTECTED REGION ID(inccpp1415205565589) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1415205565589) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	GoalKick::GoalKick() :
			DomainBehaviour("GoalKick")
	{
		/*PROTECTED REGION ID(con1415205565589) ENABLED START*/ //Add additional options here
		field = nullptr;
		leftAimPoint = nullptr;
		midAimPoint = nullptr;
		rightAimPoint = nullptr;
		aimPoint = nullptr;
		angleTolerance = 0.05;
		minKickPower = 1500.0;
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
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball.getEgoBallPosition();
		auto vNet = wm->pathPlanner.getCurrentVoronoiNet();

		if (ownPos == nullptr || egoBallPos == nullptr || vNet == nullptr)
		{
			return;
		}

		msl_actuator_msgs::BallHandleCmd bhc;
		bhc.leftMotor = (int8_t)-30;
		bhc.rightMotor = (int8_t)-30;
		send(bhc);

		if (aimPoint == nullptr)
		{
			auto obs = wm->robots.getObstacles();
			bool leftBlocked = false;
			bool midBlocked = false;
			bool rightBlocked = false;
			for (int i = 0; i < obs->size(); i++)
			{
				if (leftBlocked && midBlocked && rightBlocked)
				{
					break;
				}
				if (!leftBlocked
						&& wm->pathPlanner.corridorCheckBall(
								vNet, make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y), leftAimPoint,
								make_shared<geometry::CNPoint2D>(obs->at(i).x, obs->at(i).y)))
				{
					leftBlocked = true;
				}
				if (!midBlocked
						&& wm->pathPlanner.corridorCheckBall(
								vNet, make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y), midAimPoint,
								make_shared<geometry::CNPoint2D>(obs->at(i).x, obs->at(i).y)))
				{
					midBlocked = true;
				}
				if (!rightBlocked
						&& wm->pathPlanner.corridorCheckBall(
								vNet, make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y), rightAimPoint,
								make_shared<geometry::CNPoint2D>(obs->at(i).x, obs->at(i).y)))
				{
					rightBlocked = true;
				}

			}
			if (!leftBlocked && aimPoint == nullptr)
			{
				aimPoint = leftAimPoint;
			}
			if (!midBlocked && aimPoint == nullptr)
			{
				aimPoint = midAimPoint;
			}
			if (!rightBlocked && aimPoint == nullptr)
			{
				aimPoint = rightAimPoint;
			}
			if (leftBlocked && midBlocked && rightBlocked && aimPoint == nullptr)
			{
				aimPoint = leftAimPoint;
			}
		}
		msl_actuator_msgs::MotionControl mc = msl::RobotMovement::rapidAlignToPointWithBall(aimPoint, egoBallPos,
																							this->angleTolerance,
																							this->angleTolerance);
		if (fabs(geometry::GeometryCalculator::deltaAngle(aimPoint->angleTo(), M_PI)) < this->angleTolerance)
		{
			send(mc);
		}
		else
		{
			msl_actuator_msgs::KickControl kc;
			kc.enabled = true;
			kc.kicker = egoBallPos->angleTo();
			kc.power = min(minKickPower, aimPoint->length());
			send(kc);
			this->success = true;
		}
		/*PROTECTED REGION END*/
	}
	void GoalKick::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1415205565589) ENABLED START*/ //Add additional options here
		field = msl::MSLFootballField::getInstance();
		leftAimPoint = make_shared<geometry::CNPoint2D>(
				field->FieldLength + 250, field->posLeftOppGoalPost()->y - wm->ball.getBallDiameter() * 1.5);
		midAimPoint = make_shared<geometry::CNPoint2D>(field->FieldLength + 250, 0);
		rightAimPoint = make_shared<geometry::CNPoint2D>(
				field->FieldLength + 250, field->posRightOppGoalPost()->y + wm->ball.getBallDiameter() * 1.5);
		aimPoint = nullptr;
		angleTolerance = 0.05;
		minKickPower = 1500.0;
		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1415205565589) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
