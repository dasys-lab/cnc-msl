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
		alloLeftAimPoint = nullptr;
		alloMidAimPoint = nullptr;
		alloRightAimPoint = nullptr;
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
								vNet, make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y),
								alloLeftAimPoint,
								make_shared<geometry::CNPoint2D>(obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPos)))
				{
					leftBlocked = true;
				}
				if (!midBlocked
						&& wm->pathPlanner.corridorCheckBall(
								vNet, make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y),
								alloMidAimPoint,
								make_shared<geometry::CNPoint2D>(obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPos)))
				{
					midBlocked = true;
				}
				if (!rightBlocked
						&& wm->pathPlanner.corridorCheckBall(
								vNet, make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y),
								alloRightAimPoint,
								make_shared<geometry::CNPoint2D>(obs->at(i).x, obs->at(i).y)->egoToAllo(*ownPos)))
				{
					rightBlocked = true;
				}

			}
			if (!leftBlocked && aimPoint == nullptr)
			{
				aimPoint = alloLeftAimPoint->alloToEgo(*ownPos);
			}
			if (!midBlocked && aimPoint == nullptr)
			{
				aimPoint = alloMidAimPoint->alloToEgo(*ownPos);
			}
			if (!rightBlocked && aimPoint == nullptr)
			{
				aimPoint = alloRightAimPoint->alloToEgo(*ownPos);
			}
			if (leftBlocked && midBlocked && rightBlocked && aimPoint == nullptr)
			{
				this->failure = true;
			}
		}
		if (aimPoint != nullptr)
		{
			msl_actuator_msgs::MotionControl mc = msl::RobotMovement::alignToPointWithBall(aimPoint, egoBallPos,
																							this->angleTolerance,
																							this->angleTolerance);

			if (fabs(geometry::GeometryCalculator::deltaAngle(aimPoint->angleTo(), M_PI)) > this->angleTolerance)
			{
				cout << "turning" << endl;
				send(mc);
			}
			else
			{
				cout << "kicking" << endl;
				msl_actuator_msgs::KickControl kc;
				kc.enabled = true;
				kc.kicker = egoBallPos->angleTo();
				kc.power = min(minKickPower, aimPoint->length());
				send(kc);
				this->success = true;
			}
		}
		/*PROTECTED REGION END*/
	}
	void GoalKick::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1415205565589) ENABLED START*/ //Add additional options here
		field = msl::MSLFootballField::getInstance();
		alloLeftAimPoint = make_shared<geometry::CNPoint2D>(
				field->FieldLength / 2 , field->posLeftOppGoalPost()->y - wm->ball.getBallDiameter() * 1.5);
		alloMidAimPoint = make_shared<geometry::CNPoint2D>(field->FieldLength  / 2 , 0);
		alloRightAimPoint = make_shared<geometry::CNPoint2D>(
				field->FieldLength / 2, field->posRightOppGoalPost()->y + wm->ball.getBallDiameter() * 1.5);
		aimPoint = nullptr;
		angleTolerance = 0.05;
		minKickPower = 1500.0;
		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1415205565589) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
