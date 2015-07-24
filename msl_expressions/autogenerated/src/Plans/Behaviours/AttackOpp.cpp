using namespace std;
#include "Plans/Behaviours/AttackOpp.h"

/*PROTECTED REGION ID(inccpp1430324527403) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <cmath>
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1430324527403) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	AttackOpp::AttackOpp() :
			DomainBehaviour("AttackOpp")
	{
		/*PROTECTED REGION ID(con1430324527403) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	AttackOpp::~AttackOpp()
	{
		/*PROTECTED REGION ID(dcon1430324527403) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void AttackOpp::run(void* msg)
	{
		/*PROTECTED REGION ID(run1430324527403) ENABLED START*/

		shared_ptr<geometry::CNPosition> me = wm->rawSensorData.getOwnPositionVision();

		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball.getEgoBallPosition();

		auto vNet = wm->pathPlanner.getCurrentVoronoiNet();

		if (me == nullptr || egoBallPos == nullptr || vNet == nullptr)
		{
			return;
		}

		auto obstacles = wm->robots.getObstacles();
		bool blocked = false;
		msl_actuator_msgs::MotionControl mc;
		for (int i = 0; i < obstacles->size(); i++)
		{
			if (wm->pathPlanner.corridorCheck(
					vNet, make_shared<geometry::CNPoint2D>(me->x, me->y), egoBallPos->egoToAllo(*me),
					make_shared<geometry::CNPoint2D>(obstacles->at(i).x, obstacles->at(i).y)))
			{
				blocked = true;
				break;
			}
		}
		if (!blocked)
		{
			auto egoBallVelocity = wm->ball.getEgoBallVelocity();
			cout << "ego ball vel: " <<  egoBallVelocity << endl;
			auto vector = egoBallVelocity + egoBallPos;
			double vectorLength = vector->length();
			if (wm->ball.haveBall())
			{
				isMovingAwayIter = 0;
				isMovingCloserIter = 0;
			}
			else if (vectorLength < egoBallPos->length())
			{
				isMovingCloserIter++;
				isMovingAwayIter = 0;
			}
			else
			{
				isMovingAwayIter++;
				isMovingCloserIter = 0;
			}
			if (isMovingCloserIter >= maxIter)
			{
				cout << "get closer" << endl;
				mc = ballGetsCloser(me, egoBallVelocity, egoBallPos);

			}
			else if (isMovingAwayIter >= maxIter)
			{
				cout << "roll away" << endl;
				mc = driveToMovingBall(egoBallPos, egoBallVelocity);
			}
			else
			{
				mc.motion.angle = 0;
				mc.motion.translation = 0;
				mc.motion.rotation = 0;

			}
		}
		else
		{
			mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 0);
		}
		send(mc);

//Add additional options here
		/*PROTECTED REGION END*/
	}
	void AttackOpp::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1430324527403) ENABLED START*/ //Add additional options here
		oldDistance = 0.0;
		kP = 2.0;
		kI = 0.0;
		kD = 1.7;
		rotate_P = 1.8;
		isMovingCloserIter = 0;
		isMovingAwayIter = 0;
		maxIter = 3;
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1430324527403) ENABLED START*/ //Add additional methods here
	msl_actuator_msgs::MotionControl AttackOpp::driveToMovingBall(shared_ptr<geometry::CNPoint2D> egoBallPos,
																	shared_ptr<geometry::CNVelocity2D> egoBallVel)
	{

		msl_actuator_msgs::MotionControl mc;
		msl_actuator_msgs::BallHandleCmd bhc;

		mc.motion.angle = egoBallPos->angleTo();
		mc.motion.rotation = egoBallPos->rotate(M_PI)->angleTo() * rotate_P;

		double distance = egoBallPos->length();
		double movement = kP * distance + kD * (distance - oldDistance);
		oldDistance = distance;

		double ball_speed = egoBallVel->length();

		movement += ball_speed;
		mc.motion.translation = movement;

		if (egoBallPos->length() < 300)
		{

			bhc.leftMotor = -30;
			bhc.rightMotor = -30;

			this->send(bhc);
			//this->success = true;
		}
		return mc;
	}

	msl_actuator_msgs::MotionControl AttackOpp::ballGetsCloser(shared_ptr<geometry::CNPosition> robotPosition,
																shared_ptr<geometry::CNVelocity2D> ballVelocity,
																shared_ptr<geometry::CNPoint2D> egoBallPos)
	{
		double yIntersection = egoBallPos->y + (-(egoBallPos->x / ballVelocity->x)) * ballVelocity->y;

		shared_ptr<geometry::CNPoint2D> interPoint = make_shared<geometry::CNPoint2D>(0, yIntersection);

		msl_actuator_msgs::MotionControl mc;
		// TODO : remove later
		mc = RobotMovement::moveToPointCarefully(interPoint, egoBallPos, 300);

		return mc;
	}

/*PROTECTED REGION END*/
} /* namespace alica */
