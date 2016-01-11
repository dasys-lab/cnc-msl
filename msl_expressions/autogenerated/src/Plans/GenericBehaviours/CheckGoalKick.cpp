using namespace std;
#include "Plans/GenericBehaviours/CheckGoalKick.h"

/*PROTECTED REGION ID(inccpp1449076008755) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1449076008755) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	CheckGoalKick::CheckGoalKick() :
			DomainBehaviour("CheckGoalKick")
	{
		/*PROTECTED REGION ID(con1449076008755) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	CheckGoalKick::~CheckGoalKick()
	{
		/*PROTECTED REGION ID(dcon1449076008755) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void CheckGoalKick::run(void* msg)
	{
		/*PROTECTED REGION ID(run1449076008755) ENABLED START*/ //Add additional options here
		//get own Pos
		cout << "Start run CheckGoalKick <=============================================================" << endl;
		auto ownPosition = wm->rawSensorData.getOwnPositionVision();
		ownPos = make_shared<geometry::CNPoint2D>(ownPosition->x, ownPosition->y);
		//get ego ball pos
		egoBallPos = wm->ball.getEgoBallPosition();

		//get goal pos
		goalPosLeft = field->posLeftOppGoalPost();
		goalPosRight = field->posRightOppGoalPost();
		goalPosMiddle = field->posOppGoalMid();
//		cout << "das Glück ist eine Huuuure!!" << endl;

		readConfigParameters();
		toleranceAngle = calcToleranceAngle();
		cout << "toleranceAngle: " << toleranceAngle << " degree: "
				<< toleranceAngle * 180 / M_PI << endl;

		if (checkGoalLine() && checkShootPossibility())
		{
			cout << "kicking" << endl;
			msl_actuator_msgs::KickControl kc;
			kc.enabled = true;
			//angle
			kc.kicker = egoBallPos->angleTo();
			kc.power = kickPower;
//			send(kc);
			this->success = true;
		}

		/*PROTECTED REGION END*/
	}
	void CheckGoalKick::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1449076008755) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1449076008755) ENABLED START*/ //Add additional methods here
	/*
	 * @return true if angle to goal is smaller than tolerance angle
	 */
	bool CheckGoalKick::checkGoalLine()
	{
		// for testing!!! <=========================================
		// TODO remove later
//		return true;


//		cout << "angle to goal: " << ownPos->angleToPoint(goalPosMiddle) << " degree: "
//				<< ownPos->angleToPoint(goalPosMiddle) * 180 / M_PI << endl;
//
		cout << "goalPosMiddle: " << goalPosMiddle->toString();
		egoAlignPoint = goalPosMiddle;

		auto ownPos = wm->rawSensorData.getOwnPositionVision();
		auto egoTarget = goalPosMiddle->alloToEgo(*ownPos);

		cout << "egoTarget: " << egoTarget->toString();
		cout << "angle to goal: " << egoTarget->angleTo() << " degree: "
				<< egoTarget->angleTo() * 180 / M_PI << endl;

		cout << "if condition1: " << (egoTarget->angleTo() < M_PI * toleranceAngle) << endl;
		cout << "if condition2: " << (egoTarget->angleTo() < -M_PI * toleranceAngle) << endl;

		// if angle is smaller then tolerance angle return true
		if (egoTarget->angleTo() < toleranceAngle
				&& egoTarget->angleTo() > toleranceAngle)
		{
			cout << "ChackGoalLine = true" << endl;
			return true;
		}
		cout << "ChackGoalLine = false" << endl;
		return false;
	}

	/*
	 * checks if there is an obstacle between robot and goal.
	 *
	 * @return true if it is possible to shoot at the enemy goal
	 */
	bool CheckGoalKick::checkShootPossibility()
	{
		cout << "checkShootPossibility() ============================================" << endl;
		// check if obstacle lays in corridor
		auto obstacles = wm->robots.getObstacles();
		shared_ptr<geometry::CNPoint2D> obstaclePoint = make_shared<geometry::CNPoint2D>(0, 0);
		bool foundObstacle = false;
		int obstacleAt = -1;

		cout << "obstacle size = " << obstacles->size() << endl;
		for (int i = 0; i < obstacles->size(); i++)
		{
			obstaclePoint->x = obstacles->at(i).x;
			obstaclePoint->y = obstacles->at(i).y;

			if (wm->pathPlanner.corridorCheck(wm->pathPlanner.getCurrentVoronoiNet(), ownPos, goalPosMiddle,
												obstaclePoint))
			{
				foundObstacle = true;
				obstacleAt = i;
				break;
			}
		}

		cout << "found Obstacle = " << foundObstacle << endl;

		if (obstacleAt > -1)
		{
			// check if obstacle is blocking (Own distance -> obstacle and obstacle -> OppGoal)
			auto obstacle = obstacles->at(obstacleAt);
			shared_ptr<geometry::CNPoint2D> obstaclePos = make_shared<geometry::CNPoint2D>(0, 0);
			obstaclePos->x = obstacle.x;
			obstaclePos->y = obstacle.y;

			cout << "ownPos <----> obstaclePos = " << ownPos->distanceTo(obstaclePos) << endl;
			cout << "if condition 1 = " << (ownPos->distanceTo(obstaclePos) < robotShootDistanceOwn) << endl;
			cout << "obstaclePos <----> GoalPosMiddle = " << obstaclePos->distanceTo(goalPosMiddle) << endl;
			cout << "if condition 2 = " << (obstaclePos->distanceTo(goalPosMiddle) < robotShootDistanceGoal) << endl;

			if (ownPos->distanceTo(obstaclePos) < robotShootDistanceOwn
					|| obstaclePos->distanceTo(goalPosMiddle) < robotShootDistanceGoal)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		else
		{
			return true;
		}
	}

	void CheckGoalKick::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		robotShootDistanceOwn = (*sc)["GoalKick"]->get<double>("GoalKick.Default.robotShootDistanceOwn", NULL);
		robotShootDistanceGoal = (*sc)["GoalKick"]->get<double>("GoalKick.Default.robotShootDistanceGoal", NULL);
		kickPower = (*sc)["GoalKick"]->get<double>("GoalKick.Default.kickPower", NULL);
	}

	/*
	 * @return distance between goalPosMiddle and goalPosLeft
	 */
	double CheckGoalKick::calcToleranceAngle()
	{
		// math!!!
		double a = ownPos->distanceTo(goalPosLeft);
		double b = goalPosLeft->distanceTo(goalPosMiddle);
		double c = ownPos->distanceTo(goalPosRight);
		cout << "a = " << a << "\nb= " << b << "\nc= " << c << endl;

		return acos((pow(b, 2) - pow(c, 2) - pow(a, 2)) / (-2 * a * c));
	}

/*PROTECTED REGION END*/
} /* namespace alica */
