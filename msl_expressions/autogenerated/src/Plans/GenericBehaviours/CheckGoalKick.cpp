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
		auto ownPosition = wm->rawSensorData.getOwnPositionVision();
		ownPos->x = ownPosition->x;
		ownPos->y = ownPosition->y;

		//get ego ball pos
		egoBallPos = wm->ball.getEgoBallPosition();

		//get goal pos
		goalPosLeft = field->posLeftOppGoalPost();
		goalPosRight = field->posRightOppGoalPost();
		goalPosMiddle = field->posOppGoalMid();

		readConfigParameters();
		toleranceAngle = calcToleranceAngle();

		egoAlignPoint = goalPosMiddle;

		if (checkGoalLine() && checkShootPossibility())
		{
			cout << "kicking" << endl;
			msl_actuator_msgs::KickControl kc;
			kc.enabled = true;
			//angle
			kc.kicker = egoBallPos->angleTo();
			kc.power = kickPower;
			send(kc);
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
	bool CheckGoalKick::checkGoalLine()
	{
		// if angle is smaller then 10 degree return true
		if (egoAlignPoint->rotate(M_PI)->angleTo() < M_PI * toleranceAngle
				|| egoAlignPoint->rotate(M_PI)->angleTo() > -M_PI * toleranceAngle)
		{
			return true;
		}
		return false;
	}

	bool CheckGoalKick::checkShootPossibility()
	{

		// check if obstacle lays in corridor
		auto obstacles = wm->robots.getObstacles();
		shared_ptr<geometry::CNPoint2D> obstaclePoint = nullptr;
		bool foundObstacle = false;
		int obstacleAt = -1;

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

		if (obstacleAt > -1)
		{
			// check if obstacle is blocking (Own distance -> obstacle and obstacle -> OppGoal)
			auto obstacle = obstacles->at(obstacleAt);
			shared_ptr<geometry::CNPoint2D> obstaclePos = nullptr;
			obstaclePos->x = obstacle.x;
			obstaclePos->y = obstacle.y;

			if (ownPos->distanceTo(obstaclePos) < robotShootDistanceOwn
					&& obstaclePos->distanceTo(goalPosMiddle) < robotShootDistanceGoal)
			{
				return false;
			}
		}
		else
		{
			return true;
		}

		return false;
	}

	void CheckGoalKick::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		robotShootDistanceOwn = (*sc)["GoalKick"]->get<double>("GoalKick.Default.robotShootDistanceOwn", NULL);
		robotShootDistanceGoal = (*sc)["GoalKick"]->get<double>("GoalKick.Default.robotShootDistanceGoal", NULL);
//		toleranceAngle = (*sc)["GoalKick"]->get<double>("GoalKick.Default.toleranceAngleNumerator", NULL)
//				/ (*sc)["GoalKick"]->get<double>("GoalKick.Default.toleranceAngleDenominator", NULL);
		kickPower = (*sc)["GoalKick"]->get<double>("GoalKick.Default.kickPower", NULL);
	}

	double CheckGoalKick::calcToleranceAngle()
	{
		// math!!!
		double a = ownPos->distanceTo(goalPosLeft);
		double b = goalPosLeft->distanceTo(goalPosRight);
		double c = ownPos->distanceTo(goalPosRight);

		return acos((pow(b,2) + pow(c,2) - pow(a,2)) / (2*b*c));
	}

/*PROTECTED REGION END*/
} /* namespace alica */
