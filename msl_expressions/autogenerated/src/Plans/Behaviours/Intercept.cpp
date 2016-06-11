using namespace std;
#include "Plans/Behaviours/Intercept.h"

/*PROTECTED REGION ID(inccpp1458757170147) ENABLED START*/ //Add additional includes here
#include <Ball.h>
#include <RawSensorData.h>
#include <Game.h>
#include <Kicker.h>
#include <pathplanner/PathProxy.h>
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1458757170147) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	Intercept::Intercept() :
			DomainBehaviour("Intercept")
	{
		/*PROTECTED REGION ID(con1458757170147) ENABLED START*/ //Add additional options here
		maxVel = 2500;

		sc = supplementary::SystemConfig::getInstance();
		pp = msl::PathProxy::getInstance();

		pdist = 1.2;
		pidist = 0.1;
		pddist = 0.6;

		lastDistErr = 0;
		distIntErr = 0;

		aheadWeight = 0.5;

		prot = 0.0; //1.3
		pirot = 0.0; //0.1
		pdrot = 0.0;

		lastRotErr = 0;
		rotIntErr = 0;

		predictByRawOdo = false;

		prot = (*sc)["Drive"]->get<double>("Drive.Intercept.RotationP", NULL);
		pirot = (*sc)["Drive"]->get<double>("Drive.Intercept.RotationI", NULL);
		pdrot = (*sc)["Drive"]->get<double>("Drive.Intercept.RotationD", NULL);

		pdist = (*sc)["Drive"]->get<double>("Drive.Intercept.DistanceP", NULL);
		pidist = (*sc)["Drive"]->get<double>("Drive.Intercept.DistanceI", NULL);
		pddist = (*sc)["Drive"]->get<double>("Drive.Intercept.DistanceD", NULL);

		maxVel = (*sc)["Behaviour"]->get<double>("Behaviour.MaxSpeed", NULL);

		query = make_shared<msl::MovementQuery>();
		/*PROTECTED REGION END*/
	}
	Intercept::~Intercept()
	{
		/*PROTECTED REGION ID(dcon1458757170147) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void Intercept::run(void* msg)
	{
		/*PROTECTED REGION ID(run1458757170147) ENABLED START*/ //Add additional options here
		// ACQUIRE NECESSARY DATA
		auto ownPos = wm->rawSensorData->getOwnPositionVision();
		msl_actuator_msgs::MotionControl mc;
		if (ownPos == nullptr)
		{
			mc = rm.driveRandomly(500);
			send(mc);
			return;
		}

		auto egoBallPos = wm->ball->getEgoBallPosition();
		if (!egoBallPos)
		{
			return;
		}

		auto egoBallVel = wm->ball->getVisionBallVelocity();
		if (!egoBallVel)
		{
			egoBallVel = make_shared<geometry::CNVelocity2D>(0, 0);
		}
		else if (egoBallVel->length() > 7000) // TODO config parameter
		{
			egoBallVel = egoBallVel->normalize() * 7000;
		}

		// Ball is outside field, so drive to its position mapped into field
		auto alloBall = egoBallPos->egoToAllo(*ownPos);
		if (!wm->field->isInsideField(alloBall))
		{
			auto egoTarget = wm->field->mapInsideField(alloBall)->alloToEgo(*ownPos);

			this->query->egoDestinationPoint = egoTarget;
			this->query->egoAlignPoint = egoBallPos;
			mc = rm.moveToPoint(query);
			if (egoTarget->length() < 100) // TODO config parameter
			{
				mc.motion.translation = 0;
			}

			send(mc);
			return;
		}

		// PID controller for minimizing the distance between ball and me
		double distErr = egoBallPos->length();
		double controlDist = distErr * pdist + distIntErr * pidist + (distErr - lastDistErr) * pddist;

		distIntErr += distErr;
		distIntErr = max(-1000.0, min(1000.0, distIntErr));
		lastDistErr = distErr;

		shared_ptr<geometry::CNPoint2D> egoVelocity;
		auto currentGameState = this->wm->game->getGameState();
		if (currentGameState == msl::GameState::OppBallPossession)
		{
			//TODO if we set this to 0 it works in the simulator
			egoVelocity = make_shared<geometry::CNPoint2D>(0, 0);
		}
		else
		{
			egoVelocity = egoBallVel->getPoint();
		}
		egoVelocity->x += controlDist * cos(egoBallPos->angleTo());
		egoVelocity->y += controlDist * sin(egoBallPos->angleTo());

		auto pathPlanningPoint = egoVelocity->normalize() * min(egoVelocity->length(), egoBallPos->length());
		auto alloDest = pathPlanningPoint->egoToAllo(*ownPos);
		if (wm->field->isInsideField(alloBall, -150) && !wm->field->isInsideField(alloDest))
		{
			//pathPlanningPoint = wm->field->mapInsideField((alloDest, alloBall - ownPos))->alloToEgo(*ownPos);
			pathPlanningPoint = wm->field->mapInsideField((alloDest, alloBall - alloDest))->alloToEgo(*ownPos);
		}

		shared_ptr<msl::PathEvaluator> eval = make_shared<msl::PathEvaluator>();
		auto pathPlanningResult = pp->getEgoDirection(pathPlanningPoint, eval);
		if (pathPlanningResult == nullptr)
		{
			mc.motion.angle = pathPlanningPoint->angleTo();
		}
		else
		{
			mc.motion.angle = pathPlanningResult->angleTo();
		}

		if (pathPlanningResult->distanceTo(pathPlanningPoint) > 10.0)
		{
			mc.motion.translation = min(this->maxVel, max(pathPlanningResult->length(), egoVelocity->length()));
		}
		else
		{
			mc.motion.translation = min(this->maxVel, pathPlanningPoint->length());
		}

		// PID controller for minimizing the kicker angle to ball
		double angleGoal = msl::Kicker::kickerAngle;
		double rotErr = geometry::deltaAngle(angleGoal, egoBallPos->angleTo());
		double controlRot = rotErr * prot + rotIntErr * pirot + (rotErr - lastRotErr) * pdrot;
		controlRot = max(-4 * M_PI, min(4 * M_PI, controlRot));

		rotIntErr += rotErr;
		rotIntErr = max(-2 * M_PI, min(2 * M_PI, rotIntErr));
		lastRotErr = rotErr;

		// this is nice stuff but only when we are not approaching the opponent
		if (egoBallPos->length() < 700
				&& (currentGameState == msl::GameState::OwnBallPossession || currentGameState == msl::GameState::NobodyInBallPossession))
		{
			controlRot *= 2.3;
			//we probably translate to fast and cannot rotate anymore: So translate slower
			if (abs(rotErr) > M_PI / 6)
			{
				mc.motion.translation *= min((abs(rotErr) - M_PI / 6) / (M_PI * 5.0 / 6.0), egoBallVel->length());
			}
		}
		mc.motion.rotation = controlRot;

		// Special handling for things around critical areas
		auto tmpMC = rm.ruleActionForBallGetter();
		if (!std::isnan(tmpMC.motion.translation))
		{
			send(tmpMC);
		}
		else
		{
			send(mc);
		}

		if (wm->ball->haveBallDribble(false))
		{
			this->setSuccess(true);
		}
		/*PROTECTED REGION END*/
	}
	void Intercept::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1458757170147) ENABLED START*/ //Add additional options here
		lastDistErr = 0;
		distIntErr = 0;

		lastRotErr = 0;
		rotIntErr = 0;

		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1458757170147) ENABLED START*/ //Add additional methods here
	bool Intercept::interceptPoint(shared_ptr<geometry::CNPoint2D> egoBall, shared_ptr<geometry::CNPoint2D> ballVel,
									double maxVel, double& t, shared_ptr<geometry::CNPoint2D>& interceptVelo)
	{
		t = 0;
		interceptVelo = nullptr;
		if (ballVel->length() < 10)
		{
			return false;
		}
		double denum = egoBall->length() * egoBall->length();
		double p = 2 * (-egoBall->y * egoBall->y * ballVel->x + egoBall->x * egoBall->y * ballVel->y);
		double q = -egoBall->x * egoBall->x * maxVel * maxVel + egoBall->y * egoBall->y * ballVel->x * ballVel->x
				+ egoBall->x * egoBall->x * ballVel->y * ballVel->y
				- 2 * egoBall->x * egoBall->y * ballVel->y * ballVel->y;

		p /= denum;
		q /= denum;

		double sq = p * p / 4 - q;
		if (sq < 0)
		{
			return false; //cant reach
		}
		double vx1 = -p / 2 - sqrt(sq);
		double vx2 = -p / 2 + sqrt(sq);

		double t1 = -egoBall->x / (ballVel->x - vx1);
		double t2 = -egoBall->x / (ballVel->x - vx2);
		double vx = 0, vy = 0;

		if ((t2 < 0 && t1 > 0) || (t1 > 0 && t1 < t2))
		{
			vx = vx1;
			t = t1;
		}
		else if (t2 > 0)
		{
			vx = vx2;
			t = t2;
		}
		else
		{
			return false; //can't reach
		}
		vy = (egoBall->y + t * ballVel->y) / t;

		interceptVelo->x = vx;
		interceptVelo->y = vy;

		if (interceptVelo->length() > maxVel)
		{
			interceptVelo = interceptVelo->normalize() * maxVel;
		}
		return true;
	}
/*PROTECTED REGION END*/
} /* namespace alica */
