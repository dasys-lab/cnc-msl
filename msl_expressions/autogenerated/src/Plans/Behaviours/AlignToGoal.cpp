using namespace std;
#include "Plans/Behaviours/AlignToGoal.h"

/*PROTECTED REGION ID(inccpp1415205272843) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1415205272843) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	AlignToGoal::AlignToGoal() :
			DomainBehaviour("AlignToGoal")
	{
		/*PROTECTED REGION ID(con1415205272843) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	AlignToGoal::~AlignToGoal()
	{
		/*PROTECTED REGION ID(dcon1415205272843) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void AlignToGoal::run(void* msg)
	{
		/*PROTECTED REGION ID(run1415205272843) ENABLED START*/ //Add additional options here
		shared_ptr<geometry::CNPoint2D> ballPos = wm->ball.getEgoBallPosition();
		shared_ptr<geometry::CNVelocity2D> ballVel = wm->ball.getEgoBallVelocity();
		shared_ptr<geometry::CNPoint2D> ballVel2 = nullptr;
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision();
		shared_ptr<vector<double> > dstscan = wm->rawSensorData.getDistanceScan();

		msl_actuator_msgs::MotionControl mc;
		if (ownPos == nullptr)
		{
			//TODO
//			mc = DriveHelper.DriveRandomly(500, WM);
			send(mc);
			return;
		}
		if (ballPos == nullptr || ownPos == nullptr)
		{
			return;
		}
		//TODO
//		if (!KickHelper.MayShoot(WM))
//		{
//
//			cout << "TurnToGoal failed (may not shoot)" << endl;
//			this->failure = true;
//			return;
//		}

		if (ballVel == nullptr)
		{
			ballVel2 = make_shared<geometry::CNPoint2D>(0, 0);
		}
		else if (ballVel->length() > 5000)
		{
			shared_ptr<geometry::CNVelocity2D> v = ballVel->normalize() * 5000;
			ballVel2 = make_shared<geometry::CNPoint2D>(v->x, v->y);
		}
		else
		{
			ballVel2 = make_shared<geometry::CNPoint2D>(ballVel->x, ballVel->y);
		}
		shared_ptr<geometry::CNPoint2D> aimPoint;
		if (alloAimPoint != nullptr)
		{
			aimPoint = alloAimPoint->alloToEgo(*ownPos);
		}
		else
		{
			//TODO
//			aimPoint = KickHelper.GetFreeGoalVector(WM);
			if (aimPoint != nullptr)
			{
				alloAimPoint = aimPoint->egoToAllo(*ownPos);
			}

		}

		if (aimPoint == nullptr)
		{
			this->failure = true;
			return;
		}

		double aimAngle = aimPoint->angleTo();
		//TODO
//		double ballAngle = KickHelper.KickerToUse(ballPos->angleTo());
//
//		double deltaAngle = GeometryHelper.DeltaAngle(ballAngle, aimAngle);
//		if (dstscan != nullptr)
//		{
//			double distBeforeBall = KickHelper.MinFree(ballAngle, 200, dstscan);
//			if (deltaAngle < 20 * M_PI / 180 && distBeforeBall < 1000)
//			{
//				this->failure = true;
//			}
//		}
//		mc = msl_actuator_msgs::MotionControl();
//		mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;
//		double hitPoint = GoalLineHitPoint(ownPos, ballAngle);
//		if (abs(hitPoint - alloAimPoint->y) < this->maxYTolerance)
//		{
//			kick(aimPoint);
//		}
//		else
//		{
//			double sign = 0;
//			if (mc.motion.rotation == 0)
//			{
//				sign = 0;
//			}
//			else if (mc.motion.rotation > 0)
//			{
//				sign = 1;
//			}
//			else
//			{
//				sign = -1;
//			}
//			mc.motion.rotation = sign * min(this->maxRot, max(abs(mc.motion.rotation), this->minRot));
//		}
		//TODO
//		lastRotError = deltaAngle;
//		double transBallOrth = ballPos->length() * mc.motion.rotation; //may be negative!
//		double transBallTo = max(ballPos->length(), ballVel2->length()); //Math.Min(1000,ballVel2.Distance());//
//		if (abs(deltaAngle) < 12.0 * M_PI / 180.0)
//		{
//			transBallTo = max(500.0, transBallTo);
//		}
//
//		shared_ptr<geometry::CNPoint2D> driveTo = ballPos->rotate(-M_PI / 2.0);
//		driveTo = driveTo->normalize() * transBallOrth;
//		driveTo += ballPos->normalize() * transBallTo;
//
//		if (driveTo->length() > maxVel)
//		{
//			driveTo = driveTo->normalize() * maxVel;
//		}
//
//		mc.motion.angle = driveTo->angleTo();
//		mc.motion.translation = driveTo->length();

		send(mc);

		/*PROTECTED REGION END*/
	}
	void AlignToGoal::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1415205272843) ENABLED START*/ //Add additional options here
		maxVel = 2000;
		maxRot = M_PI * 4;
		minRot = 0.1;
		maxYTolerance = 15;
		pRot = 2.1;
		dRot = 0.0;
		iter = 0;
		kicked = false;
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision();
		if (ownPos == nullptr)
		{
			alloAimPoint = nullptr;
		}
		else
		{
			//TODO
//			shared_ptr<geometry::CNPoint2D> aimPoint = KickHelper.GetFreeGoalVector(WM);
//			if (aimPoint != nullptr)
//			{
//				alloAimPoint = aimPoint->egoToAllo(*ownPos);
//			}
		}
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1415205272843) ENABLED START*/ //Add additional methods here
	void AlignToGoal::kick(shared_ptr<geometry::CNPoint2D> egoAimPoint)
	{
		if (this->iter > 10)
		{
			this->failure = true;
		}
		if (this->kicked)
		{
			iter++;
			return;
		}
		//TODO
//		if (!KickHelper.MayShoot(WM))
//		{
//			cout << "TurnToGoal failed (must not shoot)" << endl;
//			this->failure = true;
//			return;
//		}

		shared_ptr<vector<double> > dstscan = wm->rawSensorData.getDistanceScan();
		if (dstscan == nullptr)
		{
			cout << "TurnToGoal failed (dstscan nul)" << endl;
			this->failure = true;
			return;
		}

		double MIN_FREE_DISTANCE = 1100.0;
		double FREE_RADIUS = 150;
		//TODO
//		double freeDistance = KickHelper.MinFree(egoAimPoint.Angle(), FREE_RADIUS, dstscan);
//
//		if (freeDistance < MIN_FREE_DISTANCE)
//		{
//			cout << "TurnToGoal failed (obs too close)" << endl;
//			this->failure = true;
//			return;
//		}
//
//		int kickPower = KickHelper.GetKickPower(egoAimPoint.Distance(), 800, od.Motion.Translation);
//
//		if (kickPower > 0)
//		{
//			msl_actuator_msgs::KickControl kc;
//			kc.enabled = true;
//			kc.power = (ushort)kickPower;
//			kc.kicker = egoAimPoint->angleTo();
//			send(kc);
//			this->kicked = true;
//			cout << "Kick: dist " << egoAimPoint->length() << " pw: " <<  kickPower << endl;
//		}
	}
/*PROTECTED REGION END*/
} /* namespace alica */
