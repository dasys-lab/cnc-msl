using namespace std;
#include "Plans/Behaviours/Intercept.h"

/*PROTECTED REGION ID(inccpp1458757170147) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1458757170147) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Intercept::Intercept() :
            DomainBehaviour("Intercept"), useZmachine(false)
    {
        /*PROTECTED REGION ID(con1458757170147) ENABLED START*/ //Add additional options here
    	field = msl::MSLFootballField::getInstance();
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
		bool fastIntercept = false;
		auto ballVel = wm->ball.getVisionBallVelocity();
		double smoothingLength=1.0;

		auto ballPos = wm->ball.getEgoBallPosition();

		auto ownPos  = wm->rawSensorData.getOwnPositionVision();
//		CorrectedOdometryData od  = WM.OdometryData;
//		OdometryData odRaw = WM.RawOdometryData;


		msl_actuator_msgs::MotionControl mc;
		if (ownPos == nullptr) {
			mc = msl::RobotMovement::driveRandomly(500);
			send(mc);
			return;
		}
		if (ballPos == nullptr) {
			return;
		}

		if (ballVel == nullptr) {
			ballVel = make_shared<geometry::CNVelocity2D>(0,0);
		}



		if (ballVel->length() > 7000) {
			ballVel = ballVel->normalize()*7000;
		}
		auto alloBall = ballPos->egoToAllo(*ownPos);
		if (!field->isInsideField(alloBall)) {
			auto egoTarget=field->mapInsideField(alloBall)->alloToEgo(*ownPos);
			mc = msl::RobotMovement::placeRobotCareBall(egoTarget,ballPos,maxVel);
			send(mc);

			return;
		}

		mc = msl::RobotMovement::nearGoalArea(mc);


		shared_ptr<geometry::CNPoint2D> predBall = ballPos;

		shared_ptr<geometry::CNPosition> predPos;

//		if (predictByRawOdo) {
//			WM.Predictor.PredictBallRobotSystem(odRaw.Motion,ballPos,ballVel,ownPos,160,out predBall,out predPos);
//
//		} else {
//			WM.Predictor.PredictBallRobotSystem(od.Motion,ballPos,ballVel,ownPos,160,out predBall,out predPos);
//		}

//			Console.WriteLine("predBall x " + predBall.X + " predBall y " + predBall.Y );

		/*if (predBall.Distance()> 250) {
			predBall = predBall.Normalize()*(predBall.Distance()-250);
		}*/
		auto gs = wm->game.getGameState();
		shared_ptr<geometry::CNPoint2D> vel;

		if(gs == msl::GameState::OppBallPossession) {
			vel = make_shared<geometry::CNPoint2D>(0,0);
		} else {
			vel = make_shared<geometry::CNPoint2D>(ballVel->x,ballVel->y);
		}
//
//			Point2D vel = (new Point2D(predBall.X,predBall.Y)).Normalize();
//			vel *= ballVel.Length()+500;

		double distErr = predBall->length();
		//pdist = 1.0;
		//pddist = 0.3;
		double controlDist = distErr*pdist + distIntErr*pidist + (distErr-lastDistErr)*pddist;
		//

		//controlDist=(Math.Max(600, controlDist));
//		if (useZmachine) {
//			if(distErr>1000) {
//			if(distErr>1800) distErr = 1800;
//				controlDist = transController.getOutput((distErr-150)/1000)*1000 + 400;
//				//if(distErr<300)
//				transController.updateMem((distErr-150)/1000);
//			}
//		}
		//Console.WriteLine("WIntecept Disterror/controllerout" + distErr + " / " + controlDist);

		distIntErr += distErr;
		distIntErr = max(-1000.0,min(1000.0,distIntErr));
		lastDistErr = distErr;

		vel->x += controlDist*cos(predBall->angleTo());
		vel->y += controlDist*sin(predBall->angleTo());

//		mc = new MotionControl();



		shared_ptr<geometry::CNPoint2D> t = make_shared<geometry::CNPoint2D>(ballVel->x, ballVel->y);
		auto  predBallVel = t->rotate(-1 * ownPos->theta)->rotate(predPos->theta);
		shared_ptr<geometry::CNPoint2D> pV = make_shared<geometry::CNPoint2D>(predBallVel->x,predBallVel->y);

		/*if(InterceptPoint(predBall,pV,od.Motion.Translation,out time,out intercept)) {
			double d = vel.Distance();
			vel= vel*(1-aheadWeight)+intercept*aheadWeight;
                            vel= vel.Normalize()*d;

		}*/
		double skalar = -predBall->x*ballVel->x-predBall->y*ballVel->y;
		skalar /= predBall->length()*ballVel->length();
		double ang = acos(skalar);
///TESTING:
//		double cosBallVel = ballPos.X*ballVel.Vx+ballPos.Y*ballVel.Vy / (ballPos.Distance()*ballVel.Length());
//		// cos(180 +-60) = 0.5
//		if (ballVel.Length() > 600 && cosBallVel < -0.5) {
//
//			Point2D opoint = DriveHelper.GetIntersectionPointOfLines(new Point2D(0, 0), new Point2D(ballVel.Vy, -ballVel.Vx), ballPos, new Point2D(ballVel.Vx, ballVel.Vy));
//			vel = vel.Normalize() * Math.Max(0,vel.Distance()-opoint.Distance()*4.0);
//			vel += opoint * 4.0;
//
//		}
//
//


//END TESTING
		auto pathPlanningPoint = vel->normalize()*min(vel->length(),predBall->length());

//Console.WriteLine("4vel {0} {1}",vel.X,vel.Y);
/*			if (WorldHelper.EnemyHasBall(WM)) {
			if (InterceptPoint(ballPos,ballVel,od.Motion.Translation,out time,out intercept)) {
				double d = vel2.Distance();
				vel2= vel2*(1-aheadWeight)+intercept*aheadWeight;
				vel2= vel2.Normalize()*d;
			}
		}
*/
		double trans = 0;
//		double curMaxTrans = DriveHelper.GetEmergencySpeed(WM,pathPlanningPoint);

		double currMaxTrans = maxVel;
		/*if(vel.Distance() < ballVel.Length()+750 && DriveHelper.BallMovesAway(ballPos,ballVel)) {
			vel = vel.Normalize()*(ballVel.Length()+750);
		}*/
//Console.WriteLine("3vel2 {0} {1} {2}",vel2.Angle(),vel2.X,vel2.Y);
//			Console.WriteLine("curMaxTrans " + curMaxTrans + " vel.Distance " + vel.Distance());
		auto alloDest = pathPlanningPoint->egoToAllo(*ownPos);
		if (field->isInsideField(alloBall,-150) && !field->isInsideField(alloDest)) {
			pathPlanningPoint = field->mapInsideField((alloDest,alloBall-ownPos))->alloToEgo(*ownPos);
		}
		vector<double> empty;
//		double[] empty = new double[0];

		shared_ptr<msl::PathEvaluator> eval = make_shared<msl::PathEvaluator>();
		pathPlanningPoint=pp->getEgoDirection(pathPlanningPoint,eval);

//			if(ballPos.Distance() < 300) trans = Math.Max(trans,Math.Max(500,ballPos.Distance()*2.0));
//
		if (pathPlanningPoint==nullptr) {
			mc.motion.angle = vel->angleTo();
		}
		else {
			mc.motion.angle = pathPlanningPoint->angleTo();
		}
//		if(fastIntercept) {
//			mc.Motion.Translation = Math.Min(curMaxTrans,vel.Distance());
//		}
//		else mc.Motion.Translation = trans;
		mc.motion.translation = trans;

//		double angleGoal = KickHelper.KickerToUse(ballPos.Angle());
		double angleGoal = msl::Kicker::kickerAngle;

		double rotErr = geometry::deltaAngle(angleGoal,ballPos->angleTo());

		double controlRot = rotErr*prot+rotIntErr*pirot+(rotErr-lastRotErr)*pdrot;

		rotIntErr += rotErr;
		rotIntErr = max(-2*M_PI,min(2*M_PI,rotIntErr));
		lastRotErr = rotErr;


		controlRot = max(-4*M_PI,min(4*M_PI,controlRot));
		// this is nice stuff but only when we are not approaching the opponent
		cout <<  "Intercept: PredBallDist " << predBall->length() << endl;
		if(predBall->length() < 700 && (gs == msl::GameState::OwnBallPossession || gs == msl::GameState::NobodyInBallPossession)) {
			controlRot *= 2.3;
			//we probably translate to fast and can not rotate anymore: So translate slower
			if(abs(rotErr) > M_PI/6) {
				shared_ptr<geometry::CNPoint2D> velo = make_shared<geometry::CNPoint2D>(ballVel->x, ballVel->y);
				mc.motion.translation *= min((abs(rotErr)-M_PI/6)/(M_PI*5.0/6.0),velo->length());
			}
		}
		mc.motion.rotation = controlRot;

		send(mc);
		if(wm->ball.haveBallDribble(false)) {
			this->success = true;
		}
        /*PROTECTED REGION END*/
    }
    void Intercept::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1458757170147) ENABLED START*/ //Add additional options here
    	lastDistErr = 0;
    	distIntErr=0;

    	lastRotErr = 0;
    	rotIntErr = 0;

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1458757170147) ENABLED START*/ //Add additional methods here
    bool  Intercept::interceptPoint(shared_ptr<geometry::CNPoint2D> egoBall, shared_ptr<geometry::CNPoint2D> ballVel, double maxVel, double& t, shared_ptr<geometry::CNPoint2D>& interceptVelo) {
    	t = 0;
		interceptVelo = nullptr;
		if (ballVel->length() < 10) {
			return false;
		}
		double denum = egoBall->length() * egoBall->length();
		double p = 2* (-egoBall->y*egoBall->y*ballVel->x + egoBall->x*egoBall->y*ballVel->y);
		double q = -egoBall->x*egoBall->x*maxVel*maxVel + egoBall->y*egoBall->y*ballVel->x*ballVel->x + egoBall->x*egoBall->x*ballVel->y*ballVel->y
			-2*egoBall->x*egoBall->y*ballVel->y*ballVel->y;


		p /= denum;
		q /= denum;

		double sq = p*p/4 - q;
		if (sq<0) {
			return false; //cant reach
		}
		double vx1 = -p/2-sqrt(sq);
		double vx2 = -p/2+sqrt(sq);

		double t1 = -egoBall->x / (ballVel->x - vx1);
		double t2 = -egoBall->x / (ballVel->x - vx2);
		double vx=0,vy=0;

		if ((t2 < 0 && t1 > 0)|| (t1 > 0 && t1 < t2)) {
			vx = vx1;
			t = t1;
		} else if (t2 > 0) {
			vx = vx2;
			t=t2;
		}
		else {
			return false; //can't reach
		}
		//vy = Math.Sqrt(maxVel*maxVel - vx*vx);
		vy = (egoBall->y+t*ballVel->y)/t;

		interceptVelo->x = vx;
		interceptVelo->y = vy;
//		interceptVelo = new Point2D(vx,vy);


		if (interceptVelo->length()> maxVel) {
			interceptVelo = interceptVelo->normalize()*maxVel;
		}
		return true;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
