using namespace std;
#include "Plans/Standards/Own/PassIntoPath/StandardAlignToGeneric.h"

/*PROTECTED REGION ID(inccpp1457531616421) ENABLED START*/ //Add additional includes here
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1457531616421) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StandardAlignToGeneric::StandardAlignToGeneric() :
            DomainBehaviour("StandardAlignToGeneric")
    {
        /*PROTECTED REGION ID(con1457531616421) ENABLED START*/ //Add additional options here
        tol = M_PI / 40; //45
        trans = 100.0;
//    				field = FootballField.GetInstance();
//			this.maxTranslation = this.sc["Team"].TryGetDouble(3000.0, "Team.StandardSituationSpeed");
//			this.tol = this.sc["Behaviour"].GetDouble("StandardAlign.AlignTolerance");
//			this.trans = this.sc["Behaviour"].GetDouble("StandardAlign.AlignSpeed");
        /*PROTECTED REGION END*/
    }
    StandardAlignToGeneric::~StandardAlignToGeneric()
    {
        /*PROTECTED REGION ID(dcon1457531616421) ENABLED START*/ //Add additional options here
        query = make_shared < Query > (wm->getEngine());

        supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
        maxVel = (*sys)["Behaviour"]->get<double>("Behaviour.MaxSpeed", NULL);
        /*PROTECTED REGION END*/
    }
    void StandardAlignToGeneric::run(void* msg)
    {
        /*PROTECTED REGION ID(run1457531616421) ENABLED START*/ //Add additional options here
        /*		Point2D ballPos = WM.RawBallPosition;

         Position ownPos = WM.OwnPositionCorrected;

         if(ballPos == null || ownPos == null) {
         DriveHelper.DriveRandomly(500,WM);
         return;
         }
         if (delayKickCounter == 0)
         {
         KickControl km = new KickControl();
         km.Enabled = true;
         km.Kicker = (ushort)KickHelper.KickerToUseIndex(ballPos.Angle());
         km.Power = 0;
         Send(km);
         delayKickCounter = 9;
         }
         else
         {
         delayKickCounter--;
         }

         //////////////////////////////////////////
         // If ball is far away: Move close to Ball
         //////////////////////////////////////////
         MotionControl bm = null;
         if (ballPos.Distance() > 900) {
         bm = DriveHelper.StandardMoveToPosition(ballPos,ballPos,WM);
         Send(bm);
         return;

         }
         haveBall  = WorldHelper.HaveBallDribble(WM,haveBall);
         if(ballPos.Distance() > 450) { //!haveBall) {
         bm = DriveHelper.DriveToPointAlignNoAvoidance(ballPos,ballPos,Math.Min(600,ballPos.Distance()/1.66),true,WM);
         Send(bm);
         return;
         }
         /////////////////////////////////////////
         // End of Move close Ball
         /////////////////////////////////////////

         bm = new MotionControl();
         double radian = ballPos.Distance();



         double rot = this.trans/radian;


         bool solved=true;

         solved = query.GetSolution(this.RunningPlan,out result);


         Point2D target = new Point2D();
         //Console.WriteLine(solved + " X:" +result[0] + "Y:" + result[1]);
         if(true || solved)
         {
         target = WorldHelper.Allo2Ego(new Point2D(result[0], result[1]), ownPos);

         PassMsg pm = new PassMsg();
         pm.Origin.X = ownPos.X;
         pm.Origin.Y = ownPos.Y;
         pm.Destination.X = result[0];
         pm.Destination.Y = result[1];
         pm.ValidFor = 5000; //100msec

         //Reduce Communication
         if(iterationCount++%3==0) {
         Speak(pm, -1);
         }
         }
         else
         {
         //Emergency kick
         Point2D alloBall = WorldHelper.Ego2Allo(ballPos, ownPos);

         Point2D pointTowardsUs = null;
         if (alloBall.X > -field.FieldLength/3) {
         pointTowardsUs = new Point2D(alloBall.X-1000.0, alloBall.Y);
         }
         else {
         pointTowardsUs = new Point2D(alloBall.X, alloBall.Y+Math.Sign(alloBall.Y-ownPos.Y)*1000);
         }
         target = WorldHelper.Allo2Ego(pointTowardsUs, ownPos);
         }

         double cross = target.X * ballPos.Y - target.Y * ballPos.X;
         double fac = -Math.Sign(cross);
         Point2D direction = null;
         double dangle = DeltaAngle(KickHelper.KickerToUse(ballPos.Angle()),target.Angle());

         if (Math.Abs(dangle) < 12.0*Math.PI/180.0)  {
         direction = ballPos.Rotate(-fac*Math.PI/2.0).Normalize()*this.trans*0.66;
         }
         else {
         direction = ballPos.Rotate(-fac*Math.PI/2.0).Normalize()*this.trans;
         }

         double balldangle = DeltaAngle(KickHelper.KickerToUse(ballPos.Angle()),ballPos.Angle());
         if (ballPos.Distance() > 350 && Math.Abs(dangle) > 35.0*Math.PI/180.0) {
         bm.Motion.Angle = direction.Angle();
         bm.Motion.Translation = direction.Distance()*1.6;
         bm.Motion.Rotation = fac*rot*1.6;
         Send(bm);
         return;
         }

         if(!haveBall) {
         if(Math.Abs(balldangle) > 20.0*Math.PI/180.0) {
         bm.Motion.Rotation = Math.Sign(balldangle)*0.8;
         bm.Motion.Angle = 0;
         bm.Motion.Translation = 0;
         Send(bm);
         return;
         } else {
         bm.Motion.Rotation = balldangle*0.5;
         bm.Motion.Angle = ballPos.Angle();
         bm.Motion.Translation = ballPos.Distance();
         Send(bm);
         return;
         }
         }
         bm.Motion.Angle = direction.Angle();
         bm.Motion.Translation = direction.Distance();
         bm.Motion.Rotation = fac*rot;


         if( Math.Abs(dangle) < this.tol && haveBall)
         {
         bm.Motion.Rotation = 0.0;
         bm.Motion.Translation = 0.0;
         this.SuccessStatus = true;
         }

         Send(bm);*/

//		msl_actuator_msgs::MotionControl mc;
//		auto ownPos = wm->rawSensorData.getOwnPositionVision();
//		auto ballPos = wm->ball.getAlloBallPosition();
//		if (ownPos==nullptr || ballPos==nullptr) return;
//
//		bool ret = query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result);
//		auto passGoal = make_shared<geometry::CNPoint2D>(result[0], result[1]);
//		auto p = ballPos+(ballPos-passGoal)->normalize()*(2000.0/3.0);
//		if(ret) {
//			msl_helper_msgs::PassMsg pm;
//			pm.origin.x = ownPos->x;
//			pm.origin.y = ownPos->y;
//			pm.destination.x = passGoal->x;
//			pm.destination.y = passGoal->y;
//			pm.validFor = 5000000000;
//
//			//Reduce Communication
//			if(iterationCount++%3==0) {
//				send(pm, -1);
//			}
//		}
//
//
//		auto egoBall=wm->ball.getEgoBallPosition();
//		if (result!=nullptr && result.size() > 0)  {
//			auto driveTo = p->alloToEgo(*ownPos);
//			mc = msl::RobotMovement::placeRobotCareBall(driveTo, egoBall, maxVel);
//		} else {
//			return;
//		}
//		send(mc);
        /*PROTECTED REGION END*/
    }
    void StandardAlignToGeneric::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1457531616421) ENABLED START*/ //Add additional options here
        iterationCount = 0;
        query->clearStaticVariables();
        query->addStaticVariable(getVariablesByName("X"));
        query->addStaticVariable(getVariablesByName("Y"));
        result.clear();

//			try
//			{
//				if (parameters.ContainsKey("xOffSet")) {
//					xOffSet = Double.Parse(parameters["xOffSet"]);
//				} else {
//					xOffSet = 0;
//				}
//			}
//			catch(Exception)
//			{
//				xOffSet = 0;
//			}
//			if( parameters.ContainsKey("AlignTolerance") && parameters.ContainsKey("AlignSpeed") )
//			{
//				try
//				{
//					this.tol = Double.Parse(parameters["AlignTolerance"]);
//					this.trans = Double.Parse(parameters["AlignSpeed"]);
//				}
//				catch( Exception )
//				{
//					this.tol = this.sc["Behaviour"].GetDouble("StandardAlign.AlignTolerance");
//					this.trans = this.sc["Behaviour"].GetDouble("StandardAlign.AlignSpeed");
//				}
//			}
//			else {
//				this.tol = this.sc["Behaviour"].GetDouble("StandardAlign.AlignTolerance");
//				this.trans = this.sc["Behaviour"].GetDouble("StandardAlign.AlignSpeed");
//			}
//			//receiver = GetRunningPlanEntryPointPair("TestThrowInPos1", "Defend");
//			query.ClearStaticVariables();
//			this.query.AddVariable(VariableByName("X"));
//			this.query.AddVariable(VariableByName("Y"));
//
//			haveBall = false;
//			delayKickCounter = 0;

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1457531616421) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
