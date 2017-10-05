using namespace std;
#include "Plans/Behaviours/RobotMovementDribbleTest.h"

/*PROTECTED REGION ID(inccpp1462969724089) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <RawSensorData.h>
#include <MSLWorldModel.h>
#include <Ball.h>
#include <MSLFootballField.h>
using geometry::CNPointAllo;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1462969724089) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    RobotMovementDribbleTest::RobotMovementDribbleTest() :
            DomainBehaviour("RobotMovementDribbleTest")
    {
        /*PROTECTED REGION ID(con1462969724089) ENABLED START*/ //Add additional options here
        currentTarget = make_shared<geometry::CNPoint2D>();
        attackPosY.push_back(wm->field->getFieldWidth() / 3.0 - 700);
        attackPosY.push_back(0);
        attackPosY.push_back(-wm->field->getFieldWidth() / 3.0 + 700);
        /*PROTECTED REGION END*/
    }
    RobotMovementDribbleTest::~RobotMovementDribbleTest()
    {
        /*PROTECTED REGION ID(dcon1462969724089) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void RobotMovementDribbleTest::run(void* msg)
    {
        /*PROTECTED REGION ID(run1462969724089) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;
        msl_actuator_msgs::MotionControl bm;


        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto ballPos = wm->ball->getPositionEgo();
        auto dstscan = wm->rawSensorData->getDistanceScanBuffer().getLastValidContent();

        // move to ball
        query.egoDestinationPoint = ballPos;
        query.dribble = false;
        query.egoAlignPoint = query.egoDestinationPoint;
        query.fast = true;

        cout << "allo Ball Pos: x: " << ballPos->toAllo(*ownPos).x << " y: " << ballPos->toAllo(*ownPos).y
                << endl;
        cout << "ego Ball Pos: x: " << ballPos->x << " y: " << ballPos->y << endl;

        if (wm->ball->haveBall())
        {
            query.dribble = true;
            query.egoDestinationPoint = CNPointAllo (1, 1).toEgo(*ownPos);
        }
        bm = rm.moveToPoint(query);

        // dribble
        /*
         if (ownPos == nullptr)
         return;
         if (currentTarget == nullptr)
         trueInitialize();
         if (currentTarget == nullptr)
         return;
         auto egoTarget = currentTarget->alloToEgo(*ownPos);
         if (egoTarget->length() < 1200)
         {
         this->setSuccess(true);
         }

         msl_actuator_msgs::MotionControl bm;
         shared_ptr < geometry::CNPoint2D > pathPlanningPoint;
         //bm = DribbleHelper.DribbleToPoint(egoTarget,this.dribbleVel,WM,out pathPlanningPoint);
         //		auto tmpMC = msl::RobotMovement::dribbleToPointConservative(egoTarget, pathPlanningPoint);
         query->egoDestinationPoint = egoTarget;
         query->dribble = true;
         auto tmpMC = rm.experimentallyMoveToPoint(query);

         //        Point2D oppInFront = ObstacleHelper.ClosestOpponentInCorridor(WM,ballPos.Angle(),300);
         //         double distInFront = (oppInFront==null?Double.MaxValue:oppInFront.Distance()-300);
         //
         //         double minInFrontDist = 1800;
         //         if (od!=null && od.Motion!=null) {
         //         minInFrontDist = Math.Max(minInFrontDist,Math.Min(2800,od.Motion.Translation+800));
         //         }
         //         if (ballPos != null && pathPlanningPoint!=null && Math.Abs(HHelper.DeltaAngle(pathPlanningPoint.Angle(),ballPos.Angle())) > Math.PI *4.75/6.0) {
         //         HHelper.SetTargetPoint(WorldHelper.Ego2Allo(pathPlanningPoint,ownPos));
         //         this.FailureStatus = true;
         //         } else if (ballPos!=null && dstscan!=null && distInFront < minInFrontDist && distInFront > 800){
         //         if(oppInFront!=null) HHelper.SetTargetPoint(WorldHelper.Ego2Allo(oppInFront.Rotate(Math.PI),ownPos));
         //         this.FailureStatus = true;
         //         }
         //		shared_ptr<geometry::CNPoint2D> turnTo = msl::RobotMovement::dribbleNeedToTurn(ownPos, ballPos,
         //																						pathPlanningPoint);
         //		if (turnTo!=nullptr) {
         //			HHelper.SetTargetPoint(turnTo);
         //			this->failure = true;
         //		}
         //if i drive in to the enemy goal area
         //		bm = msl::RobotMovement::nearGoalArea(bm);
         //        bm = DriveHelper.NearGoalArea(WM,bm);
         bm = rm.experimentallyRuleActionForBallGetter();

         bm = tmpMC;*/
        send(bm);
        /*PROTECTED REGION END*/
    }
    void RobotMovementDribbleTest::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1462969724089) ENABLED START*/ //Add additional options here
        currentTarget = nullptr;
//        msl::RobotMovement::reset();
        trueInitialize();
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1462969724089) ENABLED START*/ //Add additional methods here
    void RobotMovementDribbleTest::trueInitialize() // so true and so evil
    {
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!ownPos)
            return;
        //		Random rand = new Random();
        //		int index = (int)Math.Round(rand.NextDouble()*2.0);

        srand(supplementary::DateTime::getUtcNowC());
        int index = (rand() % attackPosY.size());

        if (ownPos->x < wm->field->getFieldLength() / 6.0)
        {
            currentTarget = CNPointAllo (wm->field->getFieldLength() / 6.0 - 1500, 0);
            //} else if (ownPos.X < field.FieldLength/2.0) {
            //	currentTarget = new Point2D(field.FieldLength/2.0,0);
        }
        else
        {
            currentTarget = make_shared < geometry::CNPoint2D > (wm->field->getFieldLength() / 4.0 - 1500, 0);
        }
        currentTarget.y = attackPosY.at(index);
        if (currentTarget.toEgo(*ownPos).length() < 1500)
        {
            index = (index + 1) % attackPosY.size(); // select next point in vector
            currentTarget.y = attackPosY.at(index);
        }
    }
/*PROTECTED REGION END*/
} /* namespace alica */
