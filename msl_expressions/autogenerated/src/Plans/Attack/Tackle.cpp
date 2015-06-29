using namespace std;
#include "Plans/Attack/Tackle.h"

/*PROTECTED REGION ID(inccpp1434807660243) ENABLED START*/ //Add additional includes here
#include "GeometryCalculator.h"
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1434807660243) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Tackle::Tackle() :
            DomainBehaviour("Tackle")
    {
        /*PROTECTED REGION ID(con1434807660243) ENABLED START*/ //Add additional options here
        maxVel = 4000;
        ballDist = 330;
        ballDistTolerance = 90;
        rotationVel = 2 * M_PI;
        field = msl::MSLFootballField::getInstance();
        wiggleDir = 0;
        errorInt = 0;
        sc = supplementary::SystemConfig::getInstance();
        /*PROTECTED REGION END*/
    }
    Tackle::~Tackle()
    {
        /*PROTECTED REGION ID(dcon1434807660243) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Tackle::run(void* msg)
    {
        /*PROTECTED REGION ID(run1434807660243) ENABLED START*/ //Add additional options here
//		shared_ptr<geometry::CNPoint2D> ballPos = wm->ball.getEgoBallPosition();
//		shared_ptr<geometry::CNPoint2D> enemyGoalPos;
//		shared_ptr<geometry::CNPoint2D> ownGoalPos;
//		msl_actuator_msgs::MotionControl bm;
//
//		shared_ptr<msl_sensor_msgs::CorrectedOdometryInfo> od = wm->rawSensorData.getCorrectedOdometryInfo();
//
//		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision();
//		if (ballPos->length() > 1000)
//		{
//
//			bm = DriveHelper.TryToCatchBallWrtRules(WM);
//			send(bm);
//			wiggleDir = 0;
//			errorInt = 0;
//			return;
//		}
//		shared_ptr<geometry::CNPoint2D> ballVecNorm = ballPos->normalize();
//
//		if (ownPos != nullptr)
//		{
//			enemyGoalPos = alloEnemyGoal->alloToEgo(*ownPos);
//			ownGoalPos = alloOwnGoal->alloToEgo(*ownPos);
//		}
//		else
//		{
//			enemyGoalPos = ballVecNorm * 3000;
//		}
//
//		if (WorldHelper.IsBallClear(WM, true) || !WorldHelper.EnemyHasBall(WM))
//		{
//			if (wm->ball.haveBall())
//			{
//				shared_ptr<geometry::CNPoint2D> pathPlanningPoint;
//				bm = DribbleHelper.DribbleToPoint(enemyGoalPos,maxVel,WM, out pathPlanningPoint);
//				//Console.WriteLine("Dribbling");
//				send(bm);
//				errorInt = 0;
//				this->success = true;
//				wiggleDir = 0;
//				return;
//			}
//			else
//			{
//				bm = DriveHelper.TryToCatchBallWrtRules(WM);
//				//Console.WriteLine("Catching");
//				send(bm);
//				errorInt = 0;
//				return;
//			}
//		}
//
//		double error = ballPos->length() - ballDist;
//
//		bm = msl_actuator_msgs::MotionControl();
//		bm.motion.angle = ballPos->angleTo();
//		errorInt += error * 0.01;
//		bm.motion.translation = error * 6 + errorInt;
//		if (bm.motion.translation > 1200)
//		{
//			bm.motion.translation = 1200;
//		}
//		else if (bm.motion.translation < -1200)
//		{
//			bm.motion.translation = -1200;
//		}
//		bm.motion.rotation = 0;
//		double ballAngle = ballPos->angleTo();
//
//		double kickAngle = M_PI;
//		double dangle = geometry::GeometryCalculator::deltaAngle(kickAngle, ballAngle);
//		if (ballPos->length() > ballDist + ballDistTolerance)
//		{
//			//				errorInt = 0;
//			wiggleDir = 0;
//			send(bm);
//			return;
//		}
//
//		//decide in which direction to rotate
//		if (wiggleDir == 0)
//		{
//
//			//TODO: test if directions are correct
//			if (ownGoalPos != nullptr && ownGoalPos->length() < field->FieldLength / 3.0)
//			{
//				//own goal is close, get the ball away
//				if (checkSide(ballVecNorm, ownGoalPos))
//				{
//					wiggleDir = -1;
//				}
//				else
//				{
//					wiggleDir = 1;
//				}
//			}
//			else
//			{
//
//				//own goal is far away, look for nearby friends
//				shared_ptr<geometry::CNPosition> friendly;
//				for (RobotSHWMData rdat : SHWM.TeamSHWMData.Values)
//				{
//					if (rdat.properties != SHWM.GetOwnRobotProperties())
//					{
//
//						if (rdat.positionReceived && rdat.PlayerPosition.Distance() < 2000)
//						{
//							if (friendly == nullptr || friendly.Distance() > rdat.PlayerPosition.Distance())
//							{
//								friendly = rdat.PlayerPosition;
//							}
//						}
//					}
//				}
//				if (friendly != nullptr)
//				{
//					//Console.WriteLine("Friendly");
//					//found one, try to get the ball to him
//					if (checkSide(ballVecNorm, make_shared<geometry::CNPoint2D>(friendly->x, friendly->y)))
//					{
//						wiggleDir = 1;
//						//Console.WriteLine("Pos");
//					}
//					else
//					{
//						wiggleDir = -1;
//						//Console.WriteLine("Neg");
//					}
//				}
//				else
//				{
//					//found none, look for free space
//					//						IList<Point2D> ops = WM.GetCurrentOpponentListFiltered();
//					IList < Point2D > ops = WM.GetOpponentListEgoClustered();
//					bool posFree = true;
//					bool negFree = true;
//					double opBallDelta;
//					for (int i = 0; i < ops.Count; i++)
//					{
//						opBallDelta = Math.Abs(ops[i].Angle() - ballAngle);
//						if (opBallDelta > Math.PI)
//							opBallDelta = 2 * Math.PI - opBallDelta;
//						if (ops[i].DistanceTo(ballPos) < 2000 && opBallDelta < Math.PI / 3 * 2)
//						{
//							if (checkSide(ballVecNorm, ops[i]))
//							{
//								posFree = false;
//							}
//							else
//								negFree = false;
//						}
//					}
//					if (posFree)
//					{
//						wiggleDir = 1;
//					}
//					else if (negFree)
//					{
//						wiggleDir = -1;
//					}
//					else
//					{
//						//all occupied
//						if (ownPos == nullptr)
//						{
//							//we have no idea
//							wiggleDir = 1;
//						}
//						else
//						{
//							//try the closest field border
//							shared_ptr<geometry::CNPoint2D> me = make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y);
//							shared_ptr<geometry::CNPoint2D> ballOrth = make_shared<geometry::CNPoint2D>(
//									ballVecNorm->y, -ballVecNorm->x);
//							ballOrth = ballOrth->egoToAllo(*ownPos);
//
//							double dist = field.DistanceToLine(me, ballOrth->angleTo());
//							ballOrth *= -1;
//							if (field->DistanceToLine(me, ballOrth->angleTo()) < dist)
//							{
//								wiggleDir = 1;
//							}
//							else
//							{
//								wiggleDir = -1;
//							}
//						}
//					}
//				}
//			}
//		}
//
//		bm.motion.rotation = rotationVel * wiggleDir;
//		//Correct Angle:
//		double rotating = 0;
//		if (od != nullptr && od->motion != nullptr)
//		{
//			rotating = od->motion.rotation;
//		}
//		//Console.WriteLine("Rot: {0}",rotating);
//		shared_ptr<geometry::CNPoint2D> moveSide = ballPos->rotate(M_PI / 2.0)->normalize();
//		shared_ptr<geometry::CNPoint2D> move = moveSide * 1000 * wiggleDir; // + new Point2D(bm.Motion.Translation*Math.Cos(bm.Motion.Angle),bm.Motion.Translation*Math.Sin(bm.Motion.Angle));
//		bm.motion.angle = move->angleTo();
//		bm.motion.translation = move->length();
//		//bm.Motion.Angle += 15*Math.PI/180*wiggleDir;
//		send(bm);
        /*PROTECTED REGION END*/
    }
    void Tackle::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1434807660243) ENABLED START*/ //Add additional options here
        wiggleDir = 0;
        errorInt = 0;
        maxVel = 4000;
        ballDist = 330;
        ballDistTolerance = 90;
        rotationVel = 2 * M_PI;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1434807660243) ENABLED START*/ //Add additional methods here
    bool checkSide(shared_ptr<geometry::CNPoint2D> lineVector, shared_ptr<geometry::CNPoint2D> pointToCheck)
    {
        double cross = pointToCheck->x * lineVector->y - pointToCheck->y * lineVector->x;
        return (cross < 0);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
