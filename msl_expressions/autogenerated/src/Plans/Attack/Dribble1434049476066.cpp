#include "Plans/Attack/Dribble1434049476066.h"
using namespace alica;
/*PROTECTED REGION ID(eph1434049476066) ENABLED START*/ //Add additional using directives here
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <Ball.h>
#include <RawSensorData.h>
#include <Robots.h>
#include <msl_robot/kicker/Kicker.h>
#include <obstaclehandler/Obstacles.h>
#include <msl_robot/MSLRobot.h>
#include <Game.h>
#include <nonstd/optional.hpp>
#include <cnc_geometry/CNPointAllo.h>
#include <memory>
#include <cnc_geometry/Calculator.h>
using std::vector;
using geometry::CNPointAllo;
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:Dribble

    //Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) : haveBall 

    /* 
     * Available Vars:
     */
    bool RunTimeCondition1434116267322::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434116267322) ENABLED START*/
        return wm->ball->haveBall();
        /*PROTECTED REGION END*/
    }

    /* generated comment
     
     Task: Attack  -> EntryPoint-ID: 1434049476068

     */
    shared_ptr<UtilityFunction> UtilityFunction1434049476066::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1434049476066) ENABLED START*/

        shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: Dribble in Plan: Dribble

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : may score && Goal closer than PreciseShot.MaxDistance && freeGoalVector exists && noobstacle in turn radius && Distance to goal larger than PrecideShot.MinDistance && AngleToGoal larger than 10degrees 
     *
     * Plans in State: 				
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555 				
     *   - Plan - (Name): DribbleEmergencyKickDefault, (PlanID): 1457706826895 				
     *   - Plan - (Name): DribbleAttackConservativeDefault, (PlanID): 1457967385543 
     *
     * Tasks: 
     *   - Attack (1222613952469) (Entrypoint: 1434049476068)
     *
     * States:
     *   - Dribble (1434049476067)
     *   - AlignToGoal (1434050474119)
     *   - AttackAgain (1434050502701)
     *   - ProtectBall (1434050522682)
     *   - TurnOneEighty (1434050541225)
     *
     * Vars:
     */
    bool TransitionCondition1434050620829::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434050619363) ENABLED START*/
        if (!wm->game->isMayScore())
        {
            return false;
        }
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!ownPos)
        {
            return false;
        }
        //RC-10: no goal shot from own half:
        if (ownPos->x < 0)
        {
            return false;
        }
        double distance = numeric_limits<double>::max();
        const int i = 1;
        nonstd::optional<shared_ptr<const vector<geometry::CNPointAllo, allocator<geometry::CNPointAllo>>> > obs = wm->obstacles->getRawObstaclesAlloBuffer().getLastValidContent();



        auto ballPos = wm->ball->getPositionEgo();
        if (!obs || !ballPos)
        {
            return false;
        }
        double dist = 1300;
        double temp = numeric_limits<double>::max();
        auto right = wm->field->posRightOppGoalPost().normalize() * dist;
        auto left = wm->field->posLeftOppGoalPost().normalize() * dist;
        auto ballAllo = ballPos->toAllo(*ownPos);
        // TODO used to check ego-obstacles in an allo triangle
//        vector<geometry::CNPointTemplate<geometry::CNPointAllo>> test = **obs;

        //TODO the vector **obs is not usable for the method in this way because the vector somehow makes the
        //type unrecognizable/undeductable. need to re-fill a new vector which explicitly sets the type of its
        //elements to geometry::CNPointTemplate<geometry::CNPointAllo>>!
        if (geometry::outsideTriangle(ballAllo, right, left, 450, **obs))
        {
            for (int i = 0; i < (*obs)->size(); i++)
            {
                temp = (*obs)->at(i).distanceTo(ballPos->toAllo(*ownPos));
                if (temp < distance)
                {
                    distance = temp;
                }
            }
        }

        auto goalMid = geometry::CNPointAllo(wm->field->getFieldLength() / 2, 0);

        auto ownPoint = ownPos->getPoint();
        double goalAng = goalMid.angleZToPoint(ownPoint);
        if (abs(goalAng) < 118.0 * M_PI / 180)
        {
            return false;
        }
        double goalDist = ownPos->distanceTo(goalMid);
        auto sc = supplementary::SystemConfig::getInstance();
        double preciseShotMaxDistance = (*sc)["KickHelper"]->get<double>("PreciseShot.MaxDistance", NULL);
        double preciseShotMinDistance = (*sc)["KickHelper"]->get<double>("PreciseShot.MinDistance", NULL);

        return goalDist < preciseShotMaxDistance && goalDist > preciseShotMinDistance
                && this->robot->kicker->getFreeGoalVector() != nonstd::nullopt && distance > 1300;
        /*PROTECTED REGION END*/

    }

    /*
     *
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Fail
     *
     * Plans in State:
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555
     *   - Plan - (Name): DribbleEmergencyKickDefault, (PlanID): 1457706826895
     *   - Plan - (Name): DribbleAttackConservativeDefault, (PlanID): 1457967385543
     *
     * Tasks:
     *   - Attack (1222613952469) (Entrypoint: 1434049476068)
     *
     * States:
     *   - Dribble (1434049476067)
     *   - AlignToGoal (1434050474119)
     *   - AttackAgain (1434050502701)
     *   - ProtectBall (1434050522682)
     *   - TurnOneEighty (1434050541225)
     *
     * Vars:
     */
    bool TransitionCondition1434050649090::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434050647042) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Failed);
        /*PROTECTED REGION END*/

    }

    /*
     *
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : ownPos.X > fieldLength/2 - goalAreaX - 1300 && Abs(ownPos.Y) < penaltyArea.Y
     *
     * Plans in State:
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555
     *   - Plan - (Name): DribbleEmergencyKickDefault, (PlanID): 1457706826895
     *   - Plan - (Name): DribbleAttackConservativeDefault, (PlanID): 1457967385543
     *
     * Tasks:
     *   - Attack (1222613952469) (Entrypoint: 1434049476068)
     *
     * States:
     *   - Dribble (1434049476067)
     *   - AlignToGoal (1434050474119)
     *   - AttackAgain (1434050502701)
     *   - ProtectBall (1434050522682)
     *   - TurnOneEighty (1434050541225)
     *
     * Vars:
     */
    bool TransitionCondition1434050674307::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434050656332) ENABLED START*/
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!ownPos)
        {
            return false;
        }
//        return ownPos->x > field->FieldLength / 2.0 - 1300 - field->GoalAreaLength
//                && abs(ownPos->y) < field->PenaltyAreaWidth / 2.0;
        return wm->field->isInsideOppPenalty(ownPos->getPoint(), 200);
        /*PROTECTED REGION END*/

    }

    /*
     *
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : ProtectBall opponent is in front of us
     *
     * Plans in State:
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555
     *   - Plan - (Name): DribbleEmergencyKickDefault, (PlanID): 1457706826895
     *   - Plan - (Name): DribbleAttackConservativeDefault, (PlanID): 1457967385543
     *
     * Tasks:
     *   - Attack (1222613952469) (Entrypoint: 1434049476068)
     *
     * States:
     *   - Dribble (1434049476067)
     *   - AlignToGoal (1434050474119)
     *   - AttackAgain (1434050502701)
     *   - ProtectBall (1434050522682)
     *   - TurnOneEighty (1434050541225)
     *
     * Vars:
     */
    bool TransitionCondition1434050677358::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434050674494) ENABLED START*/
        auto opp = wm->obstacles->getRawObstaclesEgoBuffer().getLastValidContent();
        if (!opp || (*opp)->size() == 0)
        {
            return false;
        }

        for (geometry::CNPointEgo ego : **opp)
        {
            if (ego.length() < wm->robots->opponents.getOpponentProtectDistance())
            {
                double angDiff = geometry::normalizeAngle(ego.angleZ() - M_PI);

                //Console.WriteLine("angle : " + angDiff);
                if (abs(angDiff) < wm->robots->opponents.getOpponentProtectAngle())
                {
                    return true;
                }
            }
        }
        return false;
        /*PROTECTED REGION END*/

    }

    //State: AlignToGoal in Plan: Dribble

    /*
     *
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Fail || couldn't kick
     *
     * Plans in State:
     *   - Plan - (Name): AlignToGoalDefault, (PlanID): 1415205285582
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555
     *   - Plan - (Name): CheckGoalKickDefault, (PlanID): 1449076029919
     *
     * Tasks:
     *   - Attack (1222613952469) (Entrypoint: 1434049476068)
     *
     * States:
     *   - Dribble (1434049476067)
     *   - AlignToGoal (1434050474119)
     *   - AttackAgain (1434050502701)
     *   - ProtectBall (1434050522682)
     *   - TurnOneEighty (1434050541225)
     *
     * Vars:
     */
    bool TransitionCondition1434050638814::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434050630827) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Failed);
        /*PROTECTED REGION END*/

    }

    //State: AttackAgain in Plan: Dribble

    /*
     *
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : success
     *
     * Plans in State:
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555
     *   - Plan - (Name): DribbleEmergencyKickDefault, (PlanID): 1457706826895
     *   - Plan - (Name): DribbleToAttackPointConservativeDefault, (PlanID): 1458132905432
     *
     * Tasks:
     *   - Attack (1222613952469) (Entrypoint: 1434049476068)
     *
     * States:
     *   - Dribble (1434049476067)
     *   - AlignToGoal (1434050474119)
     *   - AttackAgain (1434050502701)
     *   - ProtectBall (1434050522682)
     *   - TurnOneEighty (1434050541225)
     *
     * Vars:
     */
    bool TransitionCondition1434050643664::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434050639119) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    /*
     *
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Fail
     *
     * Plans in State:
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555
     *   - Plan - (Name): DribbleEmergencyKickDefault, (PlanID): 1457706826895
     *   - Plan - (Name): DribbleToAttackPointConservativeDefault, (PlanID): 1458132905432
     *
     * Tasks:
     *   - Attack (1222613952469) (Entrypoint: 1434049476068)
     *
     * States:
     *   - Dribble (1434049476067)
     *   - AlignToGoal (1434050474119)
     *   - AttackAgain (1434050502701)
     *   - ProtectBall (1434050522682)
     *   - TurnOneEighty (1434050541225)
     *
     * Vars:
     */
    bool TransitionCondition1434050656151::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434050655141) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Failed);
        /*PROTECTED REGION END*/

    }

    /*
     *
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : ProtectBall opponent is in front of us
     *
     * Plans in State:
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555
     *   - Plan - (Name): DribbleEmergencyKickDefault, (PlanID): 1457706826895
     *   - Plan - (Name): DribbleToAttackPointConservativeDefault, (PlanID): 1458132905432
     *
     * Tasks:
     *   - Attack (1222613952469) (Entrypoint: 1434049476068)
     *
     * States:
     *   - Dribble (1434049476067)
     *   - AlignToGoal (1434050474119)
     *   - AttackAgain (1434050502701)
     *   - ProtectBall (1434050522682)
     *   - TurnOneEighty (1434050541225)
     *
     * Vars:
     */
    bool TransitionCondition1434050690800::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434050686620) ENABLED START*/
        auto opp = wm->obstacles->getRawObstaclesEgoBuffer().getLastValidContent();
        if (!opp || (*opp)->size() == 0)
        {
            return false;
        }

        for (geometry::CNPointEgo ego : **opp)
        {
            if (ego.length() < wm->robots->opponents.getOpponentProtectDistance())
            {
                double angDiff = geometry::normalizeAngle(ego.angleZ() - M_PI);

                //Console.WriteLine("angle : " + angDiff);
                if (abs(angDiff) < wm->robots->opponents.getOpponentProtectAngle())
                {
                    return true;
                }
            }
        }
        return false;
        /*PROTECTED REGION END*/

    }

    //State: ProtectBall in Plan: Dribble

    /*
     *
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : any child success
     *
     * Plans in State:
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555
     *   - Plan - (Name): ProtectBallDefault, (PlanID): 1457706612268
     *   - Plan - (Name): DribbleEmergencyKickSafe, (PlanID): 1457706895442
     *
     * Tasks:
     *   - Attack (1222613952469) (Entrypoint: 1434049476068)
     *
     * States:
     *   - Dribble (1434049476067)
     *   - AlignToGoal (1434050474119)
     *   - AttackAgain (1434050502701)
     *   - ProtectBall (1434050522682)
     *   - TurnOneEighty (1434050541225)
     *
     * Vars:
     */
    bool TransitionCondition1434050685640::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434050681521) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    //State: TurnOneEighty in Plan: Dribble

    /*
     *
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Success && Angle to goal < 90 degrees
     *
     * Plans in State:
     *   - Plan - (Name): OneEightyDefault, (PlanID): 1434650910857
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555
     *
     * Tasks:
     *   - Attack (1222613952469) (Entrypoint: 1434049476068)
     *
     * States:
     *   - Dribble (1434049476067)
     *   - AlignToGoal (1434050474119)
     *   - AttackAgain (1434050502701)
     *   - ProtectBall (1434050522682)
     *   - TurnOneEighty (1434050541225)
     *
     * Vars:
     */
    bool TransitionCondition1434050650300::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434050649347) ENABLED START*/
        if (!rp->anyChildrenStatus(PlanStatus::Success))
        {
            return false;
        }
        auto ballPos = wm->ball->getPositionEgo();
        if (!ballPos)
        {
            return false;
        }
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!ownPos)
        {
            return false;
        }
        double dang = geometry::deltaAngle(
                ballPos->angleZ(), geometry::CNPointAllo(wm->field->getFieldLength() / 2, 0.0).toEgo(*ownPos).angleZ());
        return abs(dang) < M_PI / 2.0;
        /*PROTECTED REGION END*/

    }

    /*
     *
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : (Success && angle to goal > 90 degrees) or Fail
     *
     * Plans in State:
     *   - Plan - (Name): OneEightyDefault, (PlanID): 1434650910857
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555
     *
     * Tasks:
     *   - Attack (1222613952469) (Entrypoint: 1434049476068)
     *
     * States:
     *   - Dribble (1434049476067)
     *   - AlignToGoal (1434050474119)
     *   - AttackAgain (1434050502701)
     *   - ProtectBall (1434050522682)
     *   - TurnOneEighty (1434050541225)
     *
     * Vars:
     */
    bool TransitionCondition1434050655008::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434050650481) ENABLED START*/
        auto ballPos = wm->ball->getPositionEgo();
        if (!ballPos)
        {
            return false;
        }
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!ownPos)
        {
            return false;
        }
        double dang = geometry::deltaAngle(
                ballPos->angleZ(), geometry::CNPointAllo(wm->field->getFieldLength() / 2, 0.0).toEgo(*ownPos).angleZ());
        return (rp->anyChildrenStatus(PlanStatus::Success) && abs(dang) >= M_PI / 2.0)
                || rp->anyChildrenStatus(PlanStatus::Failed);
        /*PROTECTED REGION END*/

    }

}
