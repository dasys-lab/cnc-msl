#include "Plans/Attack/PassPlay1436268896671.h"
using namespace alica;
/*PROTECTED REGION ID(eph1436268896671) ENABLED START*/ //Add additional using directives here
#include "engine/model/AbstractPlan.h"
#include <MSLWorldModel.h>
#include <msl_robot/kicker/Kicker.h>
#include <MSLFootballField.h>
#include <RawSensorData.h>
#include <Game.h>
#include <obstaclehandler/Obstacles.h>
#include <Ball.h>
#include <msl_robot/MSLRobot.h>
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:PassPlay

    /* generated comment
     
     Task: DefaultTask  -> EntryPoint-ID: 1436268896674

     */
    shared_ptr<UtilityFunction> UtilityFunction1436268896671::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1436268896671) ENABLED START*/

        shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: Dribble in Plan: PassPlay

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Success 
     *
     * Plans in State: 				
     *   - Plan - (Name): SearchForPassPointDefault, (PlanID): 1436269036396 				
     *   - Plan - (Name): ShovelSelectHighKick, (PlanID): 1435156811453 				
     *   - Plan - (Name): Dribble, (PlanID): 1434049476066 				
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1436268896674)
     *
     * States:
     *   - Dribble (1436268896672)
     *   - Pass (1436268931449)
     *
     * Vars:
     */
    bool TransitionCondition1436268944209::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1436268942088) ENABLED START*/
        bool canShoot = true;
        if (wm->game->isMayScore())
        {
            auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
            if (!ownPos)
            {
                return false;
            }
            //RC-10: no goal shot from own half:
            if (ownPos->x < 0)
            {
                canShoot = false;
            }
            else
            {
                double distance = numeric_limits<double>::max();
                auto obs = wm->obstacles->getRawObstaclesAlloBuffer().getLastValidContent();
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
                //TODO this was checking if ego-centric points were inside an allo-centric triangle before
                vector<geometry::CNPointTemplate<geometry::CNPointAllo>> test = (vector<
                        geometry::CNPointTemplate<geometry::CNPointAllo>> )**obs;
                if (geometry::outsideTriangle(ballAllo, right, left, 450, test))
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
                    canShoot = false;
                }
                else
                {
                    double goalDist = ownPos->distanceTo(goalMid);
                    canShoot = goalDist < this->robot->kicker->getPreciseShotMaxDistance()
                            && goalDist > this->robot->kicker->getPreciseShotMinDistance()
                            && this->robot->kicker->getFreeGoalVector() != nonstd::nullopt && distance > 1300;
                }
            }
            if (canShoot)
            {
                return false;
            }
        }

        for (shared_ptr<RunningPlan> rpChild : *rp->getChildren())
        {
            if (rpChild->isBehaviour() && rpChild->getPlan()->getId() == 1436269036396
                    && rpChild->getStatus() == PlanStatus::Success)
            {
                return true;
            }
        }
        return false;
        /*PROTECTED REGION END*/

    }

    //State: Pass in Plan: PassPlay

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : pass fails 
     *
     * Plans in State: 				
     *   - Plan - (Name): AlignAndPassRapidDefault, (PlanID): 1436269080263 				
     *   - Plan - (Name): ShovelSelectLowKick, (PlanID): 1435156714286 				
     *   - Plan - (Name): DribbleControlDefault, (PlanID): 1449742099555 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1436268896674)
     *
     * States:
     *   - Dribble (1436268896672)
     *   - Pass (1436268931449)
     *
     * Vars:
     */
    bool TransitionCondition1436268945305::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1436268944412) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Failed) || rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

}
