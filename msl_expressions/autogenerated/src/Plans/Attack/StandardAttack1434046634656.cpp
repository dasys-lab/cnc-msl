#include "Plans/Attack/StandardAttack1434046634656.h"
using namespace alica;
/*PROTECTED REGION ID(eph1434046634656) ENABLED START*/ //Add additional using directives here
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:StandardAttack

    /* generated comment
     
     Task: Attack  -> EntryPoint-ID: 1434046634658

     */
    shared_ptr<UtilityFunction> UtilityFunction1434046634656::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1434046634656) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: GetBall in Plan: StandardAttack

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : haveBall 
     *
     * Plans in State: 				
     *   - Plan - (Name): GetBallDefault, (PlanID): 1414840399972 
     *
     * Tasks: 
     *   - Attack (1222613952469) (Entrypoint: 1434046634658)
     *
     * States:
     *   - GetBall (1434046634657)
     *   - Tackle (1434048406725)
     *   - HaveBall (1434048705508)
     *   - LostBall (1434715893346)
     *
     * Vars:
     */
    bool TransitionCondition1434048722207::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434048720937) ENABLED START*/
        return wm->ball.haveBall();
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : !haveBall && enemy has ball 
     *
     * Plans in State: 				
     *   - Plan - (Name): GetBallDefault, (PlanID): 1414840399972 
     *
     * Tasks: 
     *   - Attack (1222613952469) (Entrypoint: 1434046634658)
     *
     * States:
     *   - GetBall (1434046634657)
     *   - Tackle (1434048406725)
     *   - HaveBall (1434048705508)
     *   - LostBall (1434715893346)
     *
     * Vars:
     */
    bool TransitionCondition1434048729350::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434048723878) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : doesnt see ball 
     *
     * Plans in State: 				
     *   - Plan - (Name): GetBallDefault, (PlanID): 1414840399972 
     *
     * Tasks: 
     *   - Attack (1222613952469) (Entrypoint: 1434046634658)
     *
     * States:
     *   - GetBall (1434046634657)
     *   - Tackle (1434048406725)
     *   - HaveBall (1434048705508)
     *   - LostBall (1434715893346)
     *
     * Vars:
     */
    bool TransitionCondition1434716048353::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434716047438) ENABLED START*/
        return wm->rawSensorData.getBallPosition() == nullptr;;
        /*PROTECTED REGION END*/

    }

    //State: Tackle in Plan: StandardAttack

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : haveBall && enemy not close 
     *
     * Plans in State: 				
     *   - Plan - (Name): Tackle, (PlanID): 1434116965565 
     *
     * Tasks: 
     *   - Attack (1222613952469) (Entrypoint: 1434046634658)
     *
     * States:
     *   - GetBall (1434046634657)
     *   - Tackle (1434048406725)
     *   - HaveBall (1434048705508)
     *   - LostBall (1434715893346)
     *
     * Vars:
     */
    bool TransitionCondition1434048732966::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434048731525) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : haveball && enemy not close 
     *
     * Plans in State: 				
     *   - Plan - (Name): Tackle, (PlanID): 1434116965565 
     *
     * Tasks: 
     *   - Attack (1222613952469) (Entrypoint: 1434046634658)
     *
     * States:
     *   - GetBall (1434046634657)
     *   - Tackle (1434048406725)
     *   - HaveBall (1434048705508)
     *   - LostBall (1434715893346)
     *
     * Vars:
     */
    bool TransitionCondition1434048737070::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434048734889) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: HaveBall in Plan: StandardAttack

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): PassPlay, (PlanID): 1436268896671 
     *
     * Tasks: 
     *   - Attack (1222613952469) (Entrypoint: 1434046634658)
     *
     * States:
     *   - GetBall (1434046634657)
     *   - Tackle (1434048406725)
     *   - HaveBall (1434048705508)
     *   - LostBall (1434715893346)
     *
     * Vars:
     */
    bool TransitionCondition1434048723635::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434048722503) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : haveBall && enemy close 
     *
     * Plans in State: 				
     *   - Plan - (Name): PassPlay, (PlanID): 1436268896671 
     *
     * Tasks: 
     *   - Attack (1222613952469) (Entrypoint: 1434046634658)
     *
     * States:
     *   - GetBall (1434046634657)
     *   - Tackle (1434048406725)
     *   - HaveBall (1434048705508)
     *   - LostBall (1434715893346)
     *
     * Vars:
     */
    bool TransitionCondition1434048731181::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434048729645) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : doesnt see ball 
     *
     * Plans in State: 				
     *   - Plan - (Name): PassPlay, (PlanID): 1436268896671 
     *
     * Tasks: 
     *   - Attack (1222613952469) (Entrypoint: 1434046634658)
     *
     * States:
     *   - GetBall (1434046634657)
     *   - Tackle (1434048406725)
     *   - HaveBall (1434048705508)
     *   - LostBall (1434715893346)
     *
     * Vars:
     */
    bool TransitionCondition1434716050319::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434716049424) ENABLED START*/
        return wm->rawSensorData.getBallPosition() == nullptr;
        /*PROTECTED REGION END*/

    }

    //State: LostBall in Plan: StandardAttack

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : found ball 
     *
     * Plans in State: 				
     *   - Plan - (Name): WanderDefault, (PlanID): 1434716230628 
     *
     * Tasks: 
     *   - Attack (1222613952469) (Entrypoint: 1434046634658)
     *
     * States:
     *   - GetBall (1434046634657)
     *   - Tackle (1434048406725)
     *   - HaveBall (1434048705508)
     *   - LostBall (1434715893346)
     *
     * Vars:
     */
    bool TransitionCondition1434716047150::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434716045767) ENABLED START*/
        return wm->rawSensorData.getBallPosition() != nullptr;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : found ball 
     *
     * Plans in State: 				
     *   - Plan - (Name): WanderDefault, (PlanID): 1434716230628 
     *
     * Tasks: 
     *   - Attack (1222613952469) (Entrypoint: 1434046634658)
     *
     * States:
     *   - GetBall (1434046634657)
     *   - Tackle (1434048406725)
     *   - HaveBall (1434048705508)
     *   - LostBall (1434715893346)
     *
     * Vars:
     */
    bool TransitionCondition1434716049299::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1434716048579) ENABLED START*/
        return wm->rawSensorData.getBallPosition() != nullptr;
        /*PROTECTED REGION END*/

    }

}
