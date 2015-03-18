#include "Plans/GameStrategy/OppStandards/GenericOppStandard1426697871183.h"
using namespace alica;
/*PROTECTED REGION ID(eph1426697871183) ENABLED START*/ //Add additional using directives here
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:GenericOppStandard

    /* generated comment
     
     Task: DefaultTask  -> EntryPoint-ID: 1426697871185

     */
    shared_ptr<UtilityFunction> UtilityFunction1426697871183::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1426697871183) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: Positioning in Plan: GenericOppStandard

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Situation == Start 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1426697871185)
     *
     * States:
     *   - Positioning (1426697871184)
     *   - Execution (1426698446747)
     *   - NewSuccessState (1426698470757)
     *
     * Vars:
     */
    bool TransitionCondition1426698469169::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1426698467862) ENABLED START*/
        return wm->game.checkSituation(msl::Situation::Start);
        /*PROTECTED REGION END*/

    }

    //State: Execution in Plan: GenericOppStandard

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : anyChildSuccess 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1426697871185)
     *
     * States:
     *   - Positioning (1426697871184)
     *   - Execution (1426698446747)
     *   - NewSuccessState (1426698470757)
     *
     * Vars:
     */
    bool TransitionCondition1426698473926::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1426698472708) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

//State: NewSuccessState in Plan: GenericOppStandard

}
