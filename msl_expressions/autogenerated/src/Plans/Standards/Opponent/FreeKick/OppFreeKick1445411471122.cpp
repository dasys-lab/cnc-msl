#include "Plans/Standards/Opponent/FreeKick/OppFreeKick1445411471122.h"
using namespace alica;
/*PROTECTED REGION ID(eph1445411471122) ENABLED START*/ //Add additional using directives here
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:OppFreeKick

    //Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :  

    /* 
     * Available Vars:
     */
    bool RunTimeCondition1445442215438::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1445442215438) ENABLED START*/
        return true;
        /*PROTECTED REGION END*/
    }

    /* generated comment
     
     Task: DefaultTask  -> EntryPoint-ID: 1445411471124

     */
    shared_ptr<UtilityFunction> UtilityFunction1445411471122::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1445411471122) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: PosDef in Plan: OppFreeKick

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : any children success 
     *
     * Plans in State: 				
     *   - Plan - (Name): Pos4DefDefault, (PlanID): 1445438204426 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1445411471124)
     *
     * States:
     *   - PosDef (1445411471123)
     *   - Success (1447875657650)
     *
     * Vars:
     */
    bool TransitionCondition1447875675479::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1447875673956) ENABLED START*/
    	return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

//State: Success in Plan: OppFreeKick

}
