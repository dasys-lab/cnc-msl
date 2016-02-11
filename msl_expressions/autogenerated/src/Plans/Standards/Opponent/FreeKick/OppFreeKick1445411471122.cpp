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
     
     Task: Defend  -> EntryPoint-ID: 1445411471124

     Task: Blocker  -> EntryPoint-ID: 1454663032454

     Task: Attack  -> EntryPoint-ID: 1454663045348

     */
    shared_ptr<UtilityFunction> UtilityFunction1445411471122::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1445411471122) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);


        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: PosDefDefender in Plan: OppFreeKick

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : any children success 
     *
     * Plans in State: 				
     *   - Plan - (Name): Pos4DefDefault, (PlanID): 1445438204426 
     *
     * Tasks: 
     *   - Defend (1225115406909) (Entrypoint: 1445411471124)
     *   - Blocker (1432209050494) (Entrypoint: 1454663032454)
     *   - Attack (1222613952469) (Entrypoint: 1454663045348)
     *
     * States:
     *   - PosDefDefender (1445411471123)
     *   - Success (1447875657650)
     *   - PosDefBlocker (1454663055945)
     *   - PosDefAttacker (1454663058990)
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

    //State: PosDefBlocker in Plan: OppFreeKick

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): Pos4DefDefault, (PlanID): 1445438204426 
     *
     * Tasks: 
     *   - Defend (1225115406909) (Entrypoint: 1445411471124)
     *   - Blocker (1432209050494) (Entrypoint: 1454663032454)
     *   - Attack (1222613952469) (Entrypoint: 1454663045348)
     *
     * States:
     *   - PosDefDefender (1445411471123)
     *   - Success (1447875657650)
     *   - PosDefBlocker (1454663055945)
     *   - PosDefAttacker (1454663058990)
     *
     * Vars:
     */
    bool TransitionCondition1454663210272::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1454663208360) ENABLED START*/
    	return rp->anyChildrenStatus(PlanStatus::Success);
    	/*PROTECTED REGION END*/

    }

    //State: PosDefAttacker in Plan: OppFreeKick

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): Pos4DefDefault, (PlanID): 1445438204426 
     *
     * Tasks: 
     *   - Defend (1225115406909) (Entrypoint: 1445411471124)
     *   - Blocker (1432209050494) (Entrypoint: 1454663032454)
     *   - Attack (1222613952469) (Entrypoint: 1454663045348)
     *
     * States:
     *   - PosDefDefender (1445411471123)
     *   - Success (1447875657650)
     *   - PosDefBlocker (1454663055945)
     *   - PosDefAttacker (1454663058990)
     *
     * Vars:
     */
    bool TransitionCondition1454663213143::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1454663210633) ENABLED START*/
    	return rp->anyChildrenStatus(PlanStatus::Success);
    	/*PROTECTED REGION END*/

    }

}