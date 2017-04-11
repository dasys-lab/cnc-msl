#include "Plans/Defence/MidfieldDefense1458033329973.h"
using namespace alica;
/*PROTECTED REGION ID(eph1458033329973) ENABLED START*/ //Add additional using directives here
#include <MSLWorldModel.h>
#include <Ball.h>
#include <Game.h>
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:MidfieldDefense

    /* generated comment
     
     Task: DefaultTask  -> EntryPoint-ID: 1458033358326

     */
    shared_ptr<UtilityFunction> UtilityFunction1458033329973::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1458033329973) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: MidFieldBlock in Plan: MidfieldDefense

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : MidFieldBlock2ReleaseMid: Blocking failed or is successfull & Attacker is in opp half 
     *
     * Plans in State: 				
     *   - Plan - (Name): MidfieldBlock, (PlanID): 1458033620834 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1458033358326)
     *
     * States:
     *   - MidFieldBlock (1458033358325)
     *   - ReleaseMid (1458033385978)
     *   - ReleaseOwnHalf (1458033395158)
     *
     * Vars:
     */
    bool TransitionCondition1458033411271::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1458033410354) ENABLED START*/
        return rp->anyChildrenStatus(alica::PlanStatus::Success) || wm->game->getGameState() == msl::GameState::Duel;
        /*PROTECTED REGION END*/

    }

    //State: ReleaseMid in Plan: MidfieldDefense

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : ReleaseMid2ReleaseOwnHalf: 
     *
     * Plans in State: 				
     *   - Plan - (Name): ReleaseMidDefault, (PlanID): 1458033497042 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1458033358326)
     *
     * States:
     *   - MidFieldBlock (1458033358325)
     *   - ReleaseMid (1458033385978)
     *   - ReleaseOwnHalf (1458033395158)
     *
     * Vars:
     */
    bool TransitionCondition1458033412464::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1458033411608) ENABLED START*/
        if (wm->ball->getAlloBallPosition() && wm->ball->getAlloBallPosition()->x < 1000.0)
        {
            return true;
        }
        return false;
        /*PROTECTED REGION END*/

    }

    //State: ReleaseOwnHalf in Plan: MidfieldDefense

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : ReleaseOwnHalf2RleaseMid 
     *
     * Plans in State: 				
     *   - Plan - (Name): ReleaseOwnHalf, (PlanID): 1458033644590 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1458033358326)
     *
     * States:
     *   - MidFieldBlock (1458033358325)
     *   - ReleaseMid (1458033385978)
     *   - ReleaseOwnHalf (1458033395158)
     *
     * Vars:
     */
    bool TransitionCondition1458033413418::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1458033412731) ENABLED START*/
        return wm->ball->getAlloBallPosition() && wm->ball->getAlloBallPosition()->x > 1100.0;
        /*PROTECTED REGION END*/

    }

}
