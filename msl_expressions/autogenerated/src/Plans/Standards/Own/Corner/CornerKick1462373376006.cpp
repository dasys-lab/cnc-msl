#include "Plans/Standards/Own/Corner/CornerKick1462373376006.h"
using namespace alica;
/*PROTECTED REGION ID(eph1462373376006) ENABLED START*/ //Add additional using directives here
#include "engine/model/Plan.h"
#include "engine/Assignment.h"
#include "engine/collections/StateCollection.h"
#include "BallXIntervall.h"
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <Game.h>
#include <Ball.h>
#include <Rules.h>
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:CornerKick

    //Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :  

    /* 
     * Available Vars:
     */
    bool RunTimeCondition1464793807994::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1464793807994) ENABLED START*/
        return true;
        /*PROTECTED REGION END*/
    }

    /* generated comment
     
     Task: ExecuteStandard  -> EntryPoint-ID: 1462373376008

     Task: ReceiveStandard  -> EntryPoint-ID: 1462373457908

     Task: Block  -> EntryPoint-ID: 1464793555239

     Task: Defend  -> EntryPoint-ID: 1464793619930

     */
    shared_ptr<UtilityFunction> UtilityFunction1462373376006::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1462373376006) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: AlignExec in Plan: CornerKick

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : (child success && start) || (start && timeSinceStart > timeUntilEmergencyExecute) 
     *
     * Plans in State: 				
     *   - Plan - (Name): StandardActuateDefault, (PlanID): 1435766278023 				
     *   - Plan - (Name): ShovelSelectLowKick, (PlanID): 1435156714286 				
     *   - Plan - (Name): StandardAlignToPointExecutor, (PlanID): 1435155363994 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1462373376008)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1462373457908)
     *   - Block (1461237765109) (Entrypoint: 1464793555239)
     *   - Defend (1225115406909) (Entrypoint: 1464793619930)
     *
     * States:
     *   - AlignExec (1462373376007)
     *   - AlignRecv (1462374515826)
     *   - GrabBall (1464783052747)
     *   - Pass (1464787959998)
     *   - Wait (1464788034326)
     *   - Recieve (1464792231235)
     *   - Success (1464792294365)
     *   - Block (1464793576624)
     *   - Defend (1464793641082)
     *
     * Vars:
     */
    bool TransitionCondition1464784053079::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1464784050959) ENABLED START*/
        return (rp->anyChildrenStatus(PlanStatus::Success) && wm->game->checkSituation(msl::Situation::Start))
                || (wm->game->checkSituation(msl::Situation::Start)
                        && (this->isTimeOut(this->timeUntilEmergencyExecute, wm->game->getTimeSinceStart(), rp)));
        /*PROTECTED REGION END*/

    }

    //State: AlignRecv in Plan: CornerKick

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : pm for own id 
     *
     * Plans in State: 				
     *   - Plan - (Name): ShovelSelectHighKick, (PlanID): 1435156811453 				
     *   - Plan - (Name): Pos4ReceiverCornerKick, (PlanID): 1464787549220 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1462373376008)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1462373457908)
     *   - Block (1461237765109) (Entrypoint: 1464793555239)
     *   - Defend (1225115406909) (Entrypoint: 1464793619930)
     *
     * States:
     *   - AlignExec (1462373376007)
     *   - AlignRecv (1462374515826)
     *   - GrabBall (1464783052747)
     *   - Pass (1464787959998)
     *   - Wait (1464788034326)
     *   - Recieve (1464792231235)
     *   - Success (1464792294365)
     *   - Block (1464793576624)
     *   - Defend (1464793641082)
     *
     * Vars:
     */
    bool TransitionCondition1464792277148::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1464792274211) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success)
                && rp->getAssignment()->getRobotStateMapping()->getRobotsInState(1464788034326).size() > 0;
        /*PROTECTED REGION END*/

    }

    //State: GrabBall in Plan: CornerKick

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : has ball 
     *
     * Plans in State: 				
     *   - Plan - (Name): StandardActuateDefault, (PlanID): 1435766278023 				
     *   - Plan - (Name): ShovelSelectLowKick, (PlanID): 1435156714286 				
     *   - Plan - (Name): StandardAlignAndGrabCornerKick, (PlanID): 1465038982091 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1462373376008)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1462373457908)
     *   - Block (1461237765109) (Entrypoint: 1464793555239)
     *   - Defend (1225115406909) (Entrypoint: 1464793619930)
     *
     * States:
     *   - AlignExec (1462373376007)
     *   - AlignRecv (1462374515826)
     *   - GrabBall (1464783052747)
     *   - Pass (1464787959998)
     *   - Wait (1464788034326)
     *   - Recieve (1464792231235)
     *   - Success (1464792294365)
     *   - Block (1464793576624)
     *   - Defend (1464793641082)
     *
     * Vars:
     */
    bool TransitionCondition1464787990119::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1464787987943) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    //State: Pass in Plan: CornerKick

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : lost Ball 
     *
     * Plans in State: 				
     *   - Plan - (Name): ShovelSelectLowKick, (PlanID): 1435156714286 				
     *   - Plan - (Name): StandardPassDefault, (PlanID): 1435760175843 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1462373376008)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1462373457908)
     *   - Block (1461237765109) (Entrypoint: 1464793555239)
     *   - Defend (1225115406909) (Entrypoint: 1464793619930)
     *
     * States:
     *   - AlignExec (1462373376007)
     *   - AlignRecv (1462374515826)
     *   - GrabBall (1464783052747)
     *   - Pass (1464787959998)
     *   - Wait (1464788034326)
     *   - Recieve (1464792231235)
     *   - Success (1464792294365)
     *   - Block (1464793576624)
     *   - Defend (1464793641082)
     *
     * Vars:
     */
    bool TransitionCondition1464788000901::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1464787998856) ENABLED START*/
        return !wm->ball->haveBall();
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : executed 
     *
     * Plans in State: 				
     *   - Plan - (Name): ShovelSelectLowKick, (PlanID): 1435156714286 				
     *   - Plan - (Name): StandardPassDefault, (PlanID): 1435760175843 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1462373376008)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1462373457908)
     *   - Block (1461237765109) (Entrypoint: 1464793555239)
     *   - Defend (1225115406909) (Entrypoint: 1464793619930)
     *
     * States:
     *   - AlignExec (1462373376007)
     *   - AlignRecv (1462374515826)
     *   - GrabBall (1464783052747)
     *   - Pass (1464787959998)
     *   - Wait (1464788034326)
     *   - Recieve (1464792231235)
     *   - Success (1464792294365)
     *   - Block (1464793576624)
     *   - Defend (1464793641082)
     *
     * Vars:
     */
    bool TransitionCondition1464788060652::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1464788058735) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    //State: Wait in Plan: CornerKick

    //State: Recieve in Plan: CornerKick

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : success 
     *
     * Plans in State: 				
     *   - Plan - (Name): InterceptCarefullyDefault, (PlanID): 1427703234654 				
     *   - Plan - (Name): ShovelSelectHighKick, (PlanID): 1435156811453 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1462373376008)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1462373457908)
     *   - Block (1461237765109) (Entrypoint: 1464793555239)
     *   - Defend (1225115406909) (Entrypoint: 1464793619930)
     *
     * States:
     *   - AlignExec (1462373376007)
     *   - AlignRecv (1462374515826)
     *   - GrabBall (1464783052747)
     *   - Pass (1464787959998)
     *   - Wait (1464788034326)
     *   - Recieve (1464792231235)
     *   - Success (1464792294365)
     *   - Block (1464793576624)
     *   - Defend (1464793641082)
     *
     * Vars:
     */
    bool TransitionCondition1464796139312::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1464796134632) ENABLED START*/
        return wm->ball->haveBall() || this->isTimeOut(this->rules->getStandbyTime() * 0.75, rp->getStateStartTime(), rp);
        /*PROTECTED REGION END*/

    }

//State: Success in Plan: CornerKick

//State: Block in Plan: CornerKick

//State: Defend in Plan: CornerKick

}
