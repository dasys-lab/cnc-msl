#include "Plans/GenericStandards/GenericExecute1431522123418.h"
using namespace alica;
/*PROTECTED REGION ID(eph1431522123418) ENABLED START*/ //Add additional using directives here
#include "PriorityList.h"
#include "DistBallRobot.h"
#include "engine/model/Plan.h"
#include "engine/Assignment.h"
#include "engine/collections/StateCollection.h"
#include <MSLWorldModel.h>
#include <Ball.h>
#include <Game.h>
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:GenericExecute

    //Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :  

    /* 
     * Available Vars:
     */
    bool RunTimeCondition1457955744730::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1457955744730) ENABLED START*/
        return true;
        /*PROTECTED REGION END*/
    }

    /* generated comment
     
     Task: ExecuteStandard  -> EntryPoint-ID: 1431522155980

     Task: ReceiveStandard  -> EntryPoint-ID: 1431522269326

     Task: Blocker  -> EntryPoint-ID: 1431523395534

     Task: Defend  -> EntryPoint-ID: 1431523422152

     */
    shared_ptr<UtilityFunction> UtilityFunction1431522123418::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1431522123418) ENABLED START*/
        vector<long> entryPoints;

        entryPoints.push_back(1431522155980); // ExecuteStandard
        entryPoints.push_back(1431522269326); // ReceiveStandard
        entryPoints.push_back(1431523395534); // Blocker
        entryPoints.push_back(1431523422152); // Defend

        PriorityList* us = new PriorityList(2.0, "OtherPlan", 2, entryPoints);
        list<USummand*> utilSummands;
        utilSummands.push_back(us);
        shared_ptr < UtilityFunction > function = make_shared < UtilityFunction
                > ("OtherPlanUtilTest", utilSummands, 0.1, 0.2, plan);
        return function;
        /*PROTECTED REGION END*/
    }

    //State: GrabBall in Plan: GenericExecute

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : grab ball success 
     *
     * Plans in State: 				
     *   - Plan - (Name): ShovelSelectLowKick, (PlanID): 1435156714286 				
     *   - Plan - (Name): StandardAlignAndGrabGenericExecute, (PlanID): 1455888617961 				
     *   - Plan - (Name): StandardActuateDefault, (PlanID): 1435766278023 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1431522155980)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1431522269326)
     *   - Blocker (1432209050494) (Entrypoint: 1431523395534)
     *   - Defend (1225115406909) (Entrypoint: 1431523422152)
     *
     * States:
     *   - GrabBall (1431522155979)
     *   - AlignReceiver (1431522297705)
     *   - Pass (1431522763494)
     *   - Receive (1431522912251)
     *   - Success (1431522995646)
     *   - Block (1431523482646)
     *   - Defend (1431524014799)
     *   - SpatialDefend (1431524769489)
     *   - AlignExecutor (1433949667740)
     *
     * Vars:
     */
    bool TransitionCondition1431522783626::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1431522782044) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    //State: AlignReceiver in Plan: GenericExecute

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : aligned && robot in SpatialDefend 
     *
     * Plans in State: 				
     *   - Plan - (Name): StandardAlignToPointReceiver, (PlanID): 1433950043262 				
     *   - Plan - (Name): StandardActuateDefault, (PlanID): 1435766278023 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1431522155980)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1431522269326)
     *   - Blocker (1432209050494) (Entrypoint: 1431523395534)
     *   - Defend (1225115406909) (Entrypoint: 1431523422152)
     *
     * States:
     *   - GrabBall (1431522155979)
     *   - AlignReceiver (1431522297705)
     *   - Pass (1431522763494)
     *   - Receive (1431522912251)
     *   - Success (1431522995646)
     *   - Block (1431523482646)
     *   - Defend (1431524014799)
     *   - SpatialDefend (1431524769489)
     *   - AlignExecutor (1433949667740)
     *
     * Vars:
     */
    bool TransitionCondition1431522922124::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1431522920716) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success)
                && rp->getAssignment()->getRobotStateMapping()->getRobotsInState(1431524769489).size() > 0;
        /*PROTECTED REGION END*/

    }

    //State: Pass in Plan: GenericExecute

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : executed 
     *
     * Plans in State: 				
     *   - Plan - (Name): ShovelSelectLowKick, (PlanID): 1435156714286 				
     *   - Plan - (Name): StandardActuateDefault, (PlanID): 1435766278023 				
     *   - Plan - (Name): GenericExecutePassDefault, (PlanID): 1465040471344 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1431522155980)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1431522269326)
     *   - Blocker (1432209050494) (Entrypoint: 1431523395534)
     *   - Defend (1225115406909) (Entrypoint: 1431523422152)
     *
     * States:
     *   - GrabBall (1431522155979)
     *   - AlignReceiver (1431522297705)
     *   - Pass (1431522763494)
     *   - Receive (1431522912251)
     *   - Success (1431522995646)
     *   - Block (1431523482646)
     *   - Defend (1431524014799)
     *   - SpatialDefend (1431524769489)
     *   - AlignExecutor (1433949667740)
     *
     * Vars:
     */
    bool TransitionCondition1431524871023::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1431524869870) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : lostBall 
     *
     * Plans in State: 				
     *   - Plan - (Name): ShovelSelectLowKick, (PlanID): 1435156714286 				
     *   - Plan - (Name): StandardActuateDefault, (PlanID): 1435766278023 				
     *   - Plan - (Name): GenericExecutePassDefault, (PlanID): 1465040471344 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1431522155980)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1431522269326)
     *   - Blocker (1432209050494) (Entrypoint: 1431523395534)
     *   - Defend (1225115406909) (Entrypoint: 1431523422152)
     *
     * States:
     *   - GrabBall (1431522155979)
     *   - AlignReceiver (1431522297705)
     *   - Pass (1431522763494)
     *   - Receive (1431522912251)
     *   - Success (1431522995646)
     *   - Block (1431523482646)
     *   - Defend (1431524014799)
     *   - SpatialDefend (1431524769489)
     *   - AlignExecutor (1433949667740)
     *
     * Vars:
     */
    bool TransitionCondition1435761870069::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1435761866545) ENABLED START*/
        return !wm->ball->haveBall();
        /*PROTECTED REGION END*/

    }

    //State: Receive in Plan: GenericExecute

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : haveball 
     *
     * Plans in State: 				
     *   - Plan - (Name): ShovelSelectHighKick, (PlanID): 1435156811453 				
     *   - Plan - (Name): InterceptDefault, (PlanID): 1458757193843 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1431522155980)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1431522269326)
     *   - Blocker (1432209050494) (Entrypoint: 1431523395534)
     *   - Defend (1225115406909) (Entrypoint: 1431523422152)
     *
     * States:
     *   - GrabBall (1431522155979)
     *   - AlignReceiver (1431522297705)
     *   - Pass (1431522763494)
     *   - Receive (1431522912251)
     *   - Success (1431522995646)
     *   - Block (1431523482646)
     *   - Defend (1431524014799)
     *   - SpatialDefend (1431524769489)
     *   - AlignExecutor (1433949667740)
     *
     * Vars:
     */
    bool TransitionCondition1431523013533::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1431523011459) ENABLED START*/
        return this->isTimeOut(750000000, rp->getStateStartTime(), rp);
        /*PROTECTED REGION END*/

    }

    //State: Success in Plan: GenericExecute

    //State: Block in Plan: GenericExecute

    //State: Defend in Plan: GenericExecute

    //State: SpatialDefend in Plan: GenericExecute

    //State: AlignExecutor in Plan: GenericExecute

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : aligned  and situation start 
     *
     * Plans in State: 				
     *   - Plan - (Name): StandardAlignToPointExecutor, (PlanID): 1435155363994 				
     *   - Plan - (Name): ShovelSelectLowKick, (PlanID): 1435156714286 
     *
     * Tasks: 
     *   - ExecuteStandard (1439997010902) (Entrypoint: 1431522155980)
     *   - ReceiveStandard (1439997023446) (Entrypoint: 1431522269326)
     *   - Blocker (1432209050494) (Entrypoint: 1431523395534)
     *   - Defend (1225115406909) (Entrypoint: 1431523422152)
     *
     * States:
     *   - GrabBall (1431522155979)
     *   - AlignReceiver (1431522297705)
     *   - Pass (1431522763494)
     *   - Receive (1431522912251)
     *   - Success (1431522995646)
     *   - Block (1431523482646)
     *   - Defend (1431524014799)
     *   - SpatialDefend (1431524769489)
     *   - AlignExecutor (1433949667740)
     *
     * Vars:
     */
    bool TransitionCondition1433949707598::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1433949706015) ENABLED START*/
        return (rp->anyChildrenStatus(PlanStatus::Success) && wm->game->checkSituation(msl::Situation::Start))
                || (wm->game->checkSituation(msl::Situation::Start)
                        && wm->getTime() - wm->game->getTimeSinceStart() > 8000000000);
        /*PROTECTED REGION END*/

    }

}
