#include "Plans/TestPlans/DribbleTestMOS/TestDribbleMOS1518622787399.h"
using namespace alica;
/*PROTECTED REGION ID(eph1518622787399) ENABLED START*/ //Add additional using directives here
#include <MSLWorldModel.h>
#include <Game.h>
#include <Rules.h>
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:TestDribbleMOS

    /* generated comment
     
     Task: DefaultTask  -> EntryPoint-ID: 1518622787401

     */
    shared_ptr<UtilityFunction> UtilityFunction1518622787399::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1518622787399) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: NewState in Plan: TestDribbleMOS

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : OppGoalKick 
     *
     * Plans in State: 				
     *   - Plan - (Name): StopDefault, (PlanID): 1413992626194 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1518622787401)
     *
     * States:
     *   - NewState (1518622787400)
     *   - DriveZigZagViewZigZag (1518622802933)
     *   - DriveZigZagViewStraight (1518622804332)
     *   - CircleCenter (1518622806277)
     *   - Square (1518622824671)
     *   - DriveStraightViewZigZag (1518623882447)
     *
     * Vars:
     */
    bool TransitionCondition1518622828944::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1518622827729) ENABLED START*/
        return wm->game->checkSituation(msl::Situation::OppGoalkick);
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : OppPenalty 
     *
     * Plans in State: 				
     *   - Plan - (Name): StopDefault, (PlanID): 1413992626194 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1518622787401)
     *
     * States:
     *   - NewState (1518622787400)
     *   - DriveZigZagViewZigZag (1518622802933)
     *   - DriveZigZagViewStraight (1518622804332)
     *   - CircleCenter (1518622806277)
     *   - Square (1518622824671)
     *   - DriveStraightViewZigZag (1518623882447)
     *
     * Vars:
     */
    bool TransitionCondition1518622835957::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1518622834600) ENABLED START*/
        return wm->game->checkSituation(msl::Situation::OppPenalty);
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Situation Parking 
     *
     * Plans in State: 				
     *   - Plan - (Name): StopDefault, (PlanID): 1413992626194 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1518622787401)
     *
     * States:
     *   - NewState (1518622787400)
     *   - DriveZigZagViewZigZag (1518622802933)
     *   - DriveZigZagViewStraight (1518622804332)
     *   - CircleCenter (1518622806277)
     *   - Square (1518622824671)
     *   - DriveStraightViewZigZag (1518623882447)
     *
     * Vars:
     */
    bool TransitionCondition1518622839323::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1518622838067) ENABLED START*/
        return wm->game->checkSituation(msl::Situation::Parking);
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : OppCorner 
     *
     * Plans in State: 				
     *   - Plan - (Name): StopDefault, (PlanID): 1413992626194 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1518622787401)
     *
     * States:
     *   - NewState (1518622787400)
     *   - DriveZigZagViewZigZag (1518622802933)
     *   - DriveZigZagViewStraight (1518622804332)
     *   - CircleCenter (1518622806277)
     *   - Square (1518622824671)
     *   - DriveStraightViewZigZag (1518623882447)
     *
     * Vars:
     */
    bool TransitionCondition1518622846035::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1518622844428) ENABLED START*/
        return wm->game->checkSituation(msl::Situation::OppCorner);
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): StopDefault, (PlanID): 1413992626194 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1518622787401)
     *
     * States:
     *   - NewState (1518622787400)
     *   - DriveZigZagViewZigZag (1518622802933)
     *   - DriveZigZagViewStraight (1518622804332)
     *   - CircleCenter (1518622806277)
     *   - Square (1518622824671)
     *   - DriveStraightViewZigZag (1518623882447)
     *
     * Vars:
     */
    bool TransitionCondition1518623968811::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1518623967163) ENABLED START*/
        return wm->game->checkSituation(msl::Situation::OppFreekick);
        /*PROTECTED REGION END*/

    }

    //State: DriveZigZagViewZigZag in Plan: TestDribbleMOS

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): DriveZigZagViewZigZag, (PlanID): 1518623186077 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1518622787401)
     *
     * States:
     *   - NewState (1518622787400)
     *   - DriveZigZagViewZigZag (1518622802933)
     *   - DriveZigZagViewStraight (1518622804332)
     *   - CircleCenter (1518622806277)
     *   - Square (1518622824671)
     *   - DriveStraightViewZigZag (1518623882447)
     *
     * Vars:
     */
    bool TransitionCondition1518622830640::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1518622829104) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    //State: DriveZigZagViewStraight in Plan: TestDribbleMOS

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): DriveZigZagViewStraight, (PlanID): 1518623233217 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1518622787401)
     *
     * States:
     *   - NewState (1518622787400)
     *   - DriveZigZagViewZigZag (1518622802933)
     *   - DriveZigZagViewStraight (1518622804332)
     *   - CircleCenter (1518622806277)
     *   - Square (1518622824671)
     *   - DriveStraightViewZigZag (1518623882447)
     *
     * Vars:
     */
    bool TransitionCondition1518622834399::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1518622830792) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    //State: CircleCenter in Plan: TestDribbleMOS

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): CircleCenter, (PlanID): 1518624635783 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1518622787401)
     *
     * States:
     *   - NewState (1518622787400)
     *   - DriveZigZagViewZigZag (1518622802933)
     *   - DriveZigZagViewStraight (1518622804332)
     *   - CircleCenter (1518622806277)
     *   - Square (1518622824671)
     *   - DriveStraightViewZigZag (1518623882447)
     *
     * Vars:
     */
    bool TransitionCondition1518622837786::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1518622836117) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    //State: Square in Plan: TestDribbleMOS

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): InitialForwardBackward, (PlanID): 1519032290449 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1518622787401)
     *
     * States:
     *   - NewState (1518622787400)
     *   - DriveZigZagViewZigZag (1518622802933)
     *   - DriveZigZagViewStraight (1518622804332)
     *   - CircleCenter (1518622806277)
     *   - Square (1518622824671)
     *   - DriveStraightViewZigZag (1518623882447)
     *
     * Vars:
     */
    bool TransitionCondition1518622847338::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1518622846236) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    //State: DriveStraightViewZigZag in Plan: TestDribbleMOS

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Situation OppFreeKick 
     *
     * Plans in State: 				
     *   - Plan - (Name): DriveStraightViewZigZag, (PlanID): 1518628181064 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1518622787401)
     *
     * States:
     *   - NewState (1518622787400)
     *   - DriveZigZagViewZigZag (1518622802933)
     *   - DriveZigZagViewStraight (1518622804332)
     *   - CircleCenter (1518622806277)
     *   - Square (1518622824671)
     *   - DriveStraightViewZigZag (1518623882447)
     *
     * Vars:
     */
    bool TransitionCondition1518623970678::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1518623968955) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

}
