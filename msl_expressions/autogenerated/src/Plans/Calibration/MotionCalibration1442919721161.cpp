#include "Plans/Calibration/MotionCalibration1442919721161.h"
using namespace alica;
/*PROTECTED REGION ID(eph1442919721161) ENABLED START*/ //Add additional using directives here
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:MotionCalibration

    /* generated comment
     
     Task: DefaultTask  -> EntryPoint-ID: 1442919721163

     */
    shared_ptr<UtilityFunction> UtilityFunction1442919721161::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1442919721161) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: Stop in Plan: MotionCalibration

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Situation==start 
     *
     * Plans in State: 				
     *   - Plan - (Name): StopDefault, (PlanID): 1413992626194 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1442919721163)
     *
     * States:
     *   - Stop (1442919721162)
     *   - MoveToStartCorner (1442919790374)
     *   - MoveToOtherCorner (1442921032957)
     *   - MoveToMiddle (1443003793160)
     *   - Stop (1443003834928)
     *   - Success (1443522242711)
     *
     * Vars:
     */
    bool TransitionCondition1442919804925::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1442919801497) ENABLED START*/
        return wm->game.checkSituation(msl::Situation::Start);
        /*PROTECTED REGION END*/

    }

    //State: MoveToStartCorner in Plan: MotionCalibration

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): CalcCalibDefault, (PlanID): 1446033354004 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1442919721163)
     *
     * States:
     *   - Stop (1442919721162)
     *   - MoveToStartCorner (1442919790374)
     *   - MoveToOtherCorner (1442921032957)
     *   - MoveToMiddle (1443003793160)
     *   - Stop (1443003834928)
     *   - Success (1443522242711)
     *
     * Vars:
     */
    bool TransitionCondition1442921109582::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1442921106318) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        //return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : situation==stop 
     *
     * Plans in State: 				
     *   - Plan - (Name): CalcCalibDefault, (PlanID): 1446033354004 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1442919721163)
     *
     * States:
     *   - Stop (1442919721162)
     *   - MoveToStartCorner (1442919790374)
     *   - MoveToOtherCorner (1442921032957)
     *   - MoveToMiddle (1443003793160)
     *   - Stop (1443003834928)
     *   - Success (1443522242711)
     *
     * Vars:
     */
    bool TransitionCondition1446733733995::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1446733731468) ENABLED START*/
        return wm->game.checkSituation(msl::Situation::Stop);
        /*PROTECTED REGION END*/

    }

    //State: MoveToOtherCorner in Plan: MotionCalibration

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): DriveTo(-3500,1500), (PlanID): 1443003717671 				
     *   - Plan - (Name): CalcCalibDefault, (PlanID): 1446033354004 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1442919721163)
     *
     * States:
     *   - Stop (1442919721162)
     *   - MoveToStartCorner (1442919790374)
     *   - MoveToOtherCorner (1442921032957)
     *   - MoveToMiddle (1443003793160)
     *   - Stop (1443003834928)
     *   - Success (1443522242711)
     *
     * Vars:
     */
    bool TransitionCondition1443003809289::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1443003805912) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        //return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : situation==stop 
     *
     * Plans in State: 				
     *   - Plan - (Name): DriveTo(-3500,1500), (PlanID): 1443003717671 				
     *   - Plan - (Name): CalcCalibDefault, (PlanID): 1446033354004 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1442919721163)
     *
     * States:
     *   - Stop (1442919721162)
     *   - MoveToStartCorner (1442919790374)
     *   - MoveToOtherCorner (1442921032957)
     *   - MoveToMiddle (1443003793160)
     *   - Stop (1443003834928)
     *   - Success (1443522242711)
     *
     * Vars:
     */
    bool TransitionCondition1446733731395::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1446733729802) ENABLED START*/
        return wm->game.checkSituation(msl::Situation::Stop);
        /*PROTECTED REGION END*/

    }

    //State: MoveToMiddle in Plan: MotionCalibration

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): DriveToMiddle, (PlanID): 1431527260342 				
     *   - Plan - (Name): CalcCalibDefault, (PlanID): 1446033354004 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1442919721163)
     *
     * States:
     *   - Stop (1442919721162)
     *   - MoveToStartCorner (1442919790374)
     *   - MoveToOtherCorner (1442921032957)
     *   - MoveToMiddle (1443003793160)
     *   - Stop (1443003834928)
     *   - Success (1443522242711)
     *
     * Vars:
     */
    bool TransitionCondition1443003847207::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1443003845234) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success) || wm->game.checkSituation(msl::Situation::Stop);
        //return false;
        /*PROTECTED REGION END*/

    }

    //State: Stop in Plan: MotionCalibration

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): StopDefault, (PlanID): 1413992626194 				
     *   - Plan - (Name): CalcCalibDefault, (PlanID): 1446033354004 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1442919721163)
     *
     * States:
     *   - Stop (1442919721162)
     *   - MoveToStartCorner (1442919790374)
     *   - MoveToOtherCorner (1442921032957)
     *   - MoveToMiddle (1443003793160)
     *   - Stop (1443003834928)
     *   - Success (1443522242711)
     *
     * Vars:
     */
    bool TransitionCondition1443522265673::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1443522261454) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

//State: Success in Plan: MotionCalibration

}
