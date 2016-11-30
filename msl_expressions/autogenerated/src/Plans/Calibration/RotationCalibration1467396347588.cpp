#include "Plans/Calibration/RotationCalibration1467396347588.h"
using namespace alica;
/*PROTECTED REGION ID(eph1467396347588) ENABLED START*/ //Add additional using directives here
#include <MSLWorldModel.h>
#include <Game.h>
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:RotationCalibration

    /* generated comment
     
     Task: DefaultTask  -> EntryPoint-ID: 1467396347590

     */
    shared_ptr<UtilityFunction> UtilityFunction1467396347588::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1467396347588) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: Stop in Plan: RotationCalibration

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Status Start? 
     *
     * Plans in State: 				
     *   - Plan - (Name): RotationCalibrationDeleteLogfileDefault, (PlanID): 1479315306711 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1467396347590)
     *
     * States:
     *   - Stop (1467396347589)
     *   - Rotating (1467396438734)
     *   - Finished (1470227765155)
     *   - Return (1470237789517)
     *   - Calculation (1475074211872)
     *
     * Vars:
     */
    bool TransitionCondition1467396619848::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1467396616225) ENABLED START*/
        return wm->game->checkSituation(msl::Situation::Start);
        /*PROTECTED REGION END*/

    }

    //State: Rotating in Plan: RotationCalibration

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Rotation finished 
     *
     * Plans in State: 				
     *   - Plan - (Name): RotateOnceDefault, (PlanID): 1467398000539 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1467396347590)
     *
     * States:
     *   - Stop (1467396347589)
     *   - Rotating (1467396438734)
     *   - Finished (1470227765155)
     *   - Return (1470237789517)
     *   - Calculation (1475074211872)
     *
     * Vars:
     */
    bool TransitionCondition1470227880114::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1470227878581) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    //State: Finished in Plan: RotationCalibration

    //State: Return in Plan: RotationCalibration

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : INSTANT SUCCESS!!! 
     *
     * Plans in State: 				
     *   - Plan - (Name): RestartMotionDefault, (PlanID): 1472657588489 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1467396347590)
     *
     * States:
     *   - Stop (1467396347589)
     *   - Rotating (1467396438734)
     *   - Finished (1470227765155)
     *   - Return (1470237789517)
     *   - Calculation (1475074211872)
     *
     * Vars:
     */
    bool TransitionCondition1470237805501::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1470237803234) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    //State: Calculation in Plan: RotationCalibration

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Robot radius precise enough? 
     *
     * Plans in State: 				
     *   - Plan - (Name): RotationCalibrationCalculationDefault, (PlanID): 1475074454339 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1467396347590)
     *
     * States:
     *   - Stop (1467396347589)
     *   - Rotating (1467396438734)
     *   - Finished (1470227765155)
     *   - Return (1470237789517)
     *   - Calculation (1475074211872)
     *
     * Vars:
     */
    bool TransitionCondition1475074488561::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1475074486547) ENABLED START*/
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Calculate to Restart 
     *
     * Plans in State: 				
     *   - Plan - (Name): RotationCalibrationCalculationDefault, (PlanID): 1475074454339 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1467396347590)
     *
     * States:
     *   - Stop (1467396347589)
     *   - Rotating (1467396438734)
     *   - Finished (1470227765155)
     *   - Return (1470237789517)
     *   - Calculation (1475074211872)
     *
     * Vars:
     */
    bool TransitionCondition1480520550306::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1480520547022) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

}
