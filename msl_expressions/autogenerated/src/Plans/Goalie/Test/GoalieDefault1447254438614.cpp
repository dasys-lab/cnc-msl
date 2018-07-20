#include "Plans/Goalie/Test/GoalieDefault1447254438614.h"
using namespace alica;
/*PROTECTED REGION ID(eph1447254438614) ENABLED START*/ //Add additional using directives here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <cmath>
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:GoalieDefault

    /* generated comment
     
     Task: KeepGoal  -> EntryPoint-ID: 1447254438616

     */
    shared_ptr<UtilityFunction> UtilityFunction1447254438614::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1447254438614) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: DriveToGoal in Plan: GoalieDefault

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : situation == Goalie inside GoalArea 
     *
     * Plans in State: 				
     *   - Plan - (Name): DriveToGoalDefault, (PlanID): 1447863442558 
     *
     * Tasks: 
     *   - KeepGoal (1221754402444) (Entrypoint: 1447254438616)
     *
     * States:
     *   - DriveToGoal (1447254438615)
     *   - WatchBall (1447255061404)
     *
     * Vars:
     */
    bool TransitionCondition1447255447830::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1447255446546) ENABLED START*/
        if (rp->anyChildrenStatus(PlanStatus::Success))
        {
            cout << "--- DriveToGoal finished ---" << endl;
            return true;
        }
        else
            return false;
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

//State: WatchBall in Plan: GoalieDefault

}
