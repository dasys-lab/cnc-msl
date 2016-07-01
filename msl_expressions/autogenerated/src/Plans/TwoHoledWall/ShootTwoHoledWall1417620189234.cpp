#include "Plans/TwoHoledWall/ShootTwoHoledWall1417620189234.h"
using namespace alica;
/*PROTECTED REGION ID(eph1417620189234) ENABLED START*/ //Add additional using directives here
#include <MSLWorldModel.h>
#include <Ball.h>
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:ShootTwoHoledWall

    /* generated comment
     
     Task: DefaultTask  -> EntryPoint-ID: 1417620209051

     */
    shared_ptr<UtilityFunction> UtilityFunction1417620189234::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1417620189234) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: GrabBall in Plan: ShootTwoHoledWall

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : haveBall 
     *
     * Plans in State: 				
     *   - Plan - (Name): InterceptCarefullyDefault, (PlanID): 1427703234654 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1417620209051)
     *
     * States:
     *   - GrabBall (1417620209050)
     *   - AlignAndShoot (1417620225739)
     *   - NewSuccessState (1417620275486)
     *
     * Vars:
     */
    bool TransitionCondition1417620269159::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1417620267846) ENABLED START*/
        return wm->ball->haveBall();
        /*PROTECTED REGION END*/

    }

    //State: AlignAndShoot in Plan: ShootTwoHoledWall

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : anyChildSuccess 
     *
     * Plans in State: 				
     *   - Plan - (Name): AlignAndShootTwoHoledWallDefault, (PlanID): 1417620730939 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1417620209051)
     *
     * States:
     *   - GrabBall (1417620209050)
     *   - AlignAndShoot (1417620225739)
     *   - NewSuccessState (1417620275486)
     *
     * Vars:
     */
    bool TransitionCondition1417620286821::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1417620285804) ENABLED START*/
	//cout << "ShootTwoWallTransiton: Check any child success: "<< rp->anyChildrenStatus(PlanStatus::Success) << endl;
        return rp->anyChildrenStatus(PlanStatus::Success);
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : notHaveBall 
     *
     * Plans in State: 				
     *   - Plan - (Name): AlignAndShootTwoHoledWallDefault, (PlanID): 1417620730939 
     *
     * Tasks: 
     *   - DefaultTask (1225112227903) (Entrypoint: 1417620209051)
     *
     * States:
     *   - GrabBall (1417620209050)
     *   - AlignAndShoot (1417620225739)
     *   - NewSuccessState (1417620275486)
     *
     * Vars:
     */
    bool TransitionCondition1417620329181::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1417620327753) ENABLED START*/
        return !wm->ball->haveBall();
        /*PROTECTED REGION END*/

    }

//State: NewSuccessState in Plan: ShootTwoHoledWall

}