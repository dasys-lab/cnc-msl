#include "Plans/GameStrategy/Other/DropBallPositioning1455537014534.h"
using namespace alica;
/*PROTECTED REGION ID(eph1455537014534) ENABLED START*/ //Add additional using directives here
#include "PriorityList.h"
#include "engine/model/Plan.h"
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:DropBallPositioning

    /* generated comment
     
     Task: Attack  -> EntryPoint-ID: 1455537014536

     Task: AttackSupport  -> EntryPoint-ID: 1455537247542

     Task: DefendSupport  -> EntryPoint-ID: 1455537250535

     Task: Defend  -> EntryPoint-ID: 1455537253704

     */
    shared_ptr<UtilityFunction> UtilityFunction1455537014534::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1455537014534) ENABLED START*/
        vector<long> entryPoints;

        entryPoints.push_back(1455537014536); // Attack
        entryPoints.push_back(1455537247542); // AttackSupport
        entryPoints.push_back(1455537250535); // DefendSupport
        entryPoints.push_back(1455537253704); // Defend

        PriorityList* us = new PriorityList(2.0, "OtherPlan", 2, entryPoints);
        list<USummand*> utilSummands;
        utilSummands.push_back(us);
        shared_ptr < UtilityFunction > function = make_shared < UtilityFunction
                > ("OtherPlanUtilTest", utilSummands, 0.1, 0.2, plan);
        return function;
        /*PROTECTED REGION END*/
    }

//State: AttackerPos in Plan: DropBallPositioning

//State: CoverSpace in Plan: DropBallPositioning

//State: CoverSpaceDefensive in Plan: DropBallPositioning

//State: Defend in Plan: DropBallPositioning

}
