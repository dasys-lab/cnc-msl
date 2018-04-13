#ifndef TestThrowInPassAlignmentMaster_H_
#define TestThrowInPassAlignmentMaster_H_

#include "DomainCondition.h"
#include "engine/BasicUtilityFunction.h"
#include "engine/UtilityFunction.h"
#include "engine/DefaultUtilityFunction.h"
/*PROTECTED REGION ID(incl1523356580220) ENABLED START*/
//Add inlcudes here
/*PROTECTED REGION END*/
using namespace alica;

namespace alicaAutogenerated
{
    /*PROTECTED REGION ID(meth1523356580220) ENABLED START*/
    //Add other things here
    /*PROTECTED REGION END*/
    class UtilityFunction1523356580220 : public BasicUtilityFunction
    {
        shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    };

    class TransitionCondition1523356618875 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

    class TransitionCondition1523356628773 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

} /* namespace alica */

#endif