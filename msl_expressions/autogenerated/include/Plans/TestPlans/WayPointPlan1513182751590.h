#ifndef WayPointPlan_H_
#define WayPointPlan_H_

#include "DomainCondition.h"
#include "engine/BasicUtilityFunction.h"
#include "engine/UtilityFunction.h"
#include "engine/DefaultUtilityFunction.h"
/*PROTECTED REGION ID(incl1513182751590) ENABLED START*/
//Add inlcudes here
/*PROTECTED REGION END*/
using namespace alica;

namespace alicaAutogenerated
{
    /*PROTECTED REGION ID(meth1513182751590) ENABLED START*/
    //Add other things here
    /*PROTECTED REGION END*/
    class UtilityFunction1513182751590 : public BasicUtilityFunction
    {
        shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    };

    class TransitionCondition1513182955846 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

    class TransitionCondition1513182957882 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

    class TransitionCondition1513182959777 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

    class TransitionCondition1513182968659 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

    class TransitionCondition1513182971720 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

    class TransitionCondition1513182984479 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

    class TransitionCondition1513182981413 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

    class TransitionCondition1513182974957 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

    class TransitionCondition1513182977188 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

    class TransitionCondition1513182979288 : public DomainCondition
    {
        bool evaluate(shared_ptr<RunningPlan> rp);
    };

} /* namespace alica */

#endif
