#ifndef MoveToPointDynamic_H_
#define MoveToPointDynamic_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1456997073100) ENABLED START*/ //Add additional includes here
namespace msl {
	class MovementQuery;
}
/*PROTECTED REGION END*/
namespace alica
{
	class Query;
    class MoveToPointDynamic : public DomainBehaviour
    {
    public:
        MoveToPointDynamic();
        virtual ~MoveToPointDynamic();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1456997073100) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1456997073100) ENABLED START*/ //Add additional protected methods here
        shared_ptr<Query> query;
        double maxVel;
        bool avoidBall;
        vector<double> result;
        ulong lastResult;
        shared_ptr<msl::MovementQuery> movQuery;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1456997073100) ENABLED START*/ //Add additional private methods here
        void readConfigParameters();
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* MoveToPointDynamic_H_ */
