#ifndef PenaltyPosExecuter_H_
#define PenaltyPosExecuter_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1466940407563) ENABLED START*/ //Add additional includes here
namespace msl
{
    class MovementQuery;
}
namespace geometry
{
    class CNPoint2D;
}
/*PROTECTED REGION END*/
namespace alica
{
    class PenaltyPosExecuter : public DomainBehaviour
    {
    public:
        PenaltyPosExecuter();
        virtual ~PenaltyPosExecuter();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1466940407563) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1466940407563) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1466940407563) ENABLED START*/ //Add additional private methods here
        shared_ptr<geometry::CNPoint2D> alloTarget;
        double translation;
        double catchRadius;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PenaltyPosExecuter_H_ */
