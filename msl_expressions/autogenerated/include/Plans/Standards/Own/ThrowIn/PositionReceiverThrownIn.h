#ifndef PositionReceiverThrownIn_H_
#define PositionReceiverThrownIn_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1461584204507) ENABLED START*/ //Add additional includes here
namespace geometry
{
    class CNPoint2D;
}
namespace msl
{
    class MovementQuery;
}
;
/*PROTECTED REGION END*/
namespace alica
{
    class PositionReceiverThrownIn : public DomainBehaviour
    {
    public:
        PositionReceiverThrownIn();
        virtual ~PositionReceiverThrownIn();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1461584204507) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1461584204507) ENABLED START*/ //Add additional protected methods here
        shared_ptr<msl::MovementQuery> mQuery;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1461584204507) ENABLED START*/ //Add additional private methods here
        shared_ptr<geometry::CNPoint2D> alloTarget;
        double positionDistanceTolerance;
        double ballDistRec;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PositionReceiverThrownIn_H_ */
