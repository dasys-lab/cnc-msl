#ifndef PointToPoint_H_
#define PointToPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1489068164649) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
/*PROTECTED REGION END*/
namespace alica
{
    class PointToPoint : public DomainBehaviour
    {
    public:
        PointToPoint();
        virtual ~PointToPoint();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1489068164649) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1489068164649) ENABLED START*/ //Add additional protected methods here
        msl::MovementQuery query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1489068164649) ENABLED START*/ //Add additional private methods here
        bool toOwnPentalty;
        geometry::CNPointEgo egoTarget;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PointToPoint_H_ */
