#ifndef PositionReceiverFreeKickOppHalf_H_
#define PositionReceiverFreeKickOppHalf_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1464780799716) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
/*PROTECTED REGION END*/
namespace alica
{
    class PositionReceiverFreeKickOppHalf : public DomainBehaviour
    {
    public:
        PositionReceiverFreeKickOppHalf();
        virtual ~PositionReceiverFreeKickOppHalf();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1464780799716) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1464780799716) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1464780799716) ENABLED START*/ //Add additional private methods here
        shared_ptr<geometry::CNPoint2D> alloTarget;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PositionReceiverFreeKickOppHalf_H_ */
