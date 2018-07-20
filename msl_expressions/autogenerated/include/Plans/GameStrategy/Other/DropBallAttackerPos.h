#ifndef DropBallAttackerPos_H_
#define DropBallAttackerPos_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1455537841488) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
/*PROTECTED REGION END*/
namespace alica
{
    class DropBallAttackerPos : public DomainBehaviour
    {
    public:
        DropBallAttackerPos();
        virtual ~DropBallAttackerPos();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1455537841488) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1455537841488) ENABLED START*/ //Add additional protected methods here
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1455537841488) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DropBallAttackerPos_H_ */
