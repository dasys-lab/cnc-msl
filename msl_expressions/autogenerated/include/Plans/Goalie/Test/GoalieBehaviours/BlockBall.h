#ifndef BlockBall_H_
#define BlockBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1447863456983) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class BlockBall : public DomainBehaviour
    {
    public:
        BlockBall();
        virtual ~BlockBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1447863456983) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1447863456983) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1447863456983) ENABLED START*/ //Add additional private methods here
        // TODO: DELETE THIS BECAUSE NOT USED IN GoalieDefault PLAN!!!
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* BlockBall_H_ */
