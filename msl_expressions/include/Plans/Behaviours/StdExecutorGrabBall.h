#ifndef StdExecutorGrabBall_H_
#define StdExecutorGrabBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1441209011595) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StdExecutorGrabBall : public DomainBehaviour
    {
    public:
        StdExecutorGrabBall();
        virtual ~StdExecutorGrabBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1441209011595) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1441209011595) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1441209011595) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StdExecutorGrabBall_H_ */
