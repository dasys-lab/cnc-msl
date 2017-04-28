#ifndef StandardShieldBall_H_
#define StandardShieldBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1455888661866) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardShieldBall : public DomainBehaviour
    {
    public:
        StandardShieldBall();
        virtual ~StandardShieldBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1455888661866) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1455888661866) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1455888661866) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardShieldBall_H_ */
