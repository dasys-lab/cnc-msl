#ifndef BouncePassFinishAlign_H_
#define BouncePassFinishAlign_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1459357041857) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class BouncePassFinishAlign : public DomainBehaviour
    {
    public:
        BouncePassFinishAlign();
        virtual ~BouncePassFinishAlign();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1459357041857) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1459357041857) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1459357041857) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* BouncePassFinishAlign_H_ */
