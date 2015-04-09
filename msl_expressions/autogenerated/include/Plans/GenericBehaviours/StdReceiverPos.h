#ifndef StdReceiverPos_H_
#define StdReceiverPos_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1428507924151) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StdReceiverPos : public DomainBehaviour
    {
    public:
        StdReceiverPos();
        virtual ~StdReceiverPos();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1428507924151) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1428507924151) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1428507924151) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StdReceiverPos_H_ */
