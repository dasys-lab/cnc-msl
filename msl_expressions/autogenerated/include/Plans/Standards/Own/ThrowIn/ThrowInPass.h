#ifndef ThrowInPass_H_
#define ThrowInPass_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462363192018) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class ThrowInPass : public DomainBehaviour
    {
    public:
        ThrowInPass();
        virtual ~ThrowInPass();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1462363192018) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1462363192018) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1462363192018) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ThrowInPass_H_ */
