#ifndef SpinSlowly_H_
#define SpinSlowly_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1435159253296) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class SpinSlowly : public DomainBehaviour
    {
    public:
        SpinSlowly();
        virtual ~SpinSlowly();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1435159253296) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1435159253296) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1435159253296) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* SpinSlowly_H_ */
