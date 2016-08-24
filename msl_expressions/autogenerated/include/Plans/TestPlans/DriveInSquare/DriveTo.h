#ifndef DriveTo_H_
#define DriveTo_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1472045093826) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class DriveTo : public DomainBehaviour
    {
    public:
        DriveTo();
        virtual ~DriveTo();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1472045093826) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1472045093826) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1472045093826) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveTo_H_ */
