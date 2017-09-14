#ifndef ShovelSelect_H_
#define ShovelSelect_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1434199834892) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class ShovelSelect : public DomainBehaviour
    {
    public:
        ShovelSelect();
        virtual ~ShovelSelect();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1434199834892) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1434199834892) ENABLED START*/ //Add additional protected methods here
        bool passing;
        bool uncertain;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1434199834892) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ShovelSelect_H_ */
