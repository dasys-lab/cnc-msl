#ifndef DribbleControl_H_
#define DribbleControl_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1449742071382) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleControl : public DomainBehaviour
    {
    public:
        DribbleControl();
        virtual ~DribbleControl();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1449742071382) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1449742071382) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1449742071382) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleControl_H_ */
