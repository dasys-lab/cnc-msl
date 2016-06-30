#ifndef ActuatorPassTest_H_
#define ActuatorPassTest_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1467309160739) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class ActuatorPassTest : public DomainBehaviour
    {
    public:
        ActuatorPassTest();
        virtual ~ActuatorPassTest();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1467309160739) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1467309160739) ENABLED START*/ //Add additional protected methods here
        supplementary::SystemConfig* sys;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1467309160739) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ActuatorPassTest_H_ */
