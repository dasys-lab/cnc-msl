#ifndef CalibrationTakeBall_H_
#define CalibrationTakeBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1469109429392) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class CalibrationTakeBall : public DomainBehaviour
    {
    public:
        CalibrationTakeBall();
        virtual ~CalibrationTakeBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1469109429392) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1469109429392) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1469109429392) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CalibrationTakeBall_H_ */
