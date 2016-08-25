#ifndef CalibrationDribbleRotation_H_
#define CalibrationDribbleRotation_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1469196350730) ENABLED START*/ //Add additional includes here
#include <Plans/DribbleCalibration/Container/DribbleCalibrationContainer.h>
/*PROTECTED REGION END*/
namespace alica
{
    class CalibrationDribbleRotation : public DomainBehaviour
    {
    public:
        CalibrationDribbleRotation();
        virtual ~CalibrationDribbleRotation();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1469196350730) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1469196350730) ENABLED START*/ //Add additional protected methods here
        DribbleCalibrationContainer dcc;
        double curveRotationfactor;

        void readConfigParameters();
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1469196350730) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CalibrationDribbleRotation_H_ */
