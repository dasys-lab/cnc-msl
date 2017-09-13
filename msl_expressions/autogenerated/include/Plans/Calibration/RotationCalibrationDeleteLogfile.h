#ifndef RotationCalibrationDeleteLogfile_H_
#define RotationCalibrationDeleteLogfile_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1479315274633) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class RotationCalibrationDeleteLogfile : public DomainBehaviour
    {
    public:
        RotationCalibrationDeleteLogfile();
        virtual ~RotationCalibrationDeleteLogfile();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1479315274633) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1479315274633) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1479315274633) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* RotationCalibrationDeleteLogfile_H_ */
