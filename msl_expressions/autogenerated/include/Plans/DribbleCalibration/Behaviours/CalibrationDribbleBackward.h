#ifndef CalibrationDribbleBackward_H_
#define CalibrationDribbleBackward_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1469196252478) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class CalibrationDribbleBackward : public DomainBehaviour
    {
    public:
        CalibrationDribbleBackward();
        virtual ~CalibrationDribbleBackward();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1469196252478) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1469196252478) ENABLED START*/ //Add additional protected methods here
        void readConfigParameters();
//        void writeConfigParameters();
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1469196252478) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CalibrationDribbleBackward_H_ */
