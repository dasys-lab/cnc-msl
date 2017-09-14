#ifndef CalcDribbleParams_H_
#define CalcDribbleParams_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1488285993650) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class CalcDribbleParams : public DomainBehaviour
    {
    public:
        CalcDribbleParams();
        virtual ~CalcDribbleParams();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1488285993650) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1488285993650) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1488285993650) ENABLED START*/ //Add additional private methods here
        double epsilonT;
        double velToInput;
        double forwardActuatorSpeed;
        double backwardActuatorSpeed;

        void readConfigParameters();
        void writeConfigParameters();
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CalcDribbleParams_H_ */
