#ifndef CalcDribbleParams_H_
#define CalcDribbleParams_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1489492250448) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class CalcDribbleParams : public DomainBehaviour
    {
    public:
        CalcDribbleParams();
        virtual ~CalcDribbleParams();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1489492250448) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1489492250448) ENABLED START*/ //Add additional protected methods here
        void readConfigParams();
        void writeConfigParams();
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1489492250448) ENABLED START*/ //Add additional private methods here
        double measuredForwardSpeed;
        double measuredBackwardSpeed;

        double calibTrans;

        double velToInput;
        double epsilonT;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CalcDribbleParams_H_ */
