#ifndef TestMotorControl_H_
#define TestMotorControl_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1482163964536) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Configuration.h>
#include <limits>
#include <supplementary/InformationElement.h>
/*PROTECTED REGION END*/
namespace alica
{
    class TestMotorControl : public DomainBehaviour
    {
    public:
        TestMotorControl();
        virtual ~TestMotorControl();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1482163964536) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1482163964536) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1482163964536) ENABLED START*/ //Add additional private methods here
        int count;
        int testSpeed;
        int angle;
        supplementary::InfoTime startTime;

        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* TestMotorControl_H_ */
