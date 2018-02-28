#ifndef TestRotationRotation_H_
#define TestRotationRotation_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1492620499435) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class TestRotationRotation : public DomainBehaviour
    {
    public:
        TestRotationRotation();
        virtual ~TestRotationRotation();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1492620499435) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1492620499435) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1492620499435) ENABLED START*/ //Add additional private methods here
        double initialBearing;
        int inited;
        bool halfwayDone;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* TestRotationRotation_H_ */
