#ifndef TestSysconfig_H_
#define TestSysconfig_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1469193509285) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/MovementQuery.h"
/*PROTECTED REGION END*/
namespace alica
{
    class TestSysconfig : public DomainBehaviour
    {
    public:
        TestSysconfig();
        virtual ~TestSysconfig();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1469193509285) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1469193509285) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1469193509285) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* TestSysconfig_H_ */
