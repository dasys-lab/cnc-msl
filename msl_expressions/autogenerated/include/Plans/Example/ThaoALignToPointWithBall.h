#ifndef ThaoALignToPointWithBall_H_
#define ThaoALignToPointWithBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1454519765751) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class ThaoALignToPointWithBall : public DomainBehaviour
    {
    public:
        ThaoALignToPointWithBall();
        virtual ~ThaoALignToPointWithBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1454519765751) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1454519765751) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1454519765751) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ThaoALignToPointWithBall_H_ */
