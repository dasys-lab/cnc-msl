#ifndef ThaoRotateCircle_H_
#define ThaoRotateCircle_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1450104610893) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class ThaoRotateCircle : public DomainBehaviour
    {
    public:
        ThaoRotateCircle();
        virtual ~ThaoRotateCircle();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1450104610893) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1450104610893) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1450104610893) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ThaoRotateCircle_H_ */
