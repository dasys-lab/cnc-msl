#ifndef StdExecuterPos_H_
#define StdExecuterPos_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1428508102191) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StdExecuterPos : public DomainBehaviour
    {
    public:
        StdExecuterPos();
        virtual ~StdExecuterPos();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1428508102191) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1428508102191) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1428508102191) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StdExecuterPos_H_ */
