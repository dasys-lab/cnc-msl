#ifndef Wander_H_
#define Wander_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1434716215423) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class Wander : public DomainBehaviour
    {
    public:
        Wander();
        virtual ~Wander();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1434716215423) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1434716215423) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1434716215423) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Wander_H_ */
