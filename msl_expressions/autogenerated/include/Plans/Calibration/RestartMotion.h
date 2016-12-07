#ifndef RestartMotion_H_
#define RestartMotion_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1472657511112) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class RestartMotion : public DomainBehaviour
    {
    public:
        RestartMotion();
        virtual ~RestartMotion();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1472657511112) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1472657511112) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1472657511112) ENABLED START*/ //Add additional private methods here
        int uninitializedMotionCounter;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* RestartMotion_H_ */
