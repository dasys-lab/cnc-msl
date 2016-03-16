#ifndef PassKickIntoPath_H_
#define PassKickIntoPath_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1457531678043) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class PassKickIntoPath : public DomainBehaviour
    {
    public:
        PassKickIntoPath();
        virtual ~PassKickIntoPath();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1457531678043) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1457531678043) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1457531678043) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PassKickIntoPath_H_ */
