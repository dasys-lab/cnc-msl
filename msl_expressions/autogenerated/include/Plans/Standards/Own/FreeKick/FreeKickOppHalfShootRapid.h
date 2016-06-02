#ifndef FreeKickOppHalfShootRapid_H_
#define FreeKickOppHalfShootRapid_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1464787649214) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class FreeKickOppHalfShootRapid : public DomainBehaviour
    {
    public:
        FreeKickOppHalfShootRapid();
        virtual ~FreeKickOppHalfShootRapid();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1464787649214) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1464787649214) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1464787649214) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* FreeKickOppHalfShootRapid_H_ */
