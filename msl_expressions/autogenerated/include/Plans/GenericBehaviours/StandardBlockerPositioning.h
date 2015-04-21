#ifndef StandardBlockerPositioning_H_
#define StandardBlockerPositioning_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1429109468816) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardBlockerPositioning : public DomainBehaviour
    {
    public:
        StandardBlockerPositioning();
        virtual ~StandardBlockerPositioning();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1429109468816) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1429109468816) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1429109468816) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardBlockerPositioning_H_ */
