#ifndef StandardStdDefendPositioning_H_
#define StandardStdDefendPositioning_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1429110495141) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardStdDefendPositioning : public DomainBehaviour
    {
    public:
        StandardStdDefendPositioning();
        virtual ~StandardStdDefendPositioning();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1429110495141) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1429110495141) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1429110495141) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardStdDefendPositioning_H_ */
