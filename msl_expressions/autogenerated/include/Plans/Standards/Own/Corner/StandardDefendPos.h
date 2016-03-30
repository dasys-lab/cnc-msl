#ifndef StandardDefendPos_H_
#define StandardDefendPos_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1459355042700) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardDefendPos : public DomainBehaviour
    {
    public:
        StandardDefendPos();
        virtual ~StandardDefendPos();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1459355042700) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1459355042700) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1459355042700) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardDefendPos_H_ */
