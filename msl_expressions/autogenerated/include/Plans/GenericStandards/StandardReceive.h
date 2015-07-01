#ifndef StandardReceive_H_
#define StandardReceive_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1428509505186) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardReceive : public DomainBehaviour
    {
    public:
        StandardReceive();
        virtual ~StandardReceive();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1428509505186) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1428509505186) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1428509505186) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardReceive_H_ */
