#ifndef StandardPass_H_
#define StandardPass_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1435760160067) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardPass : public DomainBehaviour
    {
    public:
        StandardPass();
        virtual ~StandardPass();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1435760160067) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1435760160067) ENABLED START*/ //Add additional protected methods here
        string taskName;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1435760160067) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardPass_H_ */
