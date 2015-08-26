#ifndef AlignExecutor_H_
#define AlignExecutor_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1440600472019) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class AlignExecutor : public DomainBehaviour
    {
    public:
        AlignExecutor();
        virtual ~AlignExecutor();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1440600472019) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1440600472019) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1440600472019) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlignExecutor_H_ */
