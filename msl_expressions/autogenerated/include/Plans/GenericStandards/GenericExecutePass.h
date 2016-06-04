#ifndef GenericExecutePass_H_
#define GenericExecutePass_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1465040441324) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class GenericExecutePass : public DomainBehaviour
    {
    public:
        GenericExecutePass();
        virtual ~GenericExecutePass();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1465040441324) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1465040441324) ENABLED START*/ //Add additional protected methods here
        string taskName;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1465040441324) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* GenericExecutePass_H_ */
