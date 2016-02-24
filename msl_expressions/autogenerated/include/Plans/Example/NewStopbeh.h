#ifndef NewStopbeh_H_
#define NewStopbeh_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1449767981309) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class NewStopbeh : public DomainBehaviour
    {
    public:
        NewStopbeh();
        virtual ~NewStopbeh();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1449767981309) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1449767981309) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1449767981309) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* NewStopbeh_H_ */
