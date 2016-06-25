#ifndef InterceptCarefully_H_
#define InterceptCarefully_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1427703218101) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class InterceptCarefully : public DomainBehaviour
    {
    public:
        InterceptCarefully();
        virtual ~InterceptCarefully();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1427703218101) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1427703218101) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1427703218101) ENABLED START*/ //Add additional private methods here
        double interceptCarfullyRotateP;
        double defaultTranslation;
        double catchRadius;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* InterceptCarefully_H_ */
