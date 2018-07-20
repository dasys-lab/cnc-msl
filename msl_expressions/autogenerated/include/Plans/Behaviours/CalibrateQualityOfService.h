#ifndef CalibrateQualityOfService_H_
#define CalibrateQualityOfService_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1435320085836) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class CalibrateQualityOfService : public DomainBehaviour
    {
    public:
        CalibrateQualityOfService();
        virtual ~CalibrateQualityOfService();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1435320085836) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1435320085836) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1435320085836) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CalibrateQualityOfService_H_ */
