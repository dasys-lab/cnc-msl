#ifndef MeasureAndConfigure_H_
#define MeasureAndConfigure_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1507131462459) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class MeasureAndConfigure : public DomainBehaviour
    {
    public:
        MeasureAndConfigure();
        virtual ~MeasureAndConfigure();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1507131462459) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1507131462459) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1507131462459) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* MeasureAndConfigure_H_ */
