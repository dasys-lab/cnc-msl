#ifndef StdStandardDefendPos_H_
#define StdStandardDefendPos_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1428508153151) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StdStandardDefendPos : public DomainBehaviour
    {
    public:
        StdStandardDefendPos();
        virtual ~StdStandardDefendPos();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1428508153151) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1428508153151) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1428508153151) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StdStandardDefendPos_H_ */
