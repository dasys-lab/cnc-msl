#ifndef BounceShotAlignPasser_H_
#define BounceShotAlignPasser_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1459354938404) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class BounceShotAlignPasser : public DomainBehaviour
    {
    public:
        BounceShotAlignPasser();
        virtual ~BounceShotAlignPasser();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1459354938404) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1459354938404) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1459354938404) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* BounceShotAlignPasser_H_ */
