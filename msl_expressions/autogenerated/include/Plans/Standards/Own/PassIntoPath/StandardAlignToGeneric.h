#ifndef StandardAlignToGeneric_H_
#define StandardAlignToGeneric_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1457531616421) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardAlignToGeneric : public DomainBehaviour
    {
    public:
        StandardAlignToGeneric();
        virtual ~StandardAlignToGeneric();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1457531616421) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1457531616421) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1457531616421) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardAlignToGeneric_H_ */
