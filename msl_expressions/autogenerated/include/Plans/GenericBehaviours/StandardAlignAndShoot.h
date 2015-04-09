#ifndef StandardAlignAndShoot_H_
#define StandardAlignAndShoot_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1428508999190) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardAlignAndShoot : public DomainBehaviour
    {
    public:
        StandardAlignAndShoot();
        virtual ~StandardAlignAndShoot();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1428508999190) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1428508999190) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1428508999190) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardAlignAndShoot_H_ */
