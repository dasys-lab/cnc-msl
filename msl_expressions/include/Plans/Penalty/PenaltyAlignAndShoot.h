#ifndef PenaltyAlignAndShoot_H_
#define PenaltyAlignAndShoot_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1431531496053) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class PenaltyAlignAndShoot : public DomainBehaviour
    {
    public:
        PenaltyAlignAndShoot();
        virtual ~PenaltyAlignAndShoot();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1431531496053) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1431531496053) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1431531496053) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PenaltyAlignAndShoot_H_ */
