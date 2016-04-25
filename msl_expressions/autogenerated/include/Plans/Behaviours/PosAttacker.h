#ifndef PosAttacker_H_
#define PosAttacker_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1461574264623) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class PosAttacker : public DomainBehaviour
    {
    public:
        PosAttacker();
        virtual ~PosAttacker();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1461574264623) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1461574264623) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1461574264623) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PosAttacker_H_ */
