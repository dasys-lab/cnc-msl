#ifndef DribbleToAttackPointConservative_H_
#define DribbleToAttackPointConservative_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1458132872550) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleToAttackPointConservative : public DomainBehaviour
    {
    public:
        DribbleToAttackPointConservative();
        virtual ~DribbleToAttackPointConservative();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1458132872550) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1458132872550) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1458132872550) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleToAttackPointConservative_H_ */
