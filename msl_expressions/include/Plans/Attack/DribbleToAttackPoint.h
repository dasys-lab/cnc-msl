#ifndef DribbleToAttackPoint_H_
#define DribbleToAttackPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1436855838589) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleToAttackPoint : public DomainBehaviour
    {
    public:
        DribbleToAttackPoint();
        virtual ~DribbleToAttackPoint();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1436855838589) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1436855838589) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1436855838589) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleToAttackPoint_H_ */
