#ifndef ProtectBall_H_
#define ProtectBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1457706592232) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class ProtectBall : public DomainBehaviour
    {
    public:
        ProtectBall();
        virtual ~ProtectBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1457706592232) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1457706592232) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1457706592232) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ProtectBall_H_ */
