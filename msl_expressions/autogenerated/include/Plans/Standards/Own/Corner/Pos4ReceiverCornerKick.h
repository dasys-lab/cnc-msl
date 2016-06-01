#ifndef Pos4ReceiverCornerKick_H_
#define Pos4ReceiverCornerKick_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1464787469281) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class Pos4ReceiverCornerKick : public DomainBehaviour
    {
    public:
        Pos4ReceiverCornerKick();
        virtual ~Pos4ReceiverCornerKick();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1464787469281) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1464787469281) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1464787469281) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Pos4ReceiverCornerKick_H_ */
