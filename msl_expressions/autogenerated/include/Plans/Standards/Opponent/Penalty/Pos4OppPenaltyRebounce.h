#ifndef Pos4OppPenaltyRebounce_H_
#define Pos4OppPenaltyRebounce_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1466975713750) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class Pos4OppPenaltyRebounce : public DomainBehaviour
    {
    public:
        Pos4OppPenaltyRebounce();
        virtual ~Pos4OppPenaltyRebounce();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1466975713750) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1466975713750) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1466975713750) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Pos4OppPenaltyRebounce_H_ */
