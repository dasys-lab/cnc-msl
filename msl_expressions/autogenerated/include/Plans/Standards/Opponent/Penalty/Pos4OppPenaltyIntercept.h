#ifndef Pos4OppPenaltyIntercept_H_
#define Pos4OppPenaltyIntercept_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1466975753516) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class Pos4OppPenaltyIntercept : public DomainBehaviour
    {
    public:
        Pos4OppPenaltyIntercept();
        virtual ~Pos4OppPenaltyIntercept();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1466975753516) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1466975753516) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1466975753516) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Pos4OppPenaltyIntercept_H_ */
