#ifndef StayAwayAfterGamePenalty_H_
#define StayAwayAfterGamePenalty_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1466940583686) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StayAwayAfterGamePenalty : public DomainBehaviour
    {
    public:
        StayAwayAfterGamePenalty();
        virtual ~StayAwayAfterGamePenalty();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1466940583686) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1466940583686) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1466940583686) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StayAwayAfterGamePenalty_H_ */
