#ifndef CalibrationBallHolding_H_
#define CalibrationBallHolding_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1469284294147) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class CalibrationBallHolding : public DomainBehaviour
    {
    public:
        CalibrationBallHolding();
        virtual ~CalibrationBallHolding();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1469284294147) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1469284294147) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1469284294147) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CalibrationBallHolding_H_ */
