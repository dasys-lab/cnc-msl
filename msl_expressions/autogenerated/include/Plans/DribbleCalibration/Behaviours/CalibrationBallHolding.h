#ifndef CalibrationBallHolding_H_
#define CalibrationBallHolding_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1469284294147) ENABLED START*/ //Add additional includes here
#include <Plans/DribbleCalibration/Container/DribbleCalibrationContainer.h>
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
        DribbleCalibrationContainer dcc;

        double minRotation;
        double slowTranslationWheelSpeed;

        bool ballWasStanding;
        bool ballWasRotating;

        bool ballIsRotating();
        void readConfigParameters();
        void writeConfigParameters();

        vector<shared_ptr<geometry::CNPoint2D>> opQueue;
        bool queueFilled;
        bool opQueueFilled();
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1469284294147) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CalibrationBallHolding_H_ */
