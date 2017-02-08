#ifndef CalibrationBallHolding_H_
#define CalibrationBallHolding_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1469284294147) ENABLED START*/ //Add additional includes here
#include <msl_robot/dribbleCalibration/container/DribbleCalibrationContainer.h>
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
        bool runBehaviour;

        msl::DribbleCalibrationContainer dcc;
        msl::MovementContainer moveCont;

        double minRotation;
        double slowTranslationWheelSpeed;

        bool ballWasStanding;
        bool ballWasRotating;

        bool ballIsRotating();
        void readConfigParameters();
        void writeConfigParameters();

        shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opQueue;
    int queueSize;

    int changingValue;

private:

    /*PROTECTED REGION END*/private:
    /*PROTECTED REGION ID(prv1469284294147) ENABLED START*/ //Add additional private methods here
    /*PROTECTED REGION END*/};
}
/* namespace alica */

#endif /* CalibrationBallHolding_H_ */
