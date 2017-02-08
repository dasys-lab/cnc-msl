#ifndef CalibrationTakeBall_H_
#define CalibrationTakeBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1469109429392) ENABLED START*/ //Add additional includes here
#include <msl_robot/dribbleCalibration/container/DribbleCalibrationContainer.h>
/*PROTECTED REGION END*/
namespace alica
{
    class CalibrationTakeBall : public DomainBehaviour
    {
    public:
        CalibrationTakeBall();
        virtual ~CalibrationTakeBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1469109429392) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1469109429392) ENABLED START*/ //Add additional protected methods here
        bool runBehaviour;

        // consts for checkBallRotation()
        enum Rotation
        {
            RotateCorrect, RotateLeft, RotateRight, RotateTooSlow, RotateErr
        };
        msl::DribbleCalibrationContainer dcc;

        bool ballRotateCorrect;
        bool ballHoldCorrect;

        // config parameters
        double speedNoBall;
        double slowTranslationWheelSpeed;
        double minRotation;
        // left and right are swapped
        double dribbleFactorLeft;
        double dribbleFactorRight;

        // for correctWheelSpeed function
        double changingValue;
        enum Operation
        {
            Add, Sub
        };
        Operation operation;
        Operation oldOperation;

        int adaptWheel;

        // for opticalFlow stuff
        shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opQueue;
    int queueSize;

    Rotation checkBallRotation();
    void correctWheelSpeed(Rotation rotation);
    void readConfigParameters();
    void writeConfigParameters();

    // for output
    bool queueFilled;

    /*PROTECTED REGION END*/private:
    /*PROTECTED REGION ID(prv1469109429392) ENABLED START*///Add additional private methods here
    /*PROTECTED REGION END*/};
}
/* namespace alica */

#endif /* CalibrationTakeBall_H_ */
