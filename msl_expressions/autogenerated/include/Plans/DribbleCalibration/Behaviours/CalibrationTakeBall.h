#ifndef CalibrationTakeBall_H_
#define CalibrationTakeBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1469109429392) ENABLED START*/ //Add additional includes here
#include <Plans/DribbleCalibration/Container/DribbleCalibrationContainer.h>
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
        // consts for checkBallRotation()
        static const int ROTATE_CORRECT = 0;
        static const int ROTATE_LEFT = 10;
        static const int ROTATE_RIGHT = 20;
        //static const int ROTATE_BACKWARDS = 30;
        static const int ROTATE_TOO_SLOW = 40;
        static const int ROTATE_ERR = -10;
        DribbleCalibrationContainer dcc;

        bool ballRotateCorrect;
        bool ballHoldCorrect;

        // config parameters
        double speedNoBall;
        double slowTranslationWheelSpeed;
        double minRotation;
        // left and right are swapped
        double dribbleFactorLeft;
        double dribbleFactorRight;

        // for correctRightWheelSpeed function
        double dribbleFactorRightOld;
        double changingFactor;
        int defectWheel;

        int checkBallRotation();
        void correctWheelSpeed(int wheel);
        void readConfigParameters();
        void writeConfigParameters();

        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1469109429392) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CalibrationTakeBall_H_ */
