#ifndef RotateOnce_H_
#define RotateOnce_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1467397900274) ENABLED START*/ //Add additional includes here
#include "RingBuffer.h"
#include <container/CNPoint2D.h>
/*PROTECTED REGION END*/
namespace alica
{
    class RotateOnce : public DomainBehaviour
    {
    public:
        RotateOnce();
        virtual ~RotateOnce();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1467397900274) ENABLED START*/ //Add additional public methods here
        double const CIRCDIFF_THRESHOLD = 0.5; //TODO tweak
        static int logCounter;

        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1467397900274) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1467397900274) ENABLED START*/ //Add additional private methods here
        static constexpr double MAX_ROTATION_SPEED = 1.0;
        static constexpr double ACCELERATION = 0.05;
        double robotRadius;

        int motionRotations = 0;
        int imuRotations = 0;
        double lastIMUBearing;
        double lastMotionBearing;

        // rotation speed varies
        double rotationSpeed;

        double circularDiff(double a, double b);
        double getLimitedRotationSpeed(double desiredSpeed);
        void logCalibrationResult(double currentRadius, double calibError);

		double getMotionBearing();
		double getIMUBearing();

		double updateRotationCount(double currentBearing, double &lastBearing, int &rotations);
		/*PROTECTED REGION END*/
	};
} /* namespace alica */

#endif /* RotateOnce_H_ */
