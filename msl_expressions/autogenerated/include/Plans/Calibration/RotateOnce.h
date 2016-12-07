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
        double const CALIB_ERROR_THRESHOLD = 0.005;
        // this data comes from the IMU
        double initialBearing;
        // this data comes from the motion
        double initialAngle;
        static geometry::CNPoint2D* measurements[2];

        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1467397900274) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1467397900274) ENABLED START*/ //Add additional private methods here
        static const int PRECISION_BUFFER_SIZE = 10;
        static constexpr double MAX_ROTATION_SPEED = 2.0;
        static constexpr double STEP_SIZE = 3.0;
        //Buffer holding precision values of current rotation
        msl::RingBuffer<double>* precisionBuffer;
        double segments[3];
        bool visitedSegments[3];
        double initialRadius;
        double lastRotationCalibError;
        static constexpr double ACCELERATION = 0.05;

        // rotation speed varies
        double rotationSpeed;

        int getCurrentRotationSegment();
        double circularDiff(double a, double b);
        double getLimitedRotationSpeed(double desiredSpeed);
        void logCalibrationResult(double currentRadius, double calibError);

        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* RotateOnce_H_ */
