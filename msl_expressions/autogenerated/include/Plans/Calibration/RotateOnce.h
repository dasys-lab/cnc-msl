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

        static int const MIN_ROTATIONS = 3; // don't calculate a regression unless we rotated for MIN_ROTATIONS rotations
        static int const MAX_ROTATIONS = 40; // finish calibration after reaching MAX_ROTATIONS without hitting MIN_BEARING_DIFF_FOR_REGRESSION
        static double constexpr MIN_BEARING_DIFF_FOR_REGRESSION = 0.03; // start regression calculation as soon as motion/imu rotation counter difference reaches MIN_BEARING_DIFF_FOR_REGRESSION
        static int logCounter;

        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1467397900274) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1467397900274) ENABLED START*/ //Add additional private methods here
        static constexpr double MAX_ROTATION_SPEED = 1.5;
        static constexpr double ACCELERATION = 0.1;
        double robotRadius;

        int motionRotations;
        int imuRotations;
        double diffOffset;
        bool diffOffsetInitialized;
        double lastIMUBearing;
        double lastMotionBearing;

        // rotation speed varies
        double rotationSpeed;

        /**
         * Calculates the shortest angle distance between two angles.
         * @param a
         * @param b
         * @return Angle distance in [0;pi]. Additionally, the sign shows if the shortest distance is reached clockwise or counter-clockwise.
         */
        double circularDiff(double a, double b);

        /**
         * Gets a speed that is limited by constant MAX_ROTATION_SPEED. Also works for negative
         * @param desiredSpeed
         * @return
         */
        double getLimitedRotationSpeed(double desiredSpeed);

        /**
         * write the given parameter into sc->getLogPath()+"RotationCalibration.log" to be used by gnuplot later
         * @param imuMotionDifference
         */
        void logIMUMotionDifference(double imuMotionDifference);

        double getMotionBearing();
        double getIMUBearing();

        /**
         * Updates rotation count based on the given bearing.
         * @param currentBearing
         * @param lastBearing Used for reference. Will be overwritten with lastBearing.
         * @param rotations Will be overwritten with the number of whole rotations.
         * @param isfullRotation Will be overwritten with true if a whole rotation just completed, false otherwise.
         * @return Exact number of rotations including decimal digits for a partial rotation.
         */
        double updateRotationCount(double currentBearing, double &lastBearing, int &rotations, bool &isfullRotation);

        /**
         * calls gnuplot and asks it for a linear regression of previously logged values
         */
        void calculateRadius();
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* RotateOnce_H_ */
