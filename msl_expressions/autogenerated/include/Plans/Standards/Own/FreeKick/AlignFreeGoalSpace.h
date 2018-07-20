#ifndef AlignFreeGoalSpace_H_
#define AlignFreeGoalSpace_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1467039782450) ENABLED START*/ // Add additional includes here
#include "msl_actuator_msgs/MotionControl.h"
namespace geometry
{
    class CNPoint2D;
}
/*PROTECTED REGION END*/
namespace alica
{
    class AlignFreeGoalSpace : public DomainBehaviour
    {
    public:
        AlignFreeGoalSpace();
        virtual ~AlignFreeGoalSpace();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1467039782450) ENABLED START*/ // Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1467039782450) ENABLED START*/ // Add additional protected methods here
        shared_ptr<geometry::CNPoint2D> alloTarget;
        shared_ptr<geometry::CNPoint2D> alloLeftAimPoint;
        shared_ptr<geometry::CNPoint2D> alloRightAimPoint;
        shared_ptr<geometry::CNPoint2D> frontLeft;
        shared_ptr<geometry::CNPoint2D> frontRight;

        // PID parameters for alignToPointWithBall
        double defaultRotateP;
        double alignToPointpRot;
        double lastRotError;
        double alignToPointMaxRotation;
        double alignToPointMinRotation;

        double maxVel;
        double angleTolerance;
        double ballDiameter;
        double goalLineLength;
        double robotRadius;

        // 0 = not aligned, 1 = left, 2 = right
        int lastAlignment;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1467039782450) ENABLED START*/ // Add additional private methods here
        msl_actuator_msgs::MotionControl alignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
                                                              shared_ptr<geometry::CNPoint2D> egoBallPos,
                                                              double angleTolerance, double ballAngleTolerance);
        void readConfigParameters();
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlignFreeGoalSpace_H_ */
