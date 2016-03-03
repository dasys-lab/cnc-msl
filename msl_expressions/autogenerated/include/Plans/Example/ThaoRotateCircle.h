#ifndef ThaoRotateCircle_H_
#define ThaoRotateCircle_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1450104610893) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/MotionControl.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include "robotmovement/RobotMovement.h"

using namespace msl;
using namespace msl_actuator_msgs;
/*PROTECTED REGION END*/
namespace alica
{
    class ThaoRotateCircle : public DomainBehaviour
    {
    public:
        ThaoRotateCircle();
        virtual ~ThaoRotateCircle();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1450104610893) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1450104610893) ENABLED START*/ //Add additional protected methods here
        shared_ptr<geometry::CNPosition> ownPos;
        int count;
        bool haveBeenFarAway;
        double oldDistance;
        double kP;
        double kD;
        double rotate_P;
        int isMovingCloserIter;
        int isMovingAwayIter;
        int maxIter;
        double timeForPass;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1450104610893) ENABLED START*/ //Add additional private methods here
        msl_actuator_msgs::MotionControl driveToApproachingBall(shared_ptr<geometry::CNVelocity2D> ballVelocity,
                                                                shared_ptr<geometry::CNPoint2D> egoBallPos);
        msl_actuator_msgs::MotionControl driveToMovingBall(shared_ptr<geometry::CNPoint2D> egoBallPos,
                                                           shared_ptr<geometry::CNVelocity2D> egoBallVel);

        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ThaoRotateCircle_H_ */
