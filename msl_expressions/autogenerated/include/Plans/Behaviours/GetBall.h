#ifndef GetBall_H_
#define GetBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1414828300860) ENABLED START*/ //Add additional includes here
#include <container/CNPoint2D.h>
#include <container/CNPosition.h>
#include <container/CNVelocity2D.h>
#include <msl_robot/robotmovement/MovementQuery.h>
using namespace msl;
/*PROTECTED REGION END*/
namespace alica
{
    class GetBall : public DomainBehaviour
    {
    public:
        GetBall();
        virtual ~GetBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1414828300860) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1414828300860) ENABLED START*/ //Add additional protected methods here
        double oldDistance;
        double kP;
        double kD;
        double rotate_P;
        int isMovingCloserIter;
        int isMovingAwayIter;
        int maxIter;
        double timeForPass;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1414828300860) ENABLED START*/ //Add additional private methods here
        msl_actuator_msgs::MotionControl driveToApproachingBall(shared_ptr<geometry::CNVelocity2D> ballVelocity,
                                                                shared_ptr<geometry::CNPoint2D> egoBallPos);
        msl_actuator_msgs::MotionControl driveToMovingBall(shared_ptr<geometry::CNPoint2D> egoBallPos,
                                                           shared_ptr<geometry::CNVelocity2D> egoBallVel);
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* GetBall_H_ */
