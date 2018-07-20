#ifndef AttackOpp_H_
#define AttackOpp_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1430324527403) ENABLED START*/ //Add additional includes here
#include <container/CNPoint2D.h>
#include <container/CNPosition.h>
#include <container/CNVelocity2D.h>
#include <msl_robot/robotmovement/MovementQuery.h>
using namespace msl;
/*PROTECTED REGION END*/
namespace alica
{
    class AttackOpp : public DomainBehaviour
    {
    public:
        AttackOpp();
        virtual ~AttackOpp();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1430324527403) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1430324527403) ENABLED START*/ //Add additional protected methods here
        geometry::CNPoint2D alloTarget;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1430324527403) ENABLED START*/ //Add additional private methods here
        double oldDistance;
        double kP;
        double kI;
        double kD;
        double rotate_P;
        int isMovingCloserIter;
        int isMovingAwayIter;
        int maxIter;
        shared_ptr<msl::MovementQuery> query;
        msl_actuator_msgs::MotionControl ballGetsCloser(shared_ptr<geometry::CNPosition> robotPosition,
                                                        shared_ptr<geometry::CNVelocity2D> ballVelocity,
                                                        shared_ptr<geometry::CNPoint2D> egoBallPos);
        msl_actuator_msgs::MotionControl driveToMovingBall(shared_ptr<geometry::CNPoint2D> egoBallPos,
                                                           shared_ptr<geometry::CNVelocity2D> egoBallVel);
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AttackOpp_H_ */
