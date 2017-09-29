#ifndef AttackOpp_H_
#define AttackOpp_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1430324527403) ENABLED START*/ //Add additional includes here
#include <cnc_geometry/CNVecEgo.h>
#include <cnc_geometry/CNPositionAllo.h>
#include <cnc_geometry/CNPositionEgo.h>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
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
        geometry::CNPointAllo alloTarget;
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
        msl::MovementQuery query;
        msl_actuator_msgs::MotionControl ballGetsCloser(geometry::CNPositionAllo robotPosition,
                                                        geometry::CNVecEgo ballVelocity,
                                                        geometry::CNPointEgo egoBallPos);
        msl_actuator_msgs::MotionControl driveToMovingBall(nonstd::optional<geometry::CNPointEgo> egoBallPos,
                                                           nonstd::optional<geometry::CNVecEgo> egoBallVel);
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AttackOpp_H_ */
