#ifndef DriveToGoal_H_
#define DriveToGoal_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1447863424939) ENABLED START*/ //Add additional includes here
#include "cnc_geometry/CNPointAllo.h"
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_robot/robotmovement/MovementQuery.h>
/*PROTECTED REGION END*/
namespace alica
{
    class DriveToGoal : public DomainBehaviour
    {
    public:
        DriveToGoal();
        virtual ~DriveToGoal();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1447863424939) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1447863424939) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1447863424939) ENABLED START*/ //Add additional private methods here
        msl_actuator_msgs::MotionControl mc;
        geometry::CNPointAllo alloFieldCenterAlignPoint, alloGoalMid, alloGoalRight, alloGoalLeft;
        int goalieSize;
        string goalInitPos;
        msl::MovementQuery query;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveToGoal_H_ */
