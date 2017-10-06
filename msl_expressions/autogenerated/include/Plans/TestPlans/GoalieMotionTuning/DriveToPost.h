#ifndef DriveToPost_H_
#define DriveToPost_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1464189819779) ENABLED START*/ //Add additional includes here
#include <msl_actuator_msgs/MotionControl.h>
#include <nonstd/optional.hpp>
/*PROTECTED REGION END*/
namespace alica
{
    class DriveToPost : public DomainBehaviour
    {
    public:
        DriveToPost();
        virtual ~DriveToPost();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1464189819779) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1464189819779) ENABLED START*/ //Add additional protected methods here
        msl_actuator_msgs::MotionControl mc;
        geometry::CNPointAllo alloGoalLeft;
        geometry::CNPointAllo alloGoalRight;
        geometry::CNPointAllo alloGoalMid;
        nonstd::optional<geometry::CNPositionAllo> ownPos;
        double pTrans, dTrans, alignMaxVel, prevTargetDist, avgTime;
        int snapDistance, goalieSize, driveToPost;
        long int startTime;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1464189819779) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveToPost_H_ */
