#ifndef DriveToGoal_H_
#define DriveToGoal_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1447863424939) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"

using namespace msl;
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
        static const int SIMULATING = 1; // simulating 1, real life -1
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1447863424939) ENABLED START*/ //Add additional private methods here
        msl_actuator_msgs::MotionControl mc;
        shared_ptr<geometry::CNPoint2D> alloFieldCenterAlignPoint;
        shared_ptr<geometry::CNPoint2D> alloTarget;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveToGoal_H_ */
