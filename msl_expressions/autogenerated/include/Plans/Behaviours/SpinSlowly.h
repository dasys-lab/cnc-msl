#ifndef SpinSlowly_H_
#define SpinSlowly_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1435159253296) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/MotionControl.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include "msl_robot/robotmovement/RobotMovement.h"
#include "math.h"
/*PROTECTED REGION END*/
namespace alica
{
    class SpinSlowly : public DomainBehaviour
    {
    public:
        SpinSlowly();
        virtual ~SpinSlowly();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1435159253296) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1435159253296) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1435159253296) ENABLED START*/ //Add additional private methods here
        double alpha, startAngle;
        double epsilon = 0.03;
        int counter;
        shared_ptr<geometry::CNPoint2D> center;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* SpinSlowly_H_ */
