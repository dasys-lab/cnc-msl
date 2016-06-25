#ifndef DriveInSquare_H_
#define DriveInSquare_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1433939613017) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/MotionControl.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include "msl_robot/robotmovement/RobotMovement.h"

using namespace msl;
using namespace msl_actuator_msgs;

/*PROTECTED REGION END*/
namespace alica
{
    class DriveInSquare : public DomainBehaviour
    {
    public:
        DriveInSquare();
        virtual ~DriveInSquare();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1433939613017) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1433939613017) ENABLED START*/ //Add additional protected methods here
        int count;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1433939613017) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveInSquare_H_ */
