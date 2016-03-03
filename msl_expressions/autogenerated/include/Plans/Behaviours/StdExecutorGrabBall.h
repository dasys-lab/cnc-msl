#ifndef StdExecutorGrabBall_H_
#define StdExecutorGrabBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1441209011595) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/MotionControl.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include "robotmovement/RobotMovement.h"

using namespace msl;
using namespace msl_actuator_msgs;
/*PROTECTED REGION END*/
namespace alica
{
    class StdExecutorGrabBall : public DomainBehaviour
    {
    public:
        StdExecutorGrabBall();
        virtual ~StdExecutorGrabBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1441209011595) ENABLED START*/ //Add additional public methods here
        shared_ptr<geometry::CNPosition> ownPos;

        double oldDistance;
        double kP;
        double kD;
        double rotate_P;
        int isMovingCloserIter;
        int isMovingAwayIter;
        int maxIter;
        double timeForPass;

        void readConfigParameters();
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1441209011595) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1441209011595) ENABLED START*/ //Add additional private methods here
        double catchRadius;
        int count;
        int iteration;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StdExecutorGrabBall_H_ */
