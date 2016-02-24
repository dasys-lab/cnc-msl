#ifndef MoveThaoCircleBack_H_
#define MoveThaoCircleBack_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1450092434819) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/MotionControl.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    class MoveThaoCircleBack : public DomainBehaviour
    {
    public:
        MoveThaoCircleBack();
        virtual ~MoveThaoCircleBack();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1450092434819) ENABLED START*/ //Add additional public methods here
        bool haveBeenFarAway;
        shared_ptr<geometry::CNPosition> ownPos;
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1450092434819) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1450092434819) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* MoveThaoCircleBack_H_ */
