#ifndef RobotMovementTest_H_
#define RobotMovementTest_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1473862842303) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/MovementQuery.h"
/*PROTECTED REGION END*/
namespace alica
{
    class RobotMovementTest : public DomainBehaviour
    {
    public:
        RobotMovementTest();
        virtual ~RobotMovementTest();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1473862842303) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1473862842303) ENABLED START*/ //Add additional protected methods here
        double toX;
        double toY;
        msl::MovementQuery query;

        bool hadBall;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1473862842303) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* RobotMovementTest_H_ */
