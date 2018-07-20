#ifndef RobotMovementDribbleTest_H_
#define RobotMovementDribbleTest_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462969724089) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/MovementQuery.h"
/*PROTECTED REGION END*/
namespace alica
{
    class RobotMovementDribbleTest : public DomainBehaviour
    {
    public:
        RobotMovementDribbleTest();
        virtual ~RobotMovementDribbleTest();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1462969724089) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1462969724089) ENABLED START*/ //Add additional protected methods here
        void trueInitialize();
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1462969724089) ENABLED START*/ //Add additional private methods here
        shared_ptr<geometry::CNPoint2D> currentTarget;
        vector<double> attackPosY;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* RobotMovementDribbleTest_H_ */
