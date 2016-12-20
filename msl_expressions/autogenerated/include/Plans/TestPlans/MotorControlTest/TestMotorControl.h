#ifndef TestMotorControl_H_
#define TestMotorControl_H_

#include "DomainBehaviour.h"


#include <SystemConfig.h>
#include <Configuration.h>
#include <limits>


/*PROTECTED REGION ID(inc1482163964536) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class TestMotorControl : public DomainBehaviour
    {
    public:
        TestMotorControl();
        virtual ~TestMotorControl();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1482163964536) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1482163964536) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1482163964536) ENABLED START*/ //Add additional private methods here
        shared_ptr<geometry::CNPosition> start;
        shared_ptr<geometry::CNPosition> goal;
        double relGoalX;
        double relGoalY;
        double relGoalRot;
        bool straight;
        int count;
        int testSpeed;

        double abortTime;

        shared_ptr<geometry::CNPosition> goalPointer;
        double goalDistance = 0;
        double oldGoalDistance = std::numeric_limits<double>::max();

        bool terminated;

        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* TestMotorControl_H_ */
