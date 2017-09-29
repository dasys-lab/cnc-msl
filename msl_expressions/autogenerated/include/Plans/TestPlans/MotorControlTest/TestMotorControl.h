#ifndef TestMotorControl_H_
#define TestMotorControl_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1482163964536) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Configuration.h>
#include <limits>
#include <nonstd/optional.hpp>
#include "cnc_geometry/CNPositionAllo.h"
#include "cnc_geometry/CNVecAllo.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "RawSensorData.h"
#include "MSLWorldModel.h"
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
        nonstd::optional<geometry::CNPositionAllo> start;
        geometry::CNPositionAllo goal;
        nonstd::optional<geometry::CNPositionAllo> currentPos;
        double relGoalX;
        double relGoalY;
        double relGoalRot;
        bool straight;
        int count;
        int testSpeed;

        double abortTime;

        geometry::CNVecAllo goalVec;
        double goalDistance = 0;
        double oldGoalDistance = std::numeric_limits<double>::max();
        double angleDistance;

        bool terminated;

        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* TestMotorControl_H_ */
