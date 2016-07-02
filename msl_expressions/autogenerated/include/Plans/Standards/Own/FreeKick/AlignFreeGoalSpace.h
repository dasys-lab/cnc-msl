#ifndef AlignFreeGoalSpace_H_
#define AlignFreeGoalSpace_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1467039782450) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPoint3D.h"
#include "MSLFootballField.h"
#include <sstream>
#include <container/CNVelocity2D.h>
#include <GeometryCalculator.h>
#include "msl_actuator_msgs/MotionControl.h"
#include "msl_actuator_msgs/KickControl.h"
#include "msl_robot/robotmovement/RobotMovement.h"
#include "SystemConfig.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <obstaclehandler/Obstacles.h>

using namespace msl;
using namespace msl_actuator_msgs;
/*PROTECTED REGION END*/
namespace alica
{
    class AlignFreeGoalSpace : public DomainBehaviour
    {
    public:
        AlignFreeGoalSpace();
        virtual ~AlignFreeGoalSpace();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1467039782450) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1467039782450) ENABLED START*/ //Add additional protected methods here
        double maxVel;
        double angleTolerance;
        double ballDiameter;
        double goalLineLength;
        double robotRadius;
//                double wheelSpeed;
        double kickPower;
        //0 = not alignt, 1 = left, 2 = right
        int lastAlignment;

        // PID variables for alignToPointWithBall
        double defaultRotateP;
        double alignToPointpRot;
        double lastRotError;
        double alignToPointMaxRotation;
        double alignToPointMinRotation;
        //        double lastRotErrorWithBall;
        double alignMaxVel;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1467039782450) ENABLED START*/ //Add additional private methods here
        msl_actuator_msgs::MotionControl alignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
                                                              shared_ptr<geometry::CNPoint2D> egoBallPos,
                                                              double angleTolerance, double ballAngleTolerance);
        void readConfigParameters();
        shared_ptr<geometry::CNPoint2D> alloTarget;
//        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlignFreeGoalSpace_H_ */
