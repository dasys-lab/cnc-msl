#ifndef AlignFreeGoalSpace_H_
#define AlignFreeGoalSpace_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1467039782450) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "SystemConfig.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <obstaclehandler/Obstacles.h>
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
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1467039782450) ENABLED START*/ //Add additional private methods here
        shared_ptr<geometry::CNPoint2D> alloTarget;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlignFreeGoalSpace_H_ */
