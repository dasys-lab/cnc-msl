#ifndef PositionReceiver_H_
#define PositionReceiver_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1439379316897) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include "MSLWorldModel.h"
#include <pathplanner/PathProxy.h>
#include <pathplanner/evaluator/PathEvaluator.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <Game.h>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
#include <cnc_geometry/CNPositionAllo.h>
/*PROTECTED REGION END*/
namespace alica
{
    class PositionReceiver : public DomainBehaviour
    {
    public:
        PositionReceiver();
        virtual ~PositionReceiver();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1439379316897) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1439379316897) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1439379316897) ENABLED START*/ //Add additional private methods here
        static msl_actuator_msgs::MotionControl moveToPointFast(geometry::CNPointEgo egoTarget,
                                                                geometry::CNPointEgo egoAlignPoint,
                                                                double snapDistance,
                                                                nonstd::optional<std::vector<geometry::CNPointAllo>> additionalPoints);

                                                            void readConfigParameters();
                                                            double fastCatchRadius;
                                                            double slowCatchRadius;
                                                            double alignTolerance;
                                                            double ballDistanceRec;
                                                            msl::MovementQuery query;
                                                            /*PROTECTED REGION END*/};
                                                    }
                                                    /* namespace alica */

#endif /* PositionReceiver_H_ */
