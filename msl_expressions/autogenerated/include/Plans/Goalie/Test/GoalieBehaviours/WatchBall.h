#ifndef WatchBall_H_
#define WatchBall_H_

#include <InfoBuffer.h>
#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1447863466691) ENABLED START*/ //Add additional includes here
#include "cnc_geometry/CNPositionAllo.h"
#include "cnc_geometry/CNPointAllo.h"
#include "cnc_geometry/CNVecAllo.h"
#include "cnc_geometry/CNPointEgo.h"
#include <string>
#include <msl_actuator_msgs/MotionControl.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <nonstd/optional.hpp>

using namespace msl;
/*PROTECTED REGION END*/
namespace alica
{
    class WatchBall : public DomainBehaviour
    {
    public:
        WatchBall();
        virtual ~WatchBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1447863466691) ENABLED START*/ //Add additional public methods here
        double fitTargetY(double targetY);
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1447863466691) ENABLED START*/ //Add additional protected methods here
        msl_actuator_msgs::MotionControl mc;
        nonstd::optional<geometry::CNPositionAllo> ownPos;
        geometry::CNPointAllo alloGoalLeft;
        geometry::CNPointAllo alloGoalRight;
        geometry::CNPointAllo alloGoalMid;
        geometry::CNPointAllo prevTarget;
        geometry::CNPointAllo alloFieldCntr = MSLWorldModel::get()->field->posCenterMarker();
        geometry::CNPointAllo alloAlignPt = alloFieldCntr;
        bool alignTowardsBall;
        int maxVariance;
        int goalieSize;
        int nrOfPositions;
        int snapDistance;
        double alignMaxVel;
        double pTrans, dTrans, pRot, dRot;
        double prevTargetDist, lastRotErr;
        double rotationLimit;
        msl::InfoBuffer<geometry::CNPointAllo>* ballPositions;
        double calcGoalImpactY();
        void rotate(geometry::CNPointAllo alloTarget);
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1447863466691) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* WatchBall_H_ */
