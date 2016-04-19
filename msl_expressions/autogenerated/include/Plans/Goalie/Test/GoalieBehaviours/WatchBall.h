#ifndef WatchBall_H_
#define WatchBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1447863466691) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include <string>
#include <RingBuffer.h>

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
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1447863466691) ENABLED START*/ //Add additional protected methods here
        msl_actuator_msgs::MotionControl mc;
        shared_ptr<geometry::CNPosition> ownPos;
        shared_ptr<geometry::CNPoint2D> alloGoalLeft;
        shared_ptr<geometry::CNPoint2D> alloGoalRight;
        shared_ptr<geometry::CNPoint2D> alloGoalMid;
        shared_ptr<geometry::CNPoint2D> prevTarget;
        shared_ptr<geometry::CNPoint2D> alloFieldCntr = MSLWorldModel::get()->field.posCenterMarker();
        shared_ptr<geometry::CNPoint2D> alloAlignPt = alloFieldCntr;
        int maxVariance;
        int goalieSize;
        int nrOfPositions;
        int snapDistance;
        double fastRotation;
        double alignMaxVel;
        double pFactor;
        double dFactor;
        double prevTargetDist;
        msl::RingBuffer<geometry::CNPoint2D>* ballPositions;
        double fitTargetY(double targetY);
        double calcGoalImpactY();
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1447863466691) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* WatchBall_H_ */
