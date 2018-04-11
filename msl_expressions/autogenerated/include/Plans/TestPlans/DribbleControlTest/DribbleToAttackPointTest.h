#ifndef DribbleToAttackPointTest_H_
#define DribbleToAttackPointTest_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1498664309837) ENABLED START*/ //Add additional includes here
namespace geometry {
	class CNPoint2D;
}
namespace msl {
	class MovementQuery;
}
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleToAttackPointTest : public DomainBehaviour
    {
    public:
        DribbleToAttackPointTest();
        virtual ~DribbleToAttackPointTest();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1498664309837) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1498664309837) ENABLED START*/ //Add additional protected methods here
        void readConfigParameters();
        shared_ptr<geometry::CNPoint2D> getClosestOpp();
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1498664309837) ENABLED START*/ //Add additional private methods here
        std::shared_ptr<geometry::CNPoint2D> alloTargetPoint;
        std::shared_ptr<geometry::CNPoint2D> egoTargetPoint;

        // old variables -> some of them need to be removed when finished
        int wheelSpeed;
        std::shared_ptr<geometry::CNPoint2D> lastClosesOpp;
        double lastRotError;
        bool ownPenalty;
        std::vector<double> pastRotation;
        long counter;
        double maxVel;
        double maxOppDist;
        double maxDribbleSpeed;
        double oppVectorWeight;
        int clausenDepth;
        int clausenPow;
        int pastRotationSize;
        double orthoDriveWeight;
        double targetDriveWeight;

        std::shared_ptr<msl::MovementQuery> query;
        std::shared_ptr<geometry::CNPoint2D> destinationPoint;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleToAttackPointTest_H_ */
