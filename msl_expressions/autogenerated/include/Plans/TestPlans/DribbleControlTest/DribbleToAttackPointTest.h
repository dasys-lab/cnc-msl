#ifndef DribbleToAttackPointTest_H_
#define DribbleToAttackPointTest_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1498664309837) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
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
        nonstd::optional<geometry::CNPointAllo> getClosestOpp();
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1498664309837) ENABLED START*/ //Add additional private methods here
        geometry::CNPointAllo alloTargetPoint;
        geometry::CNPointEgo egoTargetPoint;
        supplementary::SystemConfig* sc;

        // old variables -> some of them need to be removed when finished
        int wheelSpeed;
        geometry::CNPointAllo lastClosesOpp;
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

        msl::MovementQuery query;
        geometry::CNPointAllo destinationPoint;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleToAttackPointTest_H_ */
