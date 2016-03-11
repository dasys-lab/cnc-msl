#ifndef CheckGoalKick_H_
#define CheckGoalKick_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1449076008755) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class CheckGoalKick : public DomainBehaviour
    {
    public:
        CheckGoalKick();
        virtual ~CheckGoalKick();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1449076008755) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1449076008755) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1449076008755) ENABLED START*/ //Add additional private methods here
        shared_ptr<geometry::CNPosition> ownPos;
        shared_ptr<geometry::CNPoint2D> egoBallPos;
        msl::MSLFootballField* field;
        double minObsDistGoal;
        double minOwnDistGoal;
        double minOppYDist;
        double closeGoalDist;
        double farGoalDist;
        double minOwnDistObs;
        double keeperDistGoal;
        double minKeeperDistBallTrajectory;
        double minKickPower;

        // testing variables for console output
        double cout_kickpower;
        bool cout_kicking;

        bool checkShootPossibility(shared_ptr<geometry::CNPoint2D> hitPoint);
        void readConfigParameters();
        void kicking(shared_ptr<geometry::CNPoint2D> hitPoint);
        bool checkGoalKeeper(shared_ptr<geometry::CNPoint2D> hitPoint);
        shared_ptr<geometry::CNPoint2D> computeHitPoint(double posX, double posY, double alloAngle);
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CheckGoalKick_H_ */
