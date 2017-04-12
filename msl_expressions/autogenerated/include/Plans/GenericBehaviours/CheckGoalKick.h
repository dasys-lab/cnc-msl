#ifndef CheckGoalKick_H_
#define CheckGoalKick_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1449076008755) ENABLED START*/ //Add additional includes here
#include <nonstd/optional.hpp>
#include <cnc_geometry/CNPointEgo.h>
#include <cnc_geometry/CNPositionAllo.h>
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
        nonstd::optional<geometry::CNPositionAllo> ownPos;
        nonstd::optional<geometry::CNPointEgo> egoBallPos;
        double minObsDistGoal;
        double minOwnDistGoal;
        double minOppYDist;
        double closeGoalDist;
        double farGoalDist;
        double minOwnDistObs;
        double keeperDistGoal;
        double minKeeperDistBallTrajectory;
        double minKickPower;

        //WM16 experiments---
        bool checkGoalie;
        //---

        bool usePrediction;
        int predictionTime;

        // testing variables for console output
        double cout_distBall2HitPoint;

        void readConfigParameters();
        double getKickPower(const geometry::CNPointAllo& hitPoint);
        void kick(double kickpower);
        bool checkGoalKeeper(const geometry::CNPointAllo& hitPoint);
        bool checkShootPossibility(const geometry::CNPointAllo& hitPoint, double& kickPower);
        nonstd::optional<geometry::CNPointAllo> computeHitPoint(double posX, double posY, double alloAngle);
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CheckGoalKick_H_ */
