#ifndef Wander_H_
#define Wander_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1434716215423) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
#include <MSLEnums.h>
/*PROTECTED REGION END*/
namespace alica
{
    class Wander : public DomainBehaviour
    {
    public:
        Wander();
        virtual ~Wander();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1434716215423) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1434716215423) ENABLED START*/ //Add additional protected methods here
        double EPSILON_RADIUS;
        double SLOW_DOWN_DISTANCE;
        double maxTranslation;
        double translation;
        vector<geometry::CNPointAllo> targetPoints;
        vector<geometry::CNPointAllo> targetPointsOwnCorner;
        vector<geometry::CNPointAllo> targetPointsOppCorner;
        vector<geometry::CNPointAllo> targetPointsOwnGoalKick;
        vector<geometry::CNPointAllo> targetPointsOppGoalKick;
        vector<geometry::CNPointAllo> targetPointsThrowIn;
        vector<geometry::CNPointAllo> targetPointsDropBall;
        geometry::CNPointAllo currentTargetPoint;
        double fieldLength;
        double fieldWidth;
        int distToCorner;
        int distToOutLine;
        bool firstTargetSet;
        msl::MovementQuery query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1434716215423) ENABLED START*/ //Add additional private methods here
        void setFirstTargetPoint(msl::Situation situation);
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Wander_H_ */
