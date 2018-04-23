#ifndef ThrowInPass_H_
#define ThrowInPass_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462363192018) ENABLED START*/ // Add additional includes here
namespace geometry
{
    class CNPoint2D;
}
/*PROTECTED REGION END*/
namespace alica
{
    class ThrowInPass : public DomainBehaviour
    {
    public:
        ThrowInPass();
        virtual ~ThrowInPass();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1462363192018) ENABLED START*/ // Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1462363192018) ENABLED START*/ // Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1462363192018) ENABLED START*/ // Add additional private methods here
        bool longPassPossible;
        string teamMateTaskName1;
        string teamMateTaskName2;
        shared_ptr<geometry::CNPoint2D> recPos;
        shared_ptr<geometry::CNPoint2D> aRecPos;
        double ratio;
        double ballRadius;
        double passCorridorWidth;
        double maxTurnAngle;
        double minOppDist;
        double closerFactor;
        double maxVel;
        double pRot;
        double dRot;
        double lastRotError;
        double minRot;
        double maxRot;
        double accel;
        double arrivalTimeOffset;
        int recId;
        int aRecId;
        int longPassCounter;
        int longPassThreshold;
        bool sentPm;
        pair<int, shared_ptr<geometry::CNPoint2D>> getTeammateIdAndPosFromTaskName(string teamMateTaskName);
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ThrowInPass_H_ */
