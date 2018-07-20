#ifndef StandardAlignAndGrab2Receivers_H_
#define StandardAlignAndGrab2Receivers_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462368682104) ENABLED START*/ // Add additional includes here
namespace geometry
{
    class CNPoint2D;
}
namespace msl
{
    class MovementQuery;
}
#include <InformationElement.h>
/*PROTECTED REGION END*/
namespace alica
{
    class StandardAlignAndGrab2Receivers : public DomainBehaviour
    {
    public:
        StandardAlignAndGrab2Receivers();
        virtual ~StandardAlignAndGrab2Receivers();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1462368682104) ENABLED START*/ // Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1462368682104) ENABLED START*/ // Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1462368682104) ENABLED START*/ // Add additional private methods here
        shared_ptr<msl::MovementQuery> query;
        string teamMateTaskName1;
        string teamMateTaskName2;
        shared_ptr<geometry::CNPoint2D> recPos;
        shared_ptr<geometry::CNPoint2D> aRecPos;
        double ratio;
        double ballRadius;
        double passCorridorWidth;
        double maxTurnAngle;
        double minOppDist;
        double minCloserOffset;
        bool longPassPossible;
        int haveBallCounter;
        double trans;
        double angleIntErr;
        double oldAngleErr;
        double minTol;
        double tol;
        msl::InfoTime startTime;
        int longPassCounter;
        int longPassThreshold;
        int adaptiveThreshold;
        int longThresholdMin;
        int longThresholdMax;

        shared_ptr<geometry::CNPoint2D> getTeammatePosFromTaskName(string teamMateTaskName);

        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardAlignAndGrab2Receivers_H_ */
