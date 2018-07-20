#ifndef SearchForPassPoint_H_
#define SearchForPassPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1436269017402) ENABLED START*/ // Add additional includes here
#define BEH_DEBUG
#include "GameState.h"
#include "engine/model/EntryPoint.h"
#define DBM_DEBUG 1

namespace msl
{
    class VoronoiNet;
}
namespace geometry
{
    class CNPoint2D;
    class CNPosition;
}
/*PROTECTED REGION END*/
namespace alica
{
    class SearchForPassPoint : public DomainBehaviour
    {
    public:
        SearchForPassPoint();
        virtual ~SearchForPassPoint();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1436269017402) ENABLED START*/ // Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1436269017402) ENABLED START*/ // Add additional protected methods here
        vector<string> teamMateTaskName;
        vector<string> teamMatePlanName;
        vector<EntryPoint *> eps;
        vector<int> teamMateIds;
        double freeOppAngle;
        double ratio;
        double ballRadius;
        double passCorridorWidth;
        double maxTurnAngle;
        double minOppDist;
        double minPassDist;
        double maxPassDist;
        double distToFieldBorder;
        double minCloserOffset;
        double closerFactor;
        double closerFactor2;
        shared_ptr<geometry::CNPoint2D> alloBall;
        shared_ptr<geometry::CNPosition> alloPos;
        shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> matePoses;
#ifdef DBM_DEBUG
    shared_ptr<msl_helper_msgs::DebugMsg> dbm;
#endif
    bool passPossible(double cf, shared_ptr<geometry::CNPoint2D> passPoint, shared_ptr<geometry::CNPoint2D> receiver, shared_ptr<msl::VoronoiNet> vNet);

    /*PROTECTED REGION END*/private:
    /*PROTECTED REGION ID(prv1436269017402) ENABLED START*/ // Add additional private methods here
    /*PROTECTED REGION END*/};
}
/* namespace alica */

#endif /* SearchForPassPoint_H_ */
