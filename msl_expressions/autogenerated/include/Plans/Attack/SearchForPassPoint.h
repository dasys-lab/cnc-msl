#ifndef SearchForPassPoint_H_
#define SearchForPassPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1436269017402) ENABLED START*/ //Add additional includes here
#define BEH_DEBUG
#include <vector>
#include <SystemConfig.h>
#include <engine/model/EntryPoint.h>
#include <pathplanner/PathProxy.h>
#include <cnc_geometry/CNVecAllo.h>
#include <cnc_geometry/CNPointAllo.h>
#include "GameState.h"
#include "MSLFootballField.h"
#define DBM_DEBUG 1
/*PROTECTED REGION END*/
namespace alica
{
    class SearchForPassPoint : public DomainBehaviour
    {
    public:
        SearchForPassPoint();
        virtual ~SearchForPassPoint();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1436269017402) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1436269017402) ENABLED START*/ //Add additional protected methods here
        supplementary::SystemConfig* sc;
        vector<string> teamMateTaskName;
        vector<string> teamMatePlanName;
        vector<EntryPoint*> eps;
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
        msl::PathProxy* pathProxy;
        nonstd::optional<geometry::CNPointAllo> alloBall;
        nonstd::optional<geometry::CNPositionAllo> alloPos;
        nonstd::optional<std::vector<geometry::CNPointAllo>> matePoses;
#ifdef DBM_DEBUG
        shared_ptr<msl_helper_msgs::DebugMsg> dbm;
#endif
        static bool outsideTriangle(geometry::CNPointAllo a, geometry::CNPointAllo b, geometry::CNPointAllo c,
                                    double tolerance, std::shared_ptr<std::vector<geometry::CNPointAllo>> points);
        static bool outsideCorridoreTeammates(geometry::CNPointAllo ball, geometry::CNPointAllo passPoint,
                                              double passCorridorWidth, vector<geometry::CNPointAllo>& points);
        static bool outsideCorridore(geometry::CNPointAllo ball, geometry::CNPointAllo passPoint,
                                     double passCorridorWidth, std::shared_ptr<std::vector<geometry::CNPointAllo>> points);
        bool passPossible(double cf, geometry::CNPointAllo passPoint,
                          geometry::CNPointAllo receiver, shared_ptr<msl::VoronoiNet> vNet);

        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1436269017402) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
}
/* namespace alica */

#endif /* SearchForPassPoint_H_ */
