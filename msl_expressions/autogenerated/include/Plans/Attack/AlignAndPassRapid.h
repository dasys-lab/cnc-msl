#ifndef AlignAndPassRapid_H_
#define AlignAndPassRapid_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1436269063295) ENABLED START*/ //Add additional includes here
#include <vector>
#include <MSLFootballField.h>
#include "GameState.h"
#include "pathplanner/PathProxy.h"
#define DBM_DEBUG 1
namespace supplementary
{
    class SystemConfig;
}
/*PROTECTED REGION END*/
namespace alica
{
    class AlignAndPassRapid : public DomainBehaviour
    {
    public:
        AlignAndPassRapid();
        virtual ~AlignAndPassRapid();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1436269063295) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1436269063295) ENABLED START*/ //Add additional protected methods here
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
        double arrivalTimeOffset;
        double bestPassUtility;
        int bestTeamMateId;
        bool found;
        int best_point;
        nonstd::optional<geometry::CNPointAllo> alloBall;
        geometry::CNPointAllo bestAoc;
        nonstd::optional<geometry::CNPositionAllo> alloPos;
        nonstd::optional<vector<geometry::CNPointAllo>> matePoses;
#ifdef DBM_DEBUG
        shared_ptr<msl_helper_msgs::DebugMsg> dbm;
#endif

        //AlignStuff
        double maxVel;
        double pRot;
        double dRot;
        double lastRotError;
        double minRot;
        double maxRot;
        double accel;

        supplementary::SystemConfig* sc;
        geometry::CNPointAllo alloAimPoint;
        msl::PathProxy* pathProxy;
        static bool outsideTriangle(geometry::CNPointAllo a, geometry::CNPointAllo b, geometry::CNPointAllo c,
                                    double tolerance, shared_ptr<vector<geometry::CNPointAllo>> points);

        static bool outsideCorridoreTeammates(geometry::CNPointAllo ball, geometry::CNPointAllo passPoint,
                                              double passCorridorWidth,
                                              vector<geometry::CNPointAllo> points);

        static bool outsideCorridore(geometry::CNPointAllo ball, geometry::CNPointAllo passPoint,
                                     double passCorridorWidth, shared_ptr<vector<geometry::CNPointAllo>> points);
        double minFree(double angle, double width, shared_ptr<vector<double> > dstscan);
        int mod(int x, int y);
        void findBestPassPoint(double cf, geometry::CNPointAllo passPoint, geometry::CNPointAllo receiver,
                               shared_ptr<msl::VoronoiNet> vNet, geometry::CNPositionAllo teamMatePos, int teamMateId);
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1436269063295) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
}
/* namespace alica */

#endif /* AlignAndPassRapid_H_ */
