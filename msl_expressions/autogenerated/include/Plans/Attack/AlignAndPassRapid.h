#ifndef AlignAndPassRapid_H_
#define AlignAndPassRapid_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1436269063295) ENABLED START*/ // Add additional includes here
#include "GameState.h"
#include "pathplanner/PathProxy.h"
#include <MSLFootballField.h>
#include <vector>
#define DBM_DEBUG 1
namespace supplementary
{
class SystemConfig;
class IAgentID;
}
/*PROTECTED REGION END*/
namespace alica
{
class AlignAndPassRapid : public DomainBehaviour
{
  public:
    AlignAndPassRapid();
    virtual ~AlignAndPassRapid();
    virtual void run(void *msg);
    /*PROTECTED REGION ID(pub1436269063295) ENABLED START*/ // Add additional public methods here
    /*PROTECTED REGION END*/
  protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1436269063295) ENABLED START*/ // Add additional protected methods here
    vector<string> teamMateTaskName;
    vector<string> teamMatePlanName;
    vector<EntryPoint *> eps;
    vector<const supplementary::IAgentID *> teamMateIds;
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
    const supplementary::IAgentID *bestTeamMateId;
    bool found;
    int best_point;
    shared_ptr<geometry::CNPoint2D> alloBall;
    shared_ptr<geometry::CNPoint2D> bestAoc;
    shared_ptr<geometry::CNPosition> alloPos;
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> matePoses;
#ifdef DBM_DEBUG
    shared_ptr<msl_helper_msgs::DebugMsg> dbm;
#endif

    // AlignStuff
    double maxVel;
    double pRot;
    double dRot;
    double lastRotError;
    double minRot;
    double maxRot;
    double accel;

    supplementary::SystemConfig *sc;
    shared_ptr<geometry::CNPoint2D> alloAimPoint;
    msl::PathProxy *pathProxy;
    static bool outsideTriangle(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b, shared_ptr<geometry::CNPoint2D> c, double tolerance,
                                shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points);

    static bool outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth,
                                          shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points);

    static bool outsideCorridore(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth,
                                 shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points);
    double minFree(double angle, double width, shared_ptr<vector<double>> dstscan);
    int mod(int x, int y);
    void findBestPassPoint(double cf, shared_ptr<geometry::CNPoint2D> passPoint, shared_ptr<geometry::CNPoint2D> receiver, shared_ptr<msl::VoronoiNet> vNet,
                           shared_ptr<geometry::CNPosition> teamMatePos, const supplementary::IAgentID *teamMateId);
    /*PROTECTED REGION END*/ private:
    /*PROTECTED REGION ID(prv1436269063295) ENABLED START*/ // Add additional private methods here
    /*PROTECTED REGION END*/};
    }
    /* namespace alica */

#endif /* AlignAndPassRapid_H_ */
