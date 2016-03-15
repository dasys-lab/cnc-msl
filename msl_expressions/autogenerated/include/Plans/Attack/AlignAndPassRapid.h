#ifndef AlignAndPassRapid_H_
#define AlignAndPassRapid_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1436269063295) ENABLED START*/ //Add additional includes here
#include <vector>
#include <MSLFootballField.h>
#include "GameState.h"
#include "pathplanner/PathProxy.h"
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
        double arrivalTimeOffset;

        //AlignStuff
        double maxVel;
        double pRot;
        double dRot;
        double lastRotError;
        double minRot;
        double maxRot;
        double accel;

        SystemConfig* sc;
        shared_ptr<geometry::CNPoint2D> alloAimPoint;
        msl::MSLFootballField* field;
        msl::PathProxy* pathProxy;
        static bool outsideTriangle(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b,
                                    shared_ptr<geometry::CNPoint2D> c, double tolerance,
                                    shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> points);

                                static bool outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball,
                                shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth,
                                shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points);

                                static bool outsideCorridore(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D> passPoint,
		                                     double passCorridorWidth, shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> points);
		double minFree(double angle, double width, shared_ptr<vector<double> > dstscan);
		int mod(int x, int y);
		/*PROTECTED REGION END*/				private:
				/*PROTECTED REGION ID(prv1436269063295) ENABLED START*/ //Add additional private methods here
		/*PROTECTED REGION END*/			};
		} /* namespace alica */

#endif /* AlignAndPassRapid_H_ */
