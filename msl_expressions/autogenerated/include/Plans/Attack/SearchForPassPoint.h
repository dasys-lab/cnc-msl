#ifndef SearchForPassPoint_H_
#define SearchForPassPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1436269017402) ENABLED START*/ //Add additional includes here
#define BEH_DEBUG
#include <vector>
#include "MSLFootballField.h"
#include <SystemConfig.h>
#include "engine/model/EntryPoint.h"
#include "GameState.h"
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
        msl::MSLFootballField* ff;
        double ballRadius;
        double passCorridorWidth;
        double maxTurnAngle;
        double minOppDist;
        double minPassDist;
        double maxPassDist;
        double distToFieldBorder;
        double minCloserOffset;
        double closerFactor;
        static bool outsideTriangle(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b,
                                    shared_ptr<geometry::CNPoint2D> c, double tolerance,
                                    shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> points);
                                static bool outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball,
                                shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth,
                                shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> points);
                                static bool outsideCorridore(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D> passPoint,
                                     double passCorridorWidth, shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> points);

        /*PROTECTED REGION END*/				private:
				/*PROTECTED REGION ID(prv1436269017402) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/			};
		} /* namespace alica */

#endif /* SearchForPassPoint_H_ */
