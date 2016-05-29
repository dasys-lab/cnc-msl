#ifndef StandardAlignAndGrab2Receivers_H_
#define StandardAlignAndGrab2Receivers_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462368682104) ENABLED START*/ //Add additional includes here
namespace geometry{
	class CNPoint2D;
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
        /*PROTECTED REGION ID(pub1462368682104) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1462368682104) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1462368682104) ENABLED START*/ //Add additional private methods here
        string teamMateTaskName1;
        string teamMateTaskName2;
        shared_ptr<geometry::CNPosition> recPos1;
        shared_ptr<geometry::CNPosition> recPos2;
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
        bool canPass;
        int haveBallCounter;
        double trans;
        double angleIntErr;
        double oldAngleErr;
        double minTol;
        double tol;
        msl::InfoTime startTime;
        bool outsideTriangle(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b,
                             shared_ptr<geometry::CNPoint2D> c, double tolerance,
                             shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points);
                         bool outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball,
                         shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth,
                         shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points);
                         bool outsideCorridore(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D>passPoint,
							  double passCorridorWidth, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points);

        /*PROTECTED REGION END*/			};
		} /* namespace alica */

#endif /* StandardAlignAndGrab2Receivers_H_ */
