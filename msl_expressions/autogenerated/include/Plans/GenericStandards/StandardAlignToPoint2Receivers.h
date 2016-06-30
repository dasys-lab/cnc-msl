#ifndef StandardAlignToPoint2Receivers_H_
#define StandardAlignToPoint2Receivers_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1467228931063) ENABLED START*/ //Add additional includes here
using namespace msl;
namespace geometry
{
    class CNPoint2D;
}
namespace msl
{
    class MovementQuery;
}
/*PROTECTED REGION END*/
namespace alica
{
    class StandardAlignToPoint2Receivers : public DomainBehaviour
    {
    public:
        StandardAlignToPoint2Receivers();
        virtual ~StandardAlignToPoint2Receivers();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1467228931063) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1467228931063) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1467228931063) ENABLED START*/ //Add additional private methods here
        string teamMateTaskName1;
        string teamMateTaskName2;
        shared_ptr<geometry::CNPoint2D> recPos1;
        shared_ptr<geometry::CNPoint2D> recPos2;
        bool canPass;
        double alignAngleTolerance;
        double positionDistanceTolerance;
        double executerDistanceToBall;
        double receiverDistanceToBall;
        double receiverBallMovedThreshold;
        double minOppDist;
        double passCorridorWidth;
        double ballRadius;
        double maxTurnAngle;
        double ratio;
        shared_ptr<MovementQuery> m_Query;
        shared_ptr<geometry::CNPoint2D> alloReceiverTarget;
        shared_ptr<geometry::CNPoint2D> oldBallPos;

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

#endif /* StandardAlignToPoint2Receivers_H_ */
