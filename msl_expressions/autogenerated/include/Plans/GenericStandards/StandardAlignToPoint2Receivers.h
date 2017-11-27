#ifndef StandardAlignToPoint2Receivers_H_
#define StandardAlignToPoint2Receivers_H_

#include "DomainBehaviour.h"
#include <msl_robot/robotmovement/MovementQuery.h>
/*PROTECTED REGION ID(inc1467228931063) ENABLED START*/ //Add additional includes here
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
        nonstd::optional<geometry::CNPointAllo> recPos1;
        nonstd::optional<geometry::CNPointAllo> recPos2;
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
        msl::MovementQuery m_Query;
        int canPassCounter;
        int canPassThreshold;

        bool outsideTriangle(geometry::CNPointAllo a, geometry::CNPointAllo b, geometry::CNPointAllo c,
                             double tolerance, vector<geometry::CNPointAllo>& points);
        bool outsideCorridoreTeammates(geometry::CNPointAllo ball, geometry::CNPointAllo passPoint,
                                       double passCorridorWidth, vector<geometry::CNPointAllo>& points);
        bool outsideCorridore(geometry::CNPointAllo ball, geometry::CNPointAllo passPoint, double passCorridorWidth,
                              vector<geometry::CNPointAllo>& points);
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardAlignToPoint2Receivers_H_ */
