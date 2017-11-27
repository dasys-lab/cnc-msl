#ifndef StandardAlignAndGrab2Receivers_H_
#define StandardAlignAndGrab2Receivers_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462368682104) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
#include <MSLWorldModel.h>

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
        msl::MovementQuery query;
        string teamMateTaskName1;
        string teamMateTaskName2;
        nonstd::optional<geometry::CNPointAllo> recPos1;
        nonstd::optional<geometry::CNPointAllo> recPos2;
        double ratio;
        double ballRadius;
        double passCorridorWidth;
        double maxTurnAngle;
        double minOppDist;
        double minCloserOffset;
        bool canPass;
        int haveBallCounter;
        double trans;
        double angleIntErr;
        double oldAngleErr;
        double minTol;
        double tol;
        msl::InfoTime startTime;
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

#endif /* StandardAlignAndGrab2Receivers_H_ */
