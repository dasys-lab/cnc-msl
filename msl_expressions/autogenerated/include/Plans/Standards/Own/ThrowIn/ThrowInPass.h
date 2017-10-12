#ifndef ThrowInPass_H_
#define ThrowInPass_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462363192018) ENABLED START*/ //Add additional includes here
#include <nonstd/optional.hpp>
#include <cnc_geometry/CNPositionAllo.h>
#include <cnc_geometry/CNPositionEgo.h>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
/*PROTECTED REGION END*/
namespace alica
{
    class ThrowInPass : public DomainBehaviour
    {
    public:
        ThrowInPass();
        virtual ~ThrowInPass();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1462363192018) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1462363192018) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1462363192018) ENABLED START*/ //Add additional private methods here
        bool canPass;
        string teamMateTaskName1;
        string teamMateTaskName2;
        nonstd::optional<geometry::CNPointAllo> recPos1;
        nonstd::optional<geometry::CNPointAllo> recPos2;
        double ratio;
        double ballRadius;
        double passCorridorWidth;
        double maxTurnAngle;
        double minOppDist;
        double closerFactor;
        double maxVel;
        double pRot;
        double dRot;
        double lastRotError;
        double minRot;
        double maxRot;
        double accel;
        double arrivalTimeOffset;
        bool sentPm;
        bool outsideTriangle(geometry::CNPointAllo a, geometry::CNPointAllo b, geometry::CNPointAllo c,
                             double tolerance, vector<geometry::CNPointAllo> &points);
        bool outsideCorridoreTeammates(geometry::CNPointAllo ball, geometry::CNPointAllo passPoint,
                                       double passCorridorWidth, vector<geometry::CNPointAllo> &points);
        bool outsideCorridore(geometry::CNPointAllo ball, geometry::CNPointAllo passPoint, double passCorridorWidth,
                              vector<geometry::CNPointAllo> &points);
        double minFree(double angle, double width, shared_ptr<vector<double> > dstscan);
        int mod(int x, int y);
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ThrowInPass_H_ */
