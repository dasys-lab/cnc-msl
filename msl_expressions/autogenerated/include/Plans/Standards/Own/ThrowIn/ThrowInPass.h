#ifndef ThrowInPass_H_
#define ThrowInPass_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462363192018) ENABLED START*/ //Add additional includes here
namespace geometry
{
    class CNPoint2D;
}
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
        shared_ptr<geometry::CNPoint2D> recPos1;
        shared_ptr<geometry::CNPoint2D> recPos2;
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
        bool outsideTriangle(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b,
                             shared_ptr<geometry::CNPoint2D> c, double tolerance,
                             shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points);
                         bool outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball,
                         shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth,
                         shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points);
                         bool outsideCorridore(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D>passPoint,
        							  double passCorridorWidth, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points);
							double minFree(double angle, double width, shared_ptr<vector<double> > dstscan);
									int mod(int x, int y);
        /*PROTECTED REGION END*/			};
		} /* namespace alica */

#endif /* ThrowInPass_H_ */
