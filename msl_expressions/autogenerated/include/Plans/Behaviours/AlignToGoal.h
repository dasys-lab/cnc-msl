#ifndef AlignToGoal_H_
#define AlignToGoal_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1415205272843) ENABLED START*/ //Add additional includes here
namespace geometry {
	class CNPoint2D;
	class CNPosition;
}
/*PROTECTED REGION END*/
namespace alica
{
    class AlignToGoal : public DomainBehaviour
    {
    public:
        AlignToGoal();
        virtual ~AlignToGoal();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1415205272843) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1415205272843) ENABLED START*/ //Add additional protected methods here
        double maxVel;
        double lastRotError;
        double maxRot;
        double minRot;
        shared_ptr<geometry::CNPoint2D> alloAimPoint;
        double yOffset;
        double maxYTolerance;
        double pRot;
        double dRot;
        double goalLineHitPoint(shared_ptr<geometry::CNPosition> ownPos, double egoAngle);
        shared_ptr<geometry::CNPoint2D> getFreeGoalVector();
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1415205272843) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlignToGoal_H_ */
