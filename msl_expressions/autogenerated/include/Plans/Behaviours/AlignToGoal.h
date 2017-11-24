#ifndef AlignToGoal_H_
#define AlignToGoal_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1415205272843) ENABLED START*/ //Add additional includes here
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPositionAllo.h>
#include <cnc_geometry/CNVecEgo.h>
#include <MSLFootballField.h>
#include <nonstd/optional.hpp>
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
        nonstd::optional<geometry::CNPointEgo> aimEgo;
        double maxYTolerance;
        double pRot;
        double dRot;
        int iter;
        bool kicked;
        double minFree(double angle, double width, std::shared_ptr<std::vector<double> > dstscan);
        int mod(int x, int y);
        nonstd::optional<geometry::CNPointEgo> getFreeGoalVector();
        double goalLineHitPoint(geometry::CNPositionAllo ownPos, double egoAngle);
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1415205272843) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlignToGoal_H_ */
