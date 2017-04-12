#ifndef AlignToGoal_H_
#define AlignToGoal_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1415205272843) ENABLED START*/ //Add additional includes here
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPositionAllo.h>
#include <cnc_geometry/CNVecEgo.h>

#include "MSLFootballField.h"

using namespace msl;

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
        std::shared_ptr<geometry::CNPointAllo> alloAimPoint;
        double maxYTolerance;
        double pRot;
        double dRot;
        int iter;
        bool kicked;
        double goalLineHitPoint(std::shared_ptr<geometry::CNPositionAllo> ownPos, double egoAngle);
        double minFree(double angle, double width, std::shared_ptr<std::vector<double> > dstscan);
        int mod(int x, int y);
        std::shared_ptr<geometry::CNVecEgo> getFreeGoalVector();
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1415205272843) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlignToGoal_H_ */
