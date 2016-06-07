#ifndef DefendGoal_H_
#define DefendGoal_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1459249294699) ENABLED START*/ //Add additional includes here
#include "container/CNPoint3D.h"
#include "robotmovement/MovementQuery.h"

/*PROTECTED REGION END*/
namespace alica
{
    class DefendGoal : public DomainBehaviour
    {
    public:
        DefendGoal();
        virtual ~DefendGoal();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1459249294699) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1459249294699) ENABLED START*/ //Add additional protected methods here
        double postOffset;
        double fieldOffset;
        double ownPosAngleMin;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1459249294699) ENABLED START*/ //Add additional private methods here#
        shared_ptr<geometry::CNPoint2D> applyBoundaries4Pos(shared_ptr<geometry::CNPoint2D> dest, double postOffset);
        double getSpeed(shared_ptr<geometry::CNPoint2D> dest);
        msl_actuator_msgs::MotionControl applyBoundaries4Heading(msl_actuator_msgs::MotionControl mc,
                                                                 shared_ptr<geometry::CNPosition> ownPos,
                                                                 shared_ptr<geometry::CNPoint2D> ballPos,
                                                                 double ownPosAngleMin);
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DefendGoal_H_ */
