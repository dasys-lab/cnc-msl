#ifndef DriveAndAlignToPoint_H_
#define DriveAndAlignToPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1516808796983) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include <msl_robot/robotmovement/MovementQuery.h>

using namespace msl;
/*PROTECTED REGION END*/
namespace alica
{
    class DriveAndAlignToPoint : public DomainBehaviour
    {
    public:
        DriveAndAlignToPoint();
        virtual ~DriveAndAlignToPoint();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1516808796983) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1516808796983) ENABLED START*/ //Add additional protected methods here
        geometry::CNPoint2D alloTarget;
        geometry::CNPoint2D alloOrientationTarget;
        double catchRadius;
        double defaultTranslation;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1516808796983) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveAndAlignToPoint_H_ */
