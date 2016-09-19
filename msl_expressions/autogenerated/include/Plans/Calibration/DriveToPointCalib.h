#ifndef DriveToPointCalib_H_
#define DriveToPointCalib_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1474278265440) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include <msl_robot/robotmovement/MovementQuery.h>

using namespace msl;
/*PROTECTED REGION END*/
namespace alica
{
    class DriveToPointCalib : public DomainBehaviour
    {
    public:
        DriveToPointCalib();
        virtual ~DriveToPointCalib();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1474278265440) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1474278265440) ENABLED START*/ //Add additional protected methods here
        geometry::CNPoint2D alloTarget;
        double defaultTranslation;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1474278265440) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveToPointCalib_H_ */
