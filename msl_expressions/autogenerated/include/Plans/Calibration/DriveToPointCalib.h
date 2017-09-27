#ifndef DriveToPointCalib_H_
#define DriveToPointCalib_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1474278265440) ENABLED START*/ //Add additional includes here
#include "cnc_geometry/CNPointEgo.h"
#include "cnc_geometry/CNPositionAllo.h"
#include "cnc_geometry/CNPointAllo.h"
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
        geometry::CNPointAllo alloTarget;
        double defaultTranslation;
        msl::MovementQuery query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1474278265440) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveToPointCalib_H_ */
