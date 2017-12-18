#ifndef Parking_H_
#define Parking_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1429111623710) ENABLED START*/ //Add additional includes here
namespace supplementary
{
    class SystemConfig;
}
namespace geometry
{
    class CNPoint2D;
}
namespace msl
{
    class MovementQuery;
    class RobotMovement;
}
/*PROTECTED REGION END*/
namespace alica
{
    class Parking : public DomainBehaviour
    {
    public:
        Parking();
        virtual ~Parking();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1429111623710) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1429111623710) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1429111623710) ENABLED START*/ //Add additional private methods here
        supplementary::SystemConfig* sc;
        shared_ptr<geometry::CNPoint2D> parkingPosition;
        shared_ptr<msl::MovementQuery> movementQuery;
        double offset;
        int parkingSlotIdx;
        double distanceToParkingPositionTolerance;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Parking_H_ */
