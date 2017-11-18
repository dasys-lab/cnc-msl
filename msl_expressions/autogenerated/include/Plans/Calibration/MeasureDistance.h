#ifndef MeasureDistance_H_
#define MeasureDistance_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1508940928986) ENABLED START*/ //Add additional includes here
#include <container/CNPoint2D.h>
/*PROTECTED REGION END*/
namespace alica
{
    class MeasureDistance : public DomainBehaviour
    {
    public:
        MeasureDistance();
        virtual ~MeasureDistance();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1508940928986) ENABLED START*/ //Add additional public methods here
        static double getOdometryDistance(DomainBehaviour* beh);
        static double getLaserDistance(DomainBehaviour* beh);
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1508940928986) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1508940928986) ENABLED START*/ //Add additional private methods here
        static shared_ptr<geometry::CNPoint2D> startPositionLaser;
        static shared_ptr<geometry::CNPoint2D> startPositionOdometry;
        static double distanceLaser;
        static double distanceOdometry;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* MeasureDistance_H_ */
