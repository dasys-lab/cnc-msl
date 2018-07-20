#ifndef StandardAlignToPoint_H_
#define StandardAlignToPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1433949970592) ENABLED START*/ //Add additional includes here
using namespace msl;
namespace geometry
{
    class CNPoint2D;
}
namespace msl
{
    class MovementQuery;
}
/*PROTECTED REGION END*/
namespace alica
{
    class StandardAlignToPoint : public DomainBehaviour
    {
    public:
        StandardAlignToPoint();
        virtual ~StandardAlignToPoint();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1433949970592) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1433949970592) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1433949970592) ENABLED START*/ //Add additional private methods here
        bool isReceiver;
        string taskName;

        double alignAngleTolerance;
        double positionDistanceTolerance;
        double executerDistanceToBall;
        double receiverDistanceToBall;
        double receiverBallMovedThreshold;
        shared_ptr<MovementQuery> m_Query;
        shared_ptr<geometry::CNPoint2D> alloReceiverTarget;
        shared_ptr<geometry::CNPoint2D> oldBallPos;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardAlignToPoint_H_ */
