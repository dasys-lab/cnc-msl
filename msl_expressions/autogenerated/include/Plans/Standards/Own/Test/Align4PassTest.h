#ifndef Align4PassTest_H_
#define Align4PassTest_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1513609382468) ENABLED START*/ //Add additional includes here
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
    class Align4PassTest : public DomainBehaviour
    {
    public:
        Align4PassTest();
        virtual ~Align4PassTest();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1513609382468) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1513609382468) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1513609382468) ENABLED START*/ //Add additional private methods here
        bool isReceiver;
        string taskName;

        double alignAngleTolerance;
        double positionDistanceTolerance;
        double executerDistanceToBall;
        double receiverDistanceToBall;
        double receiverBallMovedThreshold;
        shared_ptr<msl::MovementQuery> m_Query;
        shared_ptr<geometry::CNPoint2D> alloReceiverTarget;
        shared_ptr<geometry::CNPoint2D> oldBallPos;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Align4PassTest_H_ */
