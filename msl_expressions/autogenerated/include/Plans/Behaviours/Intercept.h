#ifndef Intercept_H_
#define Intercept_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1458757170147) ENABLED START*/ //Add additional includes here
namespace geometry
{
    class CNVelocity2D;
    class CNPoint2D;
    class CNPosition;
}
namespace msl
{
    class MovementQuery;
    class PathProxy;
}
/*PROTECTED REGION END*/
namespace alica
{
    class Intercept : public DomainBehaviour
    {
    public:
        Intercept();
        virtual ~Intercept();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1458757170147) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1458757170147) ENABLED START*/ //Add additional protected methods here
        double maxVel;

        msl::PathProxy* pp;

        double pdist;
        double pidist;
        double pddist;

        double minDistErr;

        double lastDistErr;
        double distIntErr;

        double prot;
        double pirot;
        double pdrot;

        double lastRotErr;
        double rotIntErr;

        double maxBallVelocity;
        double catchRadius;
        double mapInFieldOffset;

        double predictionTimestep; //in ms
        int predictionHorizon;

        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1458757170147) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Intercept_H_ */
