#ifndef CatchPass_H_
#define CatchPass_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1440754525537) ENABLED START*/ //Add additional includes here
namespace msl {
	class MovementQuery;
}
namespace geometry {
	class CNPoint2D;
}
/*PROTECTED REGION END*/
namespace alica
{
    class CatchPass : public DomainBehaviour
    {
    public:
        CatchPass();
        virtual ~CatchPass();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1440754525537) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1440754525537) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1440754525537) ENABLED START*/ //Add additional private methods here
        double maxVel;
        shared_ptr<geometry::CNPoint2D> passOrigin;
        shared_ptr<geometry::CNPoint2D> passVector;
        shared_ptr<geometry::CNPoint2D> passDestination;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CatchPass_H_ */
