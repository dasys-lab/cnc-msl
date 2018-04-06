#ifndef PosAlternativePassReceiver_H_
#define PosAlternativePassReceiver_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1461674942156) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
namespace geometry
{
    class CNPoint2D;
}
/*PROTECTED REGION END*/
namespace alica
{
    class PosAlternativePassReceiver : public DomainBehaviour
    {
    public:
        PosAlternativePassReceiver();
        virtual ~PosAlternativePassReceiver();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1461674942156) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1461674942156) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1461674942156) ENABLED START*/ //Add additional private methods here
        shared_ptr<geometry::CNPoint2D> alloTarget;
        string taskName;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PosAlternativePassReceiver_H_ */
