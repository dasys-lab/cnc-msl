#ifndef Pos4ReceiverCornerKick_H_
#define Pos4ReceiverCornerKick_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1464787469281) ENABLED START*/ //Add additional includes here
#include "engine/constraintmodul/Query.h"
#include <msl_robot/robotmovement/MovementQuery.h>
namespace geometry
{
    class CNPoint2D;
}
/*PROTECTED REGION END*/
namespace alica
{
    class Pos4ReceiverCornerKick : public DomainBehaviour
    {
    public:
        Pos4ReceiverCornerKick();
        virtual ~Pos4ReceiverCornerKick();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1464787469281) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1464787469281) ENABLED START*/ //Add additional protected methods here
        vector<double> result;
        shared_ptr<alica::Query> query;
        shared_ptr<msl::MovementQuery> mQuery;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1464787469281) ENABLED START*/ //Add additional private methods here
        shared_ptr<geometry::CNPoint2D> alloTarget;
        string taskName;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Pos4ReceiverCornerKick_H_ */
