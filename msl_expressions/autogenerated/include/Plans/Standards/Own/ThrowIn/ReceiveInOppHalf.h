#ifndef ReceiveInOppHalf_H_
#define ReceiveInOppHalf_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462370340143) ENABLED START*/ //Add additional includes here
#include <engine/constraintmodul/Query.h>
#include <msl_robot/robotmovement/MovementQuery.h>
namespace geometry
{
    class CNPoint2D;
}
/*PROTECTED REGION END*/
namespace alica
{
    class ReceiveInOppHalf : public DomainBehaviour
    {
    public:
        ReceiveInOppHalf();
        virtual ~ReceiveInOppHalf();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1462370340143) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1462370340143) ENABLED START*/ //Add additional protected methods here
        vector<double> result;
        shared_ptr<alica::Query> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1462370340143) ENABLED START*/ //Add additional private methods here
        geometry::CNPointAllo alloTarget;
        string taskName;
        msl::MovementQuery mQuery;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ReceiveInOppHalf_H_ */
