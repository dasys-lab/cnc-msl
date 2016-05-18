#ifndef ReceiveInOppHalf_H_
#define ReceiveInOppHalf_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462370340143) ENABLED START*/ //Add additional includes here
#include <engine/constraintmodul/ConstraintQuery.h>
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
        vector<double> result;
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1462370340143) ENABLED START*/ //Add additional protected methods here
        shared_ptr<alica::ConstraintQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1462370340143) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ReceiveInOppHalf_H_ */
