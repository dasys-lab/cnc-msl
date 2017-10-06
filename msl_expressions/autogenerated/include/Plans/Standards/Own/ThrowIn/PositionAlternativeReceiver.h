#ifndef PositionAlternativeReceiver_H_
#define PositionAlternativeReceiver_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462978634990) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
/*PROTECTED REGION END*/
namespace alica
{
    class PositionAlternativeReceiver : public DomainBehaviour
    {
    public:
        PositionAlternativeReceiver();
        virtual ~PositionAlternativeReceiver();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1462978634990) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1462978634990) ENABLED START*/ //Add additional protected methods here
        msl::MovementQuery query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1462978634990) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PositionAlternativeReceiver_H_ */
