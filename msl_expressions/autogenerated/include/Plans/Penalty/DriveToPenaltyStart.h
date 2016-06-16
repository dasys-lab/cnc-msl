#ifndef DriveToPenaltyStart_H_
#define DriveToPenaltyStart_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1459609457478) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
/*PROTECTED REGION END*/
namespace alica
{
    class DriveToPenaltyStart : public DomainBehaviour
    {
    public:
        DriveToPenaltyStart();
        virtual ~DriveToPenaltyStart();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1459609457478) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1459609457478) ENABLED START*/ //Add additional protected methods here
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1459609457478) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveToPenaltyStart_H_ */
