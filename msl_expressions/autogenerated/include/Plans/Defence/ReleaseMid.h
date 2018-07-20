#ifndef ReleaseMid_H_
#define ReleaseMid_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1458033482289) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
/*PROTECTED REGION END*/
namespace alica
{
    class ReleaseMid : public DomainBehaviour
    {
    public:
        ReleaseMid();
        virtual ~ReleaseMid();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1458033482289) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1458033482289) ENABLED START*/ //Add additional protected methods here
        string teamMateTaskName;
        string teamMatePlanName;
        EntryPoint* ep;
        int teamMateId;
        double threshold;
        double yHysteresis;
        double vMax;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1458033482289) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ReleaseMid_H_ */
