#ifndef OneGernericInGameBlocker_H_
#define OneGernericInGameBlocker_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1458034268108) ENABLED START*/ //Add additional includes here
#include "engine/constraintmodul/Query.h"
#include "msl_robot/robotmovement/MovementQuery.h"
namespace msl
{
namespace robot
{
	class IntRobotID;
}
}
/*PROTECTED REGION END*/
namespace alica
{
    class OneGernericInGameBlocker : public DomainBehaviour
    {
    public:
        OneGernericInGameBlocker();
        virtual ~OneGernericInGameBlocker();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1458034268108) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1458034268108) ENABLED START*/ //Add additional protected methods here
        shared_ptr<Query> query;
        double maxVel;
        bool avoidBall;
        vector<double> result;
        ulong lastResultFound;
        ulong failTimeThreshold;
        string teamMateTaskName;
        string teamMatePlanName;
        EntryPoint* ep;
        const msl::robot::IntRobotID* teamMateId;
        shared_ptr<msl::MovementQuery> movQuery;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1458034268108) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* OneGernericInGameBlocker_H_ */
