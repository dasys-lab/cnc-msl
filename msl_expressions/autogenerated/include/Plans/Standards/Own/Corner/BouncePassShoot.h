#ifndef BouncePassShoot_H_
#define BouncePassShoot_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1459357144291) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
/*PROTECTED REGION END*/
namespace alica
{
    class BouncePassShoot : public DomainBehaviour
    {
    public:
        BouncePassShoot();
        virtual ~BouncePassShoot();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1459357144291) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1459357144291) ENABLED START*/ //Add additional protected methods here
        string planName;
        string teamMateTaskName;
        EntryPoint* receiver;
        int counter;
        double driveSlowSpeed;
        msl::MovementQuery query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1459357144291) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* BouncePassShoot_H_ */
