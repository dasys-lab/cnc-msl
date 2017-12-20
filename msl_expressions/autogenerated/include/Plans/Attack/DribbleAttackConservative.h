#ifndef DribbleAttackConservative_H_
#define DribbleAttackConservative_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1457967322925) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "msl_robot/robotmovement/MovementQuery.h"
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleAttackConservative : public DomainBehaviour
    {
    public:
        DribbleAttackConservative();
        virtual ~DribbleAttackConservative();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1457967322925) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1457967322925) ENABLED START*/ //Add additional protected methods here
        bool before;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1457967322925) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleAttackConservative_H_ */
