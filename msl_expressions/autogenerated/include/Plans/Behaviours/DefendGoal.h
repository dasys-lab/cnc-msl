#ifndef DefendGoal_H_
#define DefendGoal_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1459249294699) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class DefendGoal : public DomainBehaviour
    {
    public:
        DefendGoal();
        virtual ~DefendGoal();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1459249294699) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1459249294699) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1459249294699) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DefendGoal_H_ */
