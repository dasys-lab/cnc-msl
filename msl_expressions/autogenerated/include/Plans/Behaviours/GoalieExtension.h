#ifndef GoalieExtension_H_
#define GoalieExtension_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1459249216387) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class GoalieExtension : public DomainBehaviour
    {
    public:
        GoalieExtension();
        virtual ~GoalieExtension();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1459249216387) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1459249216387) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1459249216387) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* GoalieExtension_H_ */
