#ifndef TestGoalCoordinates_H_
#define TestGoalCoordinates_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1532092850344) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class TestGoalCoordinates : public DomainBehaviour
    {
    public:
        TestGoalCoordinates();
        virtual ~TestGoalCoordinates();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1532092850344) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1532092850344) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1532092850344) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* TestGoalCoordinates_H_ */
