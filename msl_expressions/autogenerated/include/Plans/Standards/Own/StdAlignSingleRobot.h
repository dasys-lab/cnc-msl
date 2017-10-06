#ifndef StdAlignSingleRobot_H_
#define StdAlignSingleRobot_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1467385758084) ENABLED START*/ //Add additional includes here
namespace msl
{
    class MovementQuery;
}
/*PROTECTED REGION END*/
namespace alica
{
    class StdAlignSingleRobot : public DomainBehaviour
    {
    public:
        StdAlignSingleRobot();
        virtual ~StdAlignSingleRobot();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1467385758084) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1467385758084) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1467385758084) ENABLED START*/ //Add additional private methods here
        msl::MovementQuery m_Query;
        double executorDistanceToBall;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StdAlignSingleRobot_H_ */
