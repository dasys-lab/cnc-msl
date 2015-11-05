#ifndef AlignToRobot_H_
#define AlignToRobot_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1438779266783) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class AlignToRobot : public DomainBehaviour
    {
    public:
        AlignToRobot();
        virtual ~AlignToRobot();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1438779266783) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1438779266783) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1438779266783) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlignToRobot_H_ */
