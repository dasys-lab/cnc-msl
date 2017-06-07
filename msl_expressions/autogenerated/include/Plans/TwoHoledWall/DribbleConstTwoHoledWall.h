#ifndef DribbleConstTwoHoledWall_H_
#define DribbleConstTwoHoledWall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1496840140891) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/BallHandleCmd.h"
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleConstTwoHoledWall : public DomainBehaviour
    {
    public:
        DribbleConstTwoHoledWall();
        virtual ~DribbleConstTwoHoledWall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1496840140891) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1496840140891) ENABLED START*/ //Add additional protected methods here
        int wheelSpeed;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1496840140891) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleConstTwoHoledWall_H_ */
