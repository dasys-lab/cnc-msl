#ifndef AlignAndShootTwoHoledWall_H_
#define AlignAndShootTwoHoledWall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1417620683982) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class AlignAndShootTwoHoledWall : public DomainBehaviour
    {
    public:
        AlignAndShootTwoHoledWall();
        virtual ~AlignAndShootTwoHoledWall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1417620683982) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1417620683982) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1417620683982) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlignAndShootTwoHoledWall_H_ */
