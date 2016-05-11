#ifndef BounceShotAlignWall_H_
#define BounceShotAlignWall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1459355002202) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class BounceShotAlignWall : public DomainBehaviour
    {
    public:
        BounceShotAlignWall();
        virtual ~BounceShotAlignWall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1459355002202) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1459355002202) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1459355002202) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* BounceShotAlignWall_H_ */
