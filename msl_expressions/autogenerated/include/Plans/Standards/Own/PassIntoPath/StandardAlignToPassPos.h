#ifndef StandardAlignToPassPos_H_
#define StandardAlignToPassPos_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1457532279657) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardAlignToPassPos : public DomainBehaviour
    {
    public:
        StandardAlignToPassPos();
        virtual ~StandardAlignToPassPos();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1457532279657) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1457532279657) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1457532279657) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardAlignToPassPos_H_ */
