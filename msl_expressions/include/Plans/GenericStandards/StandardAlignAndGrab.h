#ifndef StandardAlignAndGrab_H_
#define StandardAlignAndGrab_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1455888574532) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardAlignAndGrab : public DomainBehaviour
    {
    public:
        StandardAlignAndGrab();
        virtual ~StandardAlignAndGrab();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1455888574532) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1455888574532) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1455888574532) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardAlignAndGrab_H_ */
