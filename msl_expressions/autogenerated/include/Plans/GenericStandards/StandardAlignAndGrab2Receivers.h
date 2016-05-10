#ifndef StandardAlignAndGrab2Receivers_H_
#define StandardAlignAndGrab2Receivers_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1462368682104) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardAlignAndGrab2Receivers : public DomainBehaviour
    {
    public:
        StandardAlignAndGrab2Receivers();
        virtual ~StandardAlignAndGrab2Receivers();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1462368682104) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1462368682104) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1462368682104) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardAlignAndGrab2Receivers_H_ */
