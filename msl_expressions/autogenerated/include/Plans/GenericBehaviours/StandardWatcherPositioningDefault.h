#ifndef StandardWatcherPositioningDefault_H_
#define StandardWatcherPositioningDefault_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1429109412171) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardWatcherPositioningDefault : public DomainBehaviour
    {
    public:
        StandardWatcherPositioningDefault();
        virtual ~StandardWatcherPositioningDefault();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1429109412171) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1429109412171) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1429109412171) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardWatcherPositioningDefault_H_ */
