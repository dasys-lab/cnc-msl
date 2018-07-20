#ifndef CheckPassMsg_H_
#define CheckPassMsg_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1457441479350) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class CheckPassMsg : public DomainBehaviour
    {
    public:
        CheckPassMsg();
        virtual ~CheckPassMsg();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1457441479350) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1457441479350) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1457441479350) ENABLED START*/ //Add additional private methods here
        bool receivedMsg;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CheckPassMsg_H_ */
