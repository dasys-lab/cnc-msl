#ifndef OneEighty_H_
#define OneEighty_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1434650892176) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class OneEighty : public DomainBehaviour
    {
    public:
        OneEighty();
        virtual ~OneEighty();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1434650892176) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1434650892176) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1434650892176) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* OneEighty_H_ */
