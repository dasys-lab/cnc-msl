#ifndef KickOffPassDefault_H_
#define KickOffPassDefault_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1438778042140) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class KickOffPassDefault : public DomainBehaviour
    {
    public:
        KickOffPassDefault();
        virtual ~KickOffPassDefault();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1438778042140) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1438778042140) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1438778042140) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* KickOffPassDefault_H_ */
