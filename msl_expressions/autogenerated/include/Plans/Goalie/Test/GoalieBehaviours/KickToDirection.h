#ifndef KickToDirection_H_
#define KickToDirection_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1447863478260) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class KickToDirection : public DomainBehaviour
    {
    public:
        KickToDirection();
        virtual ~KickToDirection();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1447863478260) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1447863478260) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1447863478260) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* KickToDirection_H_ */
