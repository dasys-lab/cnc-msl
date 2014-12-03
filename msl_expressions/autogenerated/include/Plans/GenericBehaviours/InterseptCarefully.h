#ifndef InterseptCarefully_H_
#define InterseptCarefully_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1417620641918) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class InterseptCarefully : public DomainBehaviour
    {
    public:
        InterseptCarefully();
        virtual ~InterseptCarefully();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1417620641918) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1417620641918) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1417620641918) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* InterseptCarefully_H_ */
