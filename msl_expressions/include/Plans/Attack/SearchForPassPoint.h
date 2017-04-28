#ifndef SearchForPassPoint_H_
#define SearchForPassPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1436269017402) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class SearchForPassPoint : public DomainBehaviour
    {
    public:
        SearchForPassPoint();
        virtual ~SearchForPassPoint();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1436269017402) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1436269017402) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1436269017402) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* SearchForPassPoint_H_ */
