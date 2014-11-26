#ifndef AlineToGoal_H_
#define AlineToGoal_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1415205272843) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"

using namespace msl;

/*PROTECTED REGION END*/
namespace alica
{
    class AlineToGoal : public DomainBehaviour
    {
    public:
        AlineToGoal();
        virtual ~AlineToGoal();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1415205272843) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1415205272843) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1415205272843) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlineToGoal_H_ */
