#ifndef DriveToPoint_H_
#define DriveToPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1417620568675) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"

using namespace msl;
/*PROTECTED REGION END*/
namespace alica
{
    class DriveToPoint : public DomainBehaviour
    {
    public:
        DriveToPoint();
        virtual ~DriveToPoint();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1417620568675) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1417620568675) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1417620568675) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveToPoint_H_ */
