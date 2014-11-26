#ifndef DribbleToPoint_H_
#define DribbleToPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1414752367688) ENABLED START*/ //Add additional includes here
#include <tuple>
#include <MSLWorldModel.h>
#include "GeometryTransformer.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"

using namespace msl;
using namespace std;
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleToPoint : public DomainBehaviour
    {
    public:
        DribbleToPoint();
        virtual ~DribbleToPoint();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1414752367688) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1414752367688) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1414752367688) ENABLED START*/ //Add additional private methods here
        CNPoint2D alloBallPos;

        CNPosition ownPos;
        CNPoint2D egoBallPos;

        CNPoint2D alloTargetPos;
        CNPoint2D egoTargetPos;

        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleToPoint_H_ */
