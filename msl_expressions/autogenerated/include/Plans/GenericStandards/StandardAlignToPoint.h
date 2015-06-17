#ifndef StandardAlignToPoint_H_
#define StandardAlignToPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1433949970592) ENABLED START*/ //Add additional includes here
#include "MSLFootballField.h"
/*PROTECTED REGION END*/

namespace alica
{
    class StandardAlignToPoint : public DomainBehaviour
    {
    public:
        StandardAlignToPoint();
        virtual ~StandardAlignToPoint();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1433949970592) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1433949970592) ENABLED START*/ //Add additional protected methods here
        geometry::CNPoint2D alloTarget;
        bool isReceiver;
        msl::MSLFootballField* field;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1433949970592) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardAlignToPoint_H_ */
