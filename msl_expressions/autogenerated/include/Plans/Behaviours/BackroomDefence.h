#ifndef BackroomDefence_H_
#define BackroomDefence_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1454507752863) ENABLED START*/ //Add additional includes here
#include "cnc_geometry/CNPointAllo.h"
#include "cnc_geometry/CNPositionAllo.h"
#include "cnc_geometry/CNVecAllo.h"
#include "msl_robot/robotmovement/MovementQuery.h"
/*PROTECTED REGION END*/
namespace alica
{
    class BackroomDefence : public DomainBehaviour
    {
    public:
        BackroomDefence();
        virtual ~BackroomDefence();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1454507752863) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1454507752863) ENABLED START*/ //Add additional protected methods here
        geometry::CNPointAllo alloTarget;
        msl::MovementQuery query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1454507752863) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* BackroomDefence_H_ */
