#ifndef CoverSpace_H_
#define CoverSpace_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1455537892946) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
#include <cnc_geometry/CNPointAllo.h>
#include <nonstd/optional.hpp>
/*PROTECTED REGION END*/
namespace alica
{
    class CoverSpace : public DomainBehaviour
    {
    public:
        CoverSpace();
        virtual ~CoverSpace();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1455537892946) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1455537892946) ENABLED START*/ //Add additional protected methods here
        double positionPercentage;
        nonstd::optional<geometry::CNPointAllo> lastPos;
        msl::MovementQuery query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1455537892946) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CoverSpace_H_ */
