#ifndef Pos2Defenders_H_
#define Pos2Defenders_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1444834678756) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
namespace geometry
{
    class CNPosition;
}
namespace supplementary{
	class IAgentID;
}
/*PROTECTED REGION END*/
namespace alica
{
    class Pos2Defenders : public DomainBehaviour
    {
    public:
        Pos2Defenders();
        virtual ~Pos2Defenders();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1444834678756) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1444834678756) ENABLED START*/ //Add additional protected methods here
        const supplementary::IAgentID* keeperId;
        shared_ptr<geometry::CNPosition> keeperPos;
        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1444834678756) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Pos2Defenders_H_ */
