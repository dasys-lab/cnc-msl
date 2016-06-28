#ifndef FetchFromSideLine_H_
#define FetchFromSideLine_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1450175655102) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
namespace geometry
{
    class CNPoint2D;
}
/*PROTECTED REGION END*/
namespace alica
{
    class FetchFromSideLine : public DomainBehaviour
    {
    public:
        FetchFromSideLine();
        virtual ~FetchFromSideLine();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1450175655102) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1450175655102) ENABLED START*/ //Add additional protected methods here
        double threshold;
        double behindDistance;
        double maxVel;
        bool nearSideLine(shared_ptr<geometry::CNPoint2D> alloBall);
        bool nearXLine(shared_ptr<geometry::CNPoint2D> alloBall);

        shared_ptr<msl::MovementQuery> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1450175655102) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* FetchFromSideLine_H_ */
