#ifndef Pos2Penalty_H_
#define Pos2Penalty_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1465474139420) ENABLED START*/ //Add additional includes here
#include <engine/constraintmodul/Query.h>
#include <msl_robot/robotmovement/MovementQuery.h>
namespace geometry
{
    class CNPoint2D;
}
/*PROTECTED REGION END*/
namespace alica
{
    class Pos2Penalty : public DomainBehaviour
    {
    public:
        Pos2Penalty();
        virtual ~Pos2Penalty();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1465474139420) ENABLED START*/ //Add additional public methods here
        vector<double> result;
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1465474139420) ENABLED START*/ //Add additional protected methods here
        shared_ptr<alica::Query> query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1465474139420) ENABLED START*/ //Add additional private methods here
        shared_ptr<msl::MovementQuery> moveQuery;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Pos2Penalty_H_ */
