#ifndef StandardAlignToPassPos_H_
#define StandardAlignToPassPos_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1457532279657) ENABLED START*/ //Add additional includes here
#include "engine/constraintmodul/Query.h"
#include "msl_robot/robotmovement/MovementQuery.h"
/*PROTECTED REGION END*/
namespace alica
{
    class StandardAlignToPassPos : public DomainBehaviour
    {
    public:
        StandardAlignToPassPos();
        virtual ~StandardAlignToPassPos();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1457532279657) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1457532279657) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1457532279657) ENABLED START*/ //Add additional private methods here
        shared_ptr<Query> query;
        vector<double> result;
        double maxVel;
        int iterationCount;
        msl::MovementQuery movQuery;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardAlignToPassPos_H_ */
