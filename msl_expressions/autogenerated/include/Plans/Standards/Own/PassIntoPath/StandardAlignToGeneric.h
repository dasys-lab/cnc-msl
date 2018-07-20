#ifndef StandardAlignToGeneric_H_
#define StandardAlignToGeneric_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1457531616421) ENABLED START*/ //Add additional includes here
#include "engine/constraintmodul/Query.h"
/*PROTECTED REGION END*/
namespace alica
{
    class StandardAlignToGeneric : public DomainBehaviour
    {
    public:
        StandardAlignToGeneric();
        virtual ~StandardAlignToGeneric();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1457531616421) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1457531616421) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1457531616421) ENABLED START*/ //Add additional private methods here
        shared_ptr<Query> query;
        vector<double> result;
        double maxVel;
        int iterationCount;

        double tol;
        double trans;

        bool haveBall = false;

        int delayKickCounter;
        double maxTranslation;
        double xOffSet;

        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardAlignToGeneric_H_ */
