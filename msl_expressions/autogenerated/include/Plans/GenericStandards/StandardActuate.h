#ifndef StandardActuate_H_
#define StandardActuate_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1435766212595) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class StandardActuate : public DomainBehaviour
    {
    public:
        StandardActuate();
        virtual ~StandardActuate();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1435766212595) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1435766212595) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1435766212595) ENABLED START*/ //Add additional private methods here
        void readConfigParameters();
        double wheelSpeed;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* StandardActuate_H_ */
