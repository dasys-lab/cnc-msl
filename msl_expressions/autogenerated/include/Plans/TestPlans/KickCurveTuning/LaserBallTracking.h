#ifndef LaserBallTracking_H_
#define LaserBallTracking_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1457698662032) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class LaserBallTracking : public DomainBehaviour
    {
    public:
        LaserBallTracking();
        virtual ~LaserBallTracking();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1457698662032) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1457698662032) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1457698662032) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* LaserBallTracking_H_ */
