#ifndef SingleRobotKickIntoOppHalf_H_
#define SingleRobotKickIntoOppHalf_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1467436234548) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class SingleRobotKickIntoOppHalf : public DomainBehaviour
    {
    public:
        SingleRobotKickIntoOppHalf();
        virtual ~SingleRobotKickIntoOppHalf();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1467436234548) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1467436234548) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1467436234548) ENABLED START*/ //Add additional private methods here
        double pRot;
        double dRot;
        double lastRotError;
        double minRot;
        double maxRot;
        double maxVel;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* SingleRobotKickIntoOppHalf_H_ */
