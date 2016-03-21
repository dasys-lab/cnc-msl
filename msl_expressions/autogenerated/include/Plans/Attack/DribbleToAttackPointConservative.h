#ifndef DribbleToAttackPointConservative_H_
#define DribbleToAttackPointConservative_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1458132872550) ENABLED START*/ //Add additional includes here
#include "DateTime.h"
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleToAttackPointConservative : public DomainBehaviour
    {
    public:
        DribbleToAttackPointConservative();
        virtual ~DribbleToAttackPointConservative();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1458132872550) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1458132872550) ENABLED START*/ //Add additional protected methods here
        void trueInitialize();
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1458132872550) ENABLED START*/ //Add additional private methods here
        msl::MSLFootballField* field;
        shared_ptr<geometry::CNPoint2D> currentTarget;
        vector<double> attackPosY;

        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleToAttackPointConservative_H_ */
