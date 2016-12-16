#ifndef DribbleControlMOS_H_
#define DribbleControlMOS_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1479905178049) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleControlMOS : public DomainBehaviour
    {
    public:
        DribbleControlMOS();
        virtual ~DribbleControlMOS();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1479905178049) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1479905178049) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1479905178049) ENABLED START*/ //Add additional private methods here
        virtual double getBallVelocity(double angle, double translation, double rotation);
        virtual double getBallAngle(double angle, double translation, double rotation);
        virtual double getLeftArmVelocity(double ballVelocity, double ballAngle);
        virtual double getRightArmVelocity(double ballVelocity, double ballAngle);

        int testBehaviour;
        int testSpeed;
        double testRot;
        double testAngle;
        int testCount;
        int testCount2;

        double velToInput;
        double staticUpperBound;
        double staticLowerBound;
        double staticNegVelX;
        double epsilonT;
        double epsilonRot;
        double rBallRobot;
        double forwConst;
        double sidewConst;
        double diagConst;
        double phi;

        supplementary::SystemConfig* sc;

        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleControlMOS_H_ */
