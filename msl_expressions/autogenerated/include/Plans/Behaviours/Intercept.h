#ifndef Intercept_H_
#define Intercept_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1458757170147) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "container/CNVelocity2D.h"
/*PROTECTED REGION END*/
namespace alica
{
    class Intercept : public DomainBehaviour
    {
    public:
        Intercept();
        virtual ~Intercept();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1458757170147) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1458757170147) ENABLED START*/ //Add additional protected methods here
		msl::MSLFootballField* field;

		double maxVel = 2500;

		bool useZmachine;

		supplementary::SystemConfig* sc;
		msl::PathProxy* pp;


		double pdist;
		double pidist;
		double pddist;

		double lastDistErr;
		double distIntErr;

		double aheadWeight;

		double prot;
		double pirot;
		double pdrot;

		double lastRotErr;
		double rotIntErr;

		bool predictByRawOdo;


        bool  interceptPoint(shared_ptr<geometry::CNPoint2D> egoBall, shared_ptr<geometry::CNPoint2D> ballVel, double maxVel, double& t, shared_ptr<geometry::CNPoint2D>& interceptVelo);
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1458757170147) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Intercept_H_ */
