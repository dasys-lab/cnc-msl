#ifndef Tackle_H_
#define Tackle_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1434807660243) ENABLED START*/ //Add additional includes here
#include <math.h>
/*PROTECTED REGION END*/
namespace alica
{
    class Tackle : public DomainBehaviour
    {
    public:
        Tackle();
        virtual ~Tackle();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1434807660243) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1434807660243) ENABLED START*/ //Add additional protected methods here
		double maxVel;
		double ballDist;
		double ballDistTolerance;
		double rotationVel;
		msl::MSLFootballField* field;
		shared_ptr<geometry::CNPoint2D> alloEnemyGoal;
		shared_ptr<geometry::CNPoint2D> alloOwnGoal;
		int wiggleDir;
		double errorInt;
		supplementary::SystemConfig* sc;
		bool checkSide(shared_ptr<geometry::CNPoint2D> lineVector, shared_ptr<geometry::CNPoint2D> pointToCheck);
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1434807660243) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Tackle_H_ */
