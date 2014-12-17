#ifndef AlignAndShootTwoHoledBall_H_
#define AlignAndShootTwoHoledBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1417620683982) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPoint3D.h"
#include "MSLFootballField.h"

using namespace msl;
/*PROTECTED REGION END*/
namespace alica
{
    class AlignAndShootTwoHoledBall : public DomainBehaviour
    {
    public:
        AlignAndShootTwoHoledBall();
        virtual ~AlignAndShootTwoHoledBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1417620683982) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1417620683982) ENABLED START*/ //Add additional protected methods here

		bool useLowerHole = false; // vorher true

		MSLFootballField* field;
		int timesOnTarget=0;


		double maxVel;
		double pRot;
		double dRot;
		double lastRotError;
		double minRot;
		double maxRot;
		double angleTolerance;
		bool disableKicking;

		CNPoint3D higherHole;
		CNPoint3D lowerHole;
		bool usedFixedHole;
		bool useLowerHoleFixed;
		double shootingSpeed;
		int TIMES_ON_TARGET;
		int wheelSpeed;
		double voltage4shoot;

		vector<shared_ptr<CNPoint2D>> highKickList;
		vector<shared_ptr<CNPoint2D>> lowKickList;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1417620683982) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlignAndShootTwoHoledBall_H_ */
