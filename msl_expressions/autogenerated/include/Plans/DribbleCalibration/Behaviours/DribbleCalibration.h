#ifndef DribbleCalibration_H_
#define DribbleCalibration_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1482339434271) ENABLED START*/ //Add additional includes here
#include "Plans/DribbleCalibration/Container/DribbleCalibrationContainer.h"
#include "Plans/DribbleCalibration/Container/MovementContainer.h"
/*PROTECTED REGION END*/
namespace alica
{
	class DribbleCalibration : public DomainBehaviour
	{
	public:
		DribbleCalibration();
		virtual ~DribbleCalibration();
		virtual void run(void* msg);
		/*PROTECTED REGION ID(pub1482339434271) ENABLED START*/ //Add additional public methods here
		/*PROTECTED REGION END*/
	protected:
		virtual void initialiseParameters();
		/*PROTECTED REGION ID(pro1482339434271) ENABLED START*/ //Add additional protected methods here
		/*PROTECTED REGION END*/
	private:
		/*PROTECTED REGION ID(prv1482339434271) ENABLED START*/ //Add additional private methods here
		bool dribbleForward;
		bool dribbleBackward;
		DribbleCalibrationContainer dcc;
		DribbleCalibrationContainer::Parm parm;
		MovementContainer moveCont;

		// for speed iterations
		double startTrans;
		double endTrans;
		int speedIter;
		int moveCount;
		int getBallCount;

		// for movement
		bool getBallFlag;
		int haveBallCount;
		int haveBallWaitingDuration;
		int collectDataWaitingDuration;

		// rotation stuff
		int minHaveBallIter;
		double minHaveBallParamPoint;
		double maxHaveBallParamPoint;
		void readConfigParameters();
		/*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleCalibration_H_ */
