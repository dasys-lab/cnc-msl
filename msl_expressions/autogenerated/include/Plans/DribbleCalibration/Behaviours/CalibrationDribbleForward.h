#ifndef CalibrationDribbleForward_H_
#define CalibrationDribbleForward_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1469116853584) ENABLED START*/ //Add additional includes here
#include <Plans/DribbleCalibration/Container/DribbleCalibrationContainer.h>
/*PROTECTED REGION END*/
namespace alica
{
	class CalibrationDribbleForward : public DomainBehaviour
	{
	public:
		CalibrationDribbleForward();
		virtual ~CalibrationDribbleForward();
		virtual void run(void* msg);
		/*PROTECTED REGION ID(pub1469116853584) ENABLED START*/ //Add additional public methods here
		/*PROTECTED REGION END*/
	protected:
		virtual void initialiseParameters();
		/*PROTECTED REGION ID(pro1469116853584) ENABLED START*/ //Add additional protected methods here
		DribbleCalibrationContainer dcc;

		vector<subsection> sections;

		static const int MAX_SPEED = 4000;
		static const int MAX_ROTATION = 5000;
		static const int SECTIONS_SIZE = 11;
		static const int MIN_ROTATE_NUMBER = 300;

		//const for checkBallRotation
		static const int TOO_SLOW = 2;
		static const int TOO_FAST = 3;
		static const int CORRECT = 4;
		static const int ROTATE_ERR = -10;

		int haveBallCount;bool increaseSpeed;
		int correctRotationCount;
		int changingFactor;

		int moveCount;

		// config Params
		double minRotation;

		void adaptWheelSpeed(int err);
		int checkBallRotation();
		void fillSections(shared_ptr<vector<string> > speedsSections);
		void createSections();
		void readConfigParameters();
		void writeConfigParameters();
		/*PROTECTED REGION END*/
	private:
		/*PROTECTED REGION ID(prv1469116853584) ENABLED START*/ //Add additional private methods here
		/*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CalibrationDribbleForward_H_ */
