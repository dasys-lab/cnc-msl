#pragma once

#include <msl_robot/dribbleCalibration/behaviours/interfaces/ICalibration.h>
#include <msl_robot/dribbleCalibration/container/MovementContainer.h>
#include <memory>

namespace msl
{
	class DribbleRotateRight : public ICalibration
	{
	public:
		DribbleRotateRight();
		virtual ~DribbleRotateRight();

		shared_ptr<DribbleCalibrationQuery> move(int trans);
		void writeConfigParameters();
		void readConfigParameters();
		void adaptParams();
		void resetParams();
		void saveParams();

	private:
		MovementContainer mCon;

		// parameters
		double epsilonRot;

		// calibration parameters
		double changingValue;
		double defaultValue;

		double rotationSpeed;

		// save parameters
		double minValue;
		double maxValue;
	};

} /* namespace alica */
