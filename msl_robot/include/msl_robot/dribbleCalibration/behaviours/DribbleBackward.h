#pragma once

#include <msl_robot/dribbleCalibration/behaviours/interfaces/ICalibration.h>
#include <msl_robot/dribbleCalibration/container/MovementContainer.h>
#include <memory>

namespace msl
{
	class DribbleBackward : public ICalibration
	{
	public:
		DribbleBackward();
		virtual ~DribbleBackward();
		shared_ptr<DribbleCalibrationQuery> move(int trans);
		void writeConfigParameters();
		void readConfigParameters();
		void adaptParams();
		void resetParams();
		void saveParams();

	private:
		MovementContainer mCon;

		// parameters
		double epsilonT;

		// calibration parameters
		double changingValue;
		double defaultValue;

		// save parameters
		double maxValue;
		double minValue;

		// ball handling
		double actuatorSpeed;
	};
}
