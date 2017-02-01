/*
 * DribbleForward.h
 *
 *  Created on: Dec 14, 2016
 *      Author: cn
 */

#ifndef AUTOGENERATED_SRC_PLANS_DRIBBLECALIBRATION_BEHAVIOURS_CALIBRATIONS_DRIBBLEFORWARD_H_
#define AUTOGENERATED_SRC_PLANS_DRIBBLECALIBRATION_BEHAVIOURS_CALIBRATIONS_DRIBBLEFORWARD_H_

#include <Plans/DribbleCalibration/Behaviours/Interfaces/ICalibration.h>
#include <Plans/DribbleCalibration/Container/MovementContainer.h>
#include <memory>

namespace alica
{
	class DribbleForward : public ICalibration
	{
	public:
		DribbleForward();
		virtual ~DribbleForward();

		shared_ptr<DribbleCalibrationQuery> move(int trans);
		void writeConfigParameters();
		void readConfigParameters();
		void adaptParams();
		void resetParams();
		void saveParams();

	private:
		MovementContainer mCon;
		shared_ptr<DribbleCalibrationQuery> query;

		// parameters
		double velToInput;

		// calibration parameters
		double changingValue;
		double defaultValue;

		// saving parameters
		double minValue;
		double maxValue;

		// ball handling
		double actuatorSpeed;
	};
}
#endif /* AUTOGENERATED_SRC_PLANS_DRIBBLECALIBRATION_BEHAVIOURS_CALIBRATIONS_DRIBBLEFORWARD_H_ */
