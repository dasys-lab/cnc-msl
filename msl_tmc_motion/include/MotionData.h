/*
 * MotionData.h
 *
 *  Created on: Aug 20, 2015
 *      Author: cnpaul
 */

#ifndef CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_MOTIONDATA_H_
#define CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_MOTIONDATA_H_

#include <DriverData.h>
namespace msl_driver
{

	class MotionSet : DriverData
	{
		public:
			double angle = 0.0;
			double translation = 0.0;
			double rotation = 0.0;

	};
	class ExtendedMotionResult : MotionSet
	{
		public:
			short pwm[];
	};
	class MotionSlipControl : DriverData
	{
		public:
			bool enabled = false;
	};
	class MotionCurrent : DriverData
	{
		public:
			double current[];
	};
	class MotionTemperature : DriverData
	{
		public:
			double temperature[];
	};
	class MotionSupplyVoltage : DriverData
	{
		public:
			double voltage = 0.0;
	};

} /* namespace msl_driver */

#endif /* CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_MOTIONDATA_H_ */
