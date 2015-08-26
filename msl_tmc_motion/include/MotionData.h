/*
 * MotionData.h
 *
 *  Created on: Aug 20, 2015
 *      Author: cnpaul
 */

#ifndef CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_MOTIONDATA_H_
#define CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_MOTIONDATA_H_

namespace msl_driver
{

	class MotionSet
	{
		public:
			double angle = 0.0;
			double translation = 0.0;
			double rotation = 0.0;
			long timespamp;

	};
	class ExtendedMotionResult : MotionSet
	{
		public:
			short pvm[];
	};
	class MotionSlipControl : MotionSet
	{
		public:
			bool enabled = false;
	};
	class MotionCurrent
	{
		public:
			double current[];
	};
	class MotionTemperature
	{
		public:
			double temperature[];
	};
	class MotionSupplyVoltage
	{
		public:
			double voltage = 0.0;
	};

} /* namespace msl_driver */

#endif /* CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_MOTIONDATA_H_ */
