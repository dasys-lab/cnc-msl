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

	class MotionSet
	{
	public:
		long timespamp;
		double angle = 0.0;
		double translation = 0.0;
		double rotation = 0.0;
		short pwm[];
	};

	class MotionSlipControl
	{
	public:
		long timespamp;
		bool enabled = false;
	};
	class MotionCurrent
	{
	public:
		long timespamp;
		double current[];
	};
	class MotionTemperature
	{
	public:
		long timespamp;
		double temperature[];
	};
	class MotionSupplyVoltage
	{
	public:
		long timespamp;
		double voltage = 0.0;
	};

} /* namespace msl_driver */

#endif /* CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_MOTIONDATA_H_ */
