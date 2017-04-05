#pragma once

#include "msl_actuator_msgs/MotionControl.h"
#include "msl_robot/dribbleCalibration/container/DribbleCalibrationQuery.h"
namespace msl
{
	class ICalibration
	{
	public:
		virtual ~ICalibration(){}

		std::string redBegin = "\033[1;31m";
		std::string redEnd = "\033[0m\n";

		/**
		 * will be called if the robot is able to calibrate
		 */
		virtual std::shared_ptr<DribbleCalibrationQuery> move(int trans) = 0;

		/**
		 * will be called if the robot looses the ball
		 */
		virtual void adaptParams() = 0;

		virtual void resetParams() = 0;

		/**
		 * will be called if the robot could hold the ball long enough
		 */
		virtual void saveParams() = 0;

		/**
		 * will be called after the parameters has changed or the behaviour finished
		 */
		virtual void writeConfigParameters() = 0;

		/**
		 * will be called in the initialize method of the DribbleCalibration behaviour
		 */
		virtual void readConfigParameters() = 0;
	};
}
