/*
 * Motion.h
 *
 *  Created on: Aug 19, 2015
 *      Author: Stephan Opfer
 */

#ifndef MSL_DRIVER_MOTION_H_
#define MSL_DRIVER_MOTION_H_

#define TMC_MOTION_DEBUG // for toggling debug output

#include <ros/ros.h>
#include "msl_actuator_msgs/RawOdometryInfo.h"
#include "msl_actuator_msgs/MotionStatInfo.h"
#include <chrono>

using namespace std;

namespace std{
	class thread;
}

namespace msl_driver
{

	class Motion
	{
	public:
		Motion(int argc, char** argv);
		virtual ~Motion();

		void initCommunication(int argc, char** argv);
		void start();
		void run();
		void handleMotionControl(msl_actuator_msgs::MotionStatInfoPtr msi);
		bool isRunning();

		static void pmSigintHandler(int sig);
		static bool running;

	private:
		ros::NodeHandle* rosNode;
		ros::AsyncSpinner* spinner;
		ros::Subscriber handleMotionControlSub;
		ros::Publisher rawOdometryInfoPub;
		ros::Publisher motionStatInfoPub;


		thread* mainThread;
	};

} /* namespace msl_driver */

#endif /* MSL_DRIVER_MOTION_H_ */
