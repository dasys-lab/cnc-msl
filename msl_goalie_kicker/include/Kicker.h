/*
 * Kicker.h
 *
 *  Created on: Aug 19, 2015
 *      Author: Stephan Opfer
 */

#ifndef MSL_DRIVER_MOTION_H_
#define MSL_DRIVER_MOTION_H_

#define TMC_MOTION_DEBUG // for toggling debug output

#include <ros/ros.h>
#include <msl_actuator_msgs/RawOdometryInfo.h>
#include "msl_actuator_msgs/KickControl.h"

#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include <SystemConfig.h>

// serial port stuff
//#include <termios.h>
#include <fcntl.h>
#include "serial/serial.h"

using namespace std;

namespace msl_driver
{

	class Kicker
	{
	public:
		Kicker(int argc, char** argv);
		virtual ~Kicker();

		void initCommunication(int argc, char** argv);
		void initialize();
		bool open();
		void start();
		void handleKickControl(msl_actuator_msgs::KickControlPtr mc);
		bool isRunning();

		static void pmSigintHandler(int sig);
		static void pmSigTermHandler(int sig);

		std::mutex motionValueMutex;

		long pulseWidthLeft;
		long pulseWidthMiddle;
		long pulseWidthRight;

		long extensionMaxTime;
		long extensionMinSleep;

		int driverAlivePeriod = 250;
		int driverOpenAttemptPeriod = 1000;
		int ownId;
		int odometryDelay = 0;

		// ROS STUFF
		ros::NodeHandle* rosNode;
		ros::AsyncSpinner* spinner;
		ros::Subscriber handleKickerControlSub;

		static bool running;
		thread runThread;

		// SERIAL PORT STUFF
		string device = "";
		int initReadTimeout = 0; // Initial read timeout (required to read the garbage provided by the VMC after power on
		int readTimeout = 0; // General read timeout
		int readTimeoutCount = 0; // Global counter for read timeouts
		int writeTimeout = 0; // General write timeout
		int writeTimeoutCount = 0; // Global counter for write timeouts

		serial::Serial* my_serial;

		supplementary::SystemConfig* sc;
//		MotorConfig mc;

		chrono::steady_clock::time_point cycleLastTimestamp;
		chrono::steady_clock::time_point deltaTime;

		bool controllerIsActive = false;

		bool writeAndCheck(string name);
		bool checkSuccess(string name, string command, bool display);
		string readResult(string command);
	};

} /* namespace msl_driver */

#endif /* MSL_DRIVER_MOTION_H_ */
