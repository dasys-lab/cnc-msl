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
#include <msl_actuator_msgs/RawOdometryInfo.h>
#include "msl_actuator_msgs/MotionControl.h"
#include "msl_actuator_msgs/MotionStatInfo.h"
#include "msl_msgs/MotionInfo.h"

#include "MotionData.h"
#include "CircleTrace.h"
#include "MotorConfig.h"
#include "CNMCPacket.h"

#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include <SystemConfig.h>
#include <Configuration.h>

// serial port stuff
//#include <termios.h>
#include <fcntl.h>
#include "serial/serial.h"

using namespace std;

namespace msl_driver
{

	class Motion
	{
	public:
		Motion(int argc, char** argv);
		virtual ~Motion();

		void initCommunication(int argc, char** argv);
		void initialize();
		void logging_goalie_init();
		void log_goalie();
		bool open();
		void start();
		void handleMotionControl(msl_actuator_msgs::MotionControlPtr mc);bool isRunning();
		void calcOdoPosition();

		static void pmSigintHandler(int sig);
		static void pmSigTermHandler(int sig);

		std::mutex motionValueMutex;
		MotionSet* motionValue = nullptr;
//		CircleTrace traceModel;

		double slipControlFactor = 1.0;
		double slipControlMinSpeed = 1250.0;
		double slipControlDiffAngle = ((M_PI / 180.0) * 10.0);
		double slipControlDiffAngleMinSpeed = 400.0;
		double slipControlOldMaxRot = (M_PI / 20.0);
		double slipControlNewMinRot = (M_PI / 2.0);

		bool slipControlEnabled = false;bool quit = false;

		int driverAlivePeriod = 250;
		int driverOpenAttemptPeriod = 1000;
		int ownId;
		int odometryDelay = 0;
		int isLogging;

		std::string logFile;
		FILE *lp;

		// ROS STUFF
		ros::NodeHandle* rosNode;
		ros::AsyncSpinner* spinner;
		ros::Subscriber handleMotionControlSub;
		ros::Publisher rawOdometryInfoPub;
		ros::Publisher motionStatInfoPub;
		msl_actuator_msgs::RawOdometryInfo rawOdoInfo;
		msl_actuator_msgs::RawOdometryInfo lastOdoInfo;

		static bool running;
		thread runThread;

		// SERIAL PORT STUFF
//		struct termios newtio;
//		int port = 0;
		string device = "";
		int initReadTimeout = 0; // Initial read timeout (required to read the garbage provided by the VMC after power on
		int readTimeout = 0; // General read timeout
		int readTimeoutCount = 0; // Global counter for read timeouts
		int writeTimeout = 0; // General write timeout
		int writeTimeoutCount = 0; // Global counter for write timeouts

		serial::Serial* my_serial;

		supplementary::SystemConfig* sc;
		MotorConfig mc;

		long minCycleTime; // Minimum cycle time in milliseconds
		chrono::steady_clock::time_point cycleLastTimestamp;
		chrono::steady_clock::time_point deltaTime;
		double radius;
		double maxVelocity;
		bool logOdometry;
		shared_ptr<vector<string>> logTypes;
		shared_ptr<vector<string>> logTypesAvailable;bool controllerIsActive = false;

		void run();
		void getMotorConfig();
		void sendMotorConfig();
		void updateMotorState(DriverData request, CNMCPacketRequestResponse* vmcp);
		void executeRequest(MotionSet* ms);

		void sendData(shared_ptr<CNMCPacket> packet);
		unique_ptr<CNMCPacket> readData();
		void checkSuccess(shared_ptr<CNMCPacket> cmd);
	};

} /* namespace msl_driver */

#endif /* MSL_DRIVER_MOTION_H_ */
