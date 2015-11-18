/*
 * CNMCTriForce.h
 *
 *  Created on: Aug 26, 2015
 *      Author: cnpaul
 */

#ifndef CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_CNMCTRIFORCE_H_
#define CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_CNMCTRIFORCE_H_

#include <CNMCPacket.h>
#include <SystemConfig.h>
#include <termios.h>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <time.h>   // time calls
#include <DriverData.h>
#include <MotionData.h>
#include <math.h>
#include <mutex>
#include <vector>
#include <typeinfo>

namespace msl_driver
{
//	template<class T>
	class CNMC
	{
	public:
		CNMC();
		virtual ~CNMC();

		void notifyStatusChange(StatusCode code, string message);
		long openAttemptPeriod = 1000; // Time between subsequent calls to Open()

		long alivePeriod = 250;

	protected:
		supplementary::SystemConfig* sc = nullptr;

		MotorConfig* mc;
		int initReadTimeout = 0; // Initial read timeout (requred to read the garbage provided by the VMC after power on
		int readTimeout = 0; // General read timeout
		int readTimeoutCount = 0; // Global counter for read timeouts
		int writeTimeout = 0; // General write timeout
		int writeTimeoutCount = 0; // Global counter fo write timeouts

		int port = 0;bool useSerial = true;
		string device = ""; // The name of the device

		long minCycleTime = 10000; // Minimum cycle time (us)
		long cycleLastTimestamp = 0; // Timestamp of the last cycle
		long deltaTime = 200000; // Time used for a cycle (including sleeptime)

		bool quit = false; // If set to true, the instance terminates
		bool active = false; // Indicates whether the driver is active
		long lastPurged = 0; // Timestamp of last purge operation (data management map)

		bool waitForRequest = true;

		double radius = 0.0;
		double maxVelocity = 1000.0;
		bool logOdometry = false;
		string logTypes[];
		void getMotorConfig();

		bool initialize();
		void open();
		void shutdownInternal();
		void updateMotorState(DriverData request, CNMCPacketRequestResponse* vmcp);
		void addResult(DriverData data);
		void executeCheck();
		struct termios newtio;
		std::mutex resultMutex;
		void signalResult(DriverData data);

		void sendData(CNMCPacket* packet);
		CNMCPacket readData();
		void run();
		CNMCPacketRequest* p;

		//Queue of requests
		vector<DriverData> results;

		std::thread runThread;

	private:
		long lastOpenAttempt = 0; // Caches the time of the last call to Open()
		string lastErrorMessage = ""; // Caches the last error message (if any)
		string defaultStatusMessage = "";
		string driverName = "";
		string driverVersion = "";
		int stepCounter = 0;
	};

} /* namespace msl_driver */

#endif /* CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_CNMCTRIFORCE_H_ */
