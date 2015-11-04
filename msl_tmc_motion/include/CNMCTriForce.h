/*
 * CNMCTriForce.h
 *
 *  Created on: Aug 26, 2015
 *      Author: cnpaul
 */

#ifndef CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_CNMCTRIFORCE_H_
#define CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_CNMCTRIFORCE_H_


#include <SystemConfig.h>
#include <termios.h>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <time.h>   // time calls
#include <CNMCPacketRequest.h>
#include <DriverData.h>
#include <MotionData.h>
#include <math.h>
#include <mutex>
#include <vector>
#include <typeinfo>

namespace msl_driver
{
//	template<class T>
	class CNMCTriForce
	{
	public:
		CNMCTriForce();
		virtual ~CNMCTriForce();
//		void request(T data);
		enum StatusCode : uint {
			Undefined = 0x00000000,

			Ok = 0x00000001,
			Error = 0x80000001,

			Initialize = 0x00000002,
			ErrorInitialize = 0x80000002,

			Open = 0x00000003,
			ErrorOpen = 0x8000003,

			Shutdown = 0x00000004,
			ErrorShutdown = 0x80000004,

			ExecutingRequest = 0x00000005,
			ErrorExecutingRequest = 0x80000005,

			ExecutingCheck = 0x00000006,
			ErrorExecutingCheck = 0x80000006,
		};
		void notifyStatusChange(StatusCode code, string message);
		long openAttemptPeriod = 1000;	// Time between subsequent calls to Open()
		bool connected();
		long alivePeriod = 250;
	private:
		long lastOpenAttempt = 0;	// Caches the time of the last call to Open()
		string lastErrorMessage = "";	// Caches the last error message (if any)
		string defaultStatusMessage = "";
		string driverName = "";
		string driverVersion = "";
		int stepCounter = 0;


	protected:
		struct MotorConfig
		{
			int resolution;
			int maxSpeed;
			int nomSpeed;
			//public int delta;
			int denominator;
			int numerator;
			int windingTime;
			int maxCurrent;
			int nomCurrent;
			int chassisTime;

			int limitedCurrent;
			int wheelRadius;
			int gearReduction;
			double vmax;

			double pidKp;
			double pidB;
			double pidKi;
			double pidKd;
			double pidKdi;
			double linFactor;
			double smoothFactor;
			int maxErrorInt;
			double rotationErrorWeight;
			double rotationErrorByVeloWeight;
			double rotationErrorByAccelWeight;

			bool controlCurrent;
			int currentErrorBound;
			double currentKp;
			double currentKi;
			double currentKd;

			int deadBand;

			double maxAcceleration;
			double maxDecceleration;
			double maxRotForce;

			double rotationAccelBound;

			int failSafeRPMBound;
			int failSafePWMBound;
			int failSafeCycles;
		};

		MotorConfig* mc;
		int initReadTimeout = 0;		// Initial read timeout (requred to read the garbage provided by the VMC after power on
		int readTimeout = 0;			// General read timeout
		int readTimeoutCount = 0;		// Global counter for read timeouts
		int writeTimeout = 0;			// General write timeout
		int writeTimeoutCount = 0;		// Global counter fo write timeouts
		int port = 0;

		supplementary::SystemConfig* sc = nullptr;

		bool useSerial = true;

		string device = "";				// The name of the device
		string listen = "";

		long minCycleTime = 10000;		// Minimum cycle time (us)
		long cycleLastTimestamp = 0; 	// Timestamp of the last cycle
		long deltaTime = 200000;		// Time used for a cycle (including sleeptime)

		bool quit = false;				// If set to true, the instance terminates
		bool active = false;			// Indicates whether the driver is active
		long lastPurged = 0;			// Timestamp of last purge operation (data management map)

		bool waitForRequest = true;

		double radius = 0.0;
		double maxVelocity = 1000.0;
		bool logOdometry = false;
		string logTypes[];
		void getMotorConfig();

		void initializeInternal();
		bool initialize();
		void openInternal();
		void shutdownInternal();
		void updateMotorState(DriverData request, CNMCPacketRequestResponse* vmcp);
		void addResult(DriverData data);
		void executeCheck();
		struct termios newtio;
		std::mutex resultMutex;
		void signalResult(DriverData data);

		void sendData(CNMCPacket* packet);
		CNMCPacket readData();
		CNMCPacketRequest* p;
		//Queue of requests

		vector<DriverData> results;


	};

} /* namespace msl_driver */

#endif /* CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_CNMCTRIFORCE_H_ */
