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
#include "MotionData.h"
#include <chrono>
#include <math.h>
#include "AccelCompensation.h"
#include "CircleTrace.h"

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
		static void pmSigTermHandler(int sig);
		static bool running;

	private:
		ros::NodeHandle* rosNode;
		ros::AsyncSpinner* spinner;
		ros::Subscriber handleMotionControlSub;
		ros::Publisher rawOdometryInfoPub;
		ros::Publisher motionStatInfoPub;


		thread* mainThread;

	protected:
		MotionSet* motionValue = nullptr;
		MotionSet* motionResult = nullptr;
		AccelCompensation* accelComp = nullptr;
		CircleTrace* traceModel = nullptr;

		double slipControlFactor = 1.0;
		double slipControlMinSpeed = 1250.0;
		double slipControlDiffAngle = ((M_PI / 180.0) * 10.0);
		double slipControlDiffAngleMinSpeed = 400.0;
		double slipControlOldMaxRot = (M_PI / 20.0);
		double slipControlNewMinRot = (M_PI / 2.0);
		double accelCompMaxAccel = 4000.0;
		double accelCompMaxAngularAccel = M_PI / 2.0;

		bool accelCompEnabled = false;
		bool slipControlEnabled = false;
		bool sendRosCompliant;

		int driverAlivePeriod = 250;
		int driverOpenAttemptPeriod = 1000;
		int ownId;
		int odometryDelay = 0;
	};

} /* namespace msl_driver */

#endif /* MSL_DRIVER_MOTION_H_ */
