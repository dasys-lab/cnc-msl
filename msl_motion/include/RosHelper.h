#ifndef RosHelper_h
#define RosHelper_h

#include <stdio.h>
#include <sys/time.h>
#include "ros/ros.h"
#include "msl_actuator_msgs/RawOdometryInfo.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "msl_msgs/MotionInfo.h"
//#include "msl_msgs/PositionInfo.h"

class RosHelper {
	public:
		static void initialize(int argc, char** argv);
		static msl_actuator_msgs::RawOdometryInfo rawOdo;
		static ros::Publisher rawOdo_pub;
		static ros::Subscriber motionControl_sub;

		static void handleMotionControlMessage(const msl_actuator_msgs::MotionControl::ConstPtr& message);
		static void sendOdometry();
		static msl_msgs::MotionInfo* getMotion();
		static void sendError(std::string msg);
		static void sendWarning(std::string msg);
		static void sendInfo(std::string msg);
	protected:
		static msl_msgs::MotionInfo currentCommand;
		static struct timeval last_cmd_received;
		static ros::AsyncSpinner *spinner;
};



#endif
