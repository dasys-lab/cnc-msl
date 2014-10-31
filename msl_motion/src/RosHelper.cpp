#include "RosHelper.h"
#include "gonzales.h" 
#include <SystemConfig.h>
#include <iostream>
using namespace msl_msgs;
using namespace msl_actuator_msgs;

MotionInfo RosHelper::currentCommand;
RawOdometryInfo RosHelper::rawOdo;
struct timeval RosHelper::last_cmd_received;
ros::Publisher RosHelper::rawOdo_pub;
ros::Subscriber RosHelper::motionControl_sub;
ros::AsyncSpinner *RosHelper::spinner;


void RosHelper::initialize(int argc, char** argv) { 
		
	ros::init(argc, argv,supplementary::SystemConfig::robotNodeName("Gonzales"));

	ros::NodeHandle motionCEP;
	
	RosHelper::rawOdo_pub = motionCEP.advertise<RawOdometryInfo>("RawOdometry", 1);
	
	RosHelper::motionControl_sub = motionCEP.subscribe<MotionControl>("MotionControl", 1, handleMotionControlMessage);
	
	gettimeofday(&last_cmd_received,NULL);
	
	spinner = new ros::AsyncSpinner(1);
	spinner->start();
}

MotionInfo* RosHelper::getMotion() {
	static struct timeval cur_time;
	gettimeofday(&cur_time,NULL);
	if (TIMEDIFFMS(cur_time,last_cmd_received) > current_settings.commandTimeout) {
        //std::cout << "Time diff to large: last:" << last_cmd_received.tv_sec << " " << last_cmd_received.tv_usec << " cur: " << cur_time.tv_sec << " " << cur_time.tv_usec << std::endl;
		return NULL;
	}
	return &currentCommand;
}

void RosHelper::handleMotionControlMessage(const MotionControl::ConstPtr& message){
	currentCommand = (message->motion);
	//std::cout<< message->motion.angle<< std::endl;
	//std::cout<< message->motion.rotation<< std::endl;
	//std::cout<< message->motion.translation<< std::endl;

	gettimeofday(&last_cmd_received,NULL);
//std::cout << "GotMotionMsg: set time to: " << last_cmd_received.tv_sec << " " << last_cmd_received.tv_usec << std::endl;
	return;
}

void RosHelper::sendOdometry() {
	rawOdo_pub.publish(rawOdo);
}

