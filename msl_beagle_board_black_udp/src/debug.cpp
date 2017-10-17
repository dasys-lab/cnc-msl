/*
 * debug.cpp
 *
 *  Created on: Feb 24, 2016
 *      Author: Lukas Will
 */


#include <iostream>
#include <unistd.h>
#include <string>

using namespace std;

// ROS
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/MotionLight.h"
#include "msl_actuator_msgs/MotionBurst.h"
#include "msl_actuator_msgs/ShovelSelectCmd.h"
#include "msl_actuator_msgs/VisionRelocTrigger.h"
#include "msl_actuator_msgs/IMUData.h"
#include "process_manager/ProcessCommand.h"


bool button[2] = {false, false};
bool mb, hbi, imu = false;


void handlePC(const process_manager::ProcessCommand msg) {
	button[0] = true;
}

void handleVRT(const msl_actuator_msgs::VisionRelocTrigger msg) {
	button[1] = true;
}

void handleMB(const msl_actuator_msgs::MotionBurst msg) {
	mb = true;
}

void handleHBI(const std_msgs::Bool msg) {
	hbi = true;
}

void handleIMU(const msl_actuator_msgs::IMUData msg) {
	imu = true;
}


int main(int argc, char** argv) {
	// ROS Init
	ros::init(argc, argv, "ActuatorDebug");
	ros::NodeHandle node;
	ros::Rate loop_rate(1);		// in Hz

	ros::Subscriber brtSub = node.subscribe<process_manager::ProcessCommand>("ProcessCommand", 25, handlePC);
	ros::Subscriber vrtSub = node.subscribe<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/VisionRelocTrigger", 25, handleVRT);
	ros::Subscriber mbcSub = node.subscribe<msl_actuator_msgs::MotionBurst>("CNActuator/MotionBurst", 25, handleMB);
	ros::Subscriber hbiSub = node.subscribe<std_msgs::Bool>("LightBarrierInfo", 25, handleHBI);
	ros::Subscriber imuSub = node.subscribe<msl_actuator_msgs::IMUData>("/IMUData", 25, handleIMU);

	ros::Publisher sscPub = node.advertise<msl_actuator_msgs::ShovelSelectCmd>("ShovelSelectControl", 10);
	ros::Publisher mlcPub = node.advertise<msl_actuator_msgs::MotionLight>("CNActuator/MotionLight", 10);
	ros::Publisher bhcPub = node.advertise<msl_actuator_msgs::BallHandleCmd>("BallHandleControl", 10);

	msl_actuator_msgs::ShovelSelectCmd msg_shovelselect;
	msl_actuator_msgs::BallHandleCmd msg_ballhandle;
	msl_actuator_msgs::MotionLight msg_motionlight;

	usleep(1000000);

	cout << "ShovelSelect: Low Kick (Passing)" << endl;
	msg_shovelselect.passing = true;
	sscPub.publish(msg_shovelselect);
	usleep(1000000);

	cout << "ShovelSelect: High Kick" << endl;
	msg_shovelselect.passing = false;
	sscPub.publish(msg_shovelselect);
	usleep(1000000);


	msg_ballhandle.enabled = true;
	msg_ballhandle.leftMotor = 0;

	cout << "right Motor: Turning right" << endl;
	msg_ballhandle.rightMotor = 50;
	bhcPub.publish(msg_ballhandle);
	usleep(1000000);

	cout << "right Motor: Turning left" << endl;
	msg_ballhandle.rightMotor = -50;
	bhcPub.publish(msg_ballhandle);
	usleep(1000000);

	msg_ballhandle.rightMotor = 0;

	cout << "left Motor: Turning right" << endl;
	msg_ballhandle.leftMotor = 50;
	bhcPub.publish(msg_ballhandle);
	usleep(1000000);

	cout << "left Motor: Turning left" << endl;
	msg_ballhandle.leftMotor = -50;
	bhcPub.publish(msg_ballhandle);
	usleep(1000000);


	cout << "Press the Buttons!" << endl;

	bool testing = true;
	while(ros::ok() && testing) {
		cout << "Bundle:   " << button[0] << endl;
		cout << "Vision:   " << button[1] << endl;
		cout << "Motion:   " << mb << endl;
		cout << "HaveBall: " << hbi << endl;
		cout << "IMU:      " << imu << endl;

		if(button[0] && button[1] && mb && hbi && imu) {
			testing = false;
		}
	}

	cout << "Nachrichten empfangen." << endl;
	cout << "Aktoren erfolgreich angesteuert? (y or n)" << endl;

	string str;
	cin >> str;

	if(str == "y") {
		cout << "Tests erfolgreich!" << endl;
	} else if(str == "n") {
		cout << "Nicht erfolgreich!" << endl;
	} else {
		cout << "Eingabe nicht erkannt." << endl;
	}
	cout << "Programm ende." << endl;

	return 0;
}
