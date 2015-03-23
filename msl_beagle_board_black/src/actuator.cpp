/*
 * main.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Lukas Will
 */

#include <iostream>
#include <sstream>

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"

// ROS - Messages
#include "msl_actuator_msgs/BallCatchCmd.h"
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/ShovelSelectCmd.h"
#include "msl_actuator_msgs/HaveBallInfo.h"
#include "msl_actuator_msgs/VisionRelocTrigger.h"
#include "msl_actuator_msgs/MotionLight.h"

// BlackLibs
#include "BlackADC.h"
#include "BlackGPIO.h"
#include "BlackI2C.h"
#include "BlackPWM.h"
#include "BlackSPI.h"

//eigene
#include "config.h"

// fuer Tests
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <stdio.h>		// File Open
#include <unistd.h>		// File Open

using namespace BlackLib;

void pubyy(const std_msgs::String msg) {
	// BallHandling
	ROS_INFO("Sub ausgeloest: %s", msg.data.c_str());
	// Nachricht verarbeiten
}

void handleBallHandleControl(const msl_actuator_msgs::BallHandleCmd) {
	// BallHandling

	// Nachricht verarbeiten
}

void handleShovelSelectControl(const msl_actuator_msgs::ShovelSelectCmd) {
	// Schussauswahl

	//
}

void handleMotionLight(const msl_actuator_msgs::MotionLight) {
	// LED vom Motionsensor
}

int main(int argc, char** argv) {
	std::cout << "Test" << std::endl;

	// Initialisierungen
	ros::init(argc, argv, "ActuatorController");

	ros::NodeHandle node;
	ros::Rate loop_rate(1);		// 1 Hz

	// ros::Publisher TOPIC_pub = node.advertise<std_msgs::String>("TOPIC", 1000);


	//ros::Subscriber bhcSub = node.subscribe<msl_actuator_msgs::BallHandleCmd>("BallHandleControl", 25, handleBallHandleControl, this);
	//ros::Subscriber sscSub = node.subscribe<msl_actuator_msgs::ShovelSelectCmd>("ShovelSelectControl", 25, handleShovelSelectControl, this);
	//ros::Subscriber mlcSub = node.subscribe<msl_actuator_msgs::MotionLight>("CNActuator/MotionLight", 25, handleMotionLight, this);
	//ros::Subscriber bhcSub = node.subscribe<msl_actuator_msgs::BallHandleCmd>("BallHandleControl", 25, handleBallHandleControl);
	ros::Subscriber pubySub = node.subscribe<std_msgs::String>("puby", 25, pubyy);

	ros::Publisher puby = node.advertise<std_msgs::String>("puby", 1000);
	// ros::Publisher bsPub = node.advertise<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/BundleStatus", 10);
	// ros::Publisher brtPub = node.advertise<std_msgs::Empty>("CNActuator/BundleRestartTrigger", 10);
	// ros::Publisher vrtPub = node.advertise<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/VisionRelocTrigger", 10);
	// ros::Publisher mbcPub = node.advertise<msl_actuator_msgs::MotionBurst>("CNActuator/MotionBurst", 10);
	// ros::Publisher hbiPub = node.advertise<msl_actuator_msgs::HaveBallInfo>("HaveBallInfo", 10);
	// ros::Publisher imuPub = node.advertise<YYeigene msg bauenYY>("IMU", 10);


	// PINS
	BlackGPIO BH_R_Reset(GPIO_66, output, FastMode);


	// ADC
	BlackADC adc_light(AIN1);

	// I2C
	BlackI2C myI2C(I2C_1, 0x22);		// I2C_2 hinzufuegen und aktivieren
	myI2C.open(ReadWrite | NonBlock);	// Gyro: 0x69, Accel: 0x53, Magnet: 0x1E, Thermo: 0x77

	// SPI
	BlackSPI mySpi(SPI0_0, 8, SpiDefault, 200000);


	// PWM
	BlackPWM Servo_PWM(P9_14);
	Servo_PWM.setPeriodTime(20, milisecond);		// 20ms Periodendauer
	Servo_PWM.setSpaceRatioTime(1, milisecond);		// Werte zwischen 1ms und 2ms



	Servo_PWM.setRunState(run);



	bool lightbarrier = false;
	bool lightbarrier_old = false;
	int count = 0;
	while(ros::ok()) {
		// loop_rate legt Frequenz fest
			count++;


		BH_R_Reset.toggleValue();

		Servo_PWM.setSpaceRatioTime(count%20 , milisecond); // Werte von 1 bis 2 */
		ROS_INFO("ADC: %s", adc_light.getValue().c_str());
		ROS_INFO("BH_Reset: %s", BH_R_Reset.getValue().c_str());
		ROS_INFO("PWM Change: %s", Servo_PWM.getValue().c_str());

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("Gesendet: %s", msg.data.c_str());
		puby.publish(msg);


		//TOPIC_pub.publish(msg);
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
