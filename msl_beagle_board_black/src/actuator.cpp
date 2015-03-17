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

int main(int argc, char** argv) {
	std::cout << "Test" << std::endl;

	// Initialisierungen
	ros::init(argc, argv, "ActuatorController");

	ros::NodeHandle node;
	ros::Rate loop_rate(1);		// 1 Hz

	// ros::Publisher TOPIC_pub = node.advertise<std_msgs::String>("TOPIC", 1000);


	// PINS
	BlackGPIO BH_R_Reset(GPIO_66, output);


	// ADC
	BlackADC adc_light(AIN1);

	BlackLib::BlackGPIO   led1(BlackLib::GPIO_51, BlackLib::output, BlackLib::SecureMode);
	BlackLib::BlackGPIO   led2(BlackLib::GPIO_22, BlackLib::output, BlackLib::FastMode);


	led1.setValue(high);
	led2.setValue(low);


	bool lightbarrier = false;
	bool lightbarrier_old = false;

	while(ros::ok()) {
		// loop_rate legt Frequenz fest


		led1.toggleValue();
		led2.toggleValue();
		std::cout << "LED1: " << led1.getValue() << std::endl;
		std::cout << "LED2: " << led2.getValue() << std::endl;





		BH_R_Reset.toggleValue();
		std::cout << "BH: " << BH_R_Reset.getValue() << std::endl;

		usleep(1000000);


		ROS_INFO("ADC: %s", adc_light.getValue().c_str());
		ROS_INFO("ADC: %s", BH_R_Reset.getValue().c_str());

		//TOPIC_pub.publish(msg);
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
