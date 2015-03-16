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

using namespace BlackLib;

int main(int argc, char** argv) {
	std::cout << "Test" << std::endl;

	// Initialisierungen
	ros::init(argc, argv, "ActuatorController");

	ros::NodeHandle node;
	ros::Rate loop_rate(1);		// 1 Hz

	ros::Publisher TOPIC_pub = node.advertise<std_msgs::String>("TOPIC", 1000);





	// PINS
	BlackGPIO BH_R_Reset(GPIO_66, output);


	// ADC
	BlackADC adc_light(AIN1);


	bool lightbarrier = false;
	bool lightbarrier_old = false;

	while(ros::ok()) {
		// loop_rate legt Frequenz fest

		// ADC
		if (adc_light.getNumericValue() > LIGHTBARRIER_THRESHOLD) {
			lightbarrier = true;			// Etwas in Lichtschranke
		} else {
			lightbarrier = false;
		}

		if (lightbarrier != lightbarrier_old) {
			msl_actuator_msgs::HaveBallInfo info;
			info.haveBall = lightbarrier;

			//hbiPub.publish(info);
		}


		std_msgs::String msg;

		std::stringstream ss;
		ss << "Test";
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		//TOPIC_pub.publish(msg);
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
