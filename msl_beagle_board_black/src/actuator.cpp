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

void pubyy(const msl_actuator_msgs::HaveBallInfo msg) {
	// BallHandling
	if (msg.haveBall == true) {
		ROS_INFO("Sub ausgeloest: HB-True");
	} else {
		ROS_INFO("Sub ausgeloest: HB-False");
	}

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

	// ROS
		ros::init(argc, argv, "ActuatorController");

		ros::NodeHandle node;
		ros::Rate loop_rate(1);		// 1 Hz

		// ros::Publisher TOPIC_pub = node.advertise<std_msgs::String>("TOPIC", 1000);


		//ros::Subscriber bhcSub = node.subscribe<msl_actuator_msgs::BallHandleCmd>("BallHandleControl", 25, handleBallHandleControl, this);
		//ros::Subscriber sscSub = node.subscribe<msl_actuator_msgs::ShovelSelectCmd>("ShovelSelectControl", 25, handleShovelSelectControl, this);
		//ros::Subscriber mlcSub = node.subscribe<msl_actuator_msgs::MotionLight>("CNActuator/MotionLight", 25, handleMotionLight, this);
		//ros::Subscriber bhcSub = node.subscribe<msl_actuator_msgs::BallHandleCmd>("BallHandleControl", 25, handleBallHandleControl);
		ros::Subscriber pubySub = node.subscribe<msl_actuator_msgs::HaveBallInfo>("HaveBallInfo", 25, pubyy);

		ros::Publisher puby = node.advertise<std_msgs::String>("puby", 1000);
		// ros::Publisher bsPub = node.advertise<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/BundleStatus", 10);
		// ros::Publisher brtPub = node.advertise<std_msgs::Empty>("CNActuator/BundleRestartTrigger", 10);
		// ros::Publisher vrtPub = node.advertise<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/VisionRelocTrigger", 10);
		// ros::Publisher mbcPub = node.advertise<msl_actuator_msgs::MotionBurst>("CNActuator/MotionBurst", 10);
		ros::Publisher hbiPub = node.advertise<msl_actuator_msgs::HaveBallInfo>("HaveBallInfo", 10);
		// ros::Publisher imuPub = node.advertise<YYeigene msg bauenYY>("IMU", 10);



	// PINS
		BlackGPIO BH_R_Reset(GPIO_66, output, FastMode);	// P8 07
		BlackGPIO BH_R_Dir(GPIO_67, output, FastMode);		// P8 08
		BlackGPIO BH_R_FF2(GPIO_68, input, FastMode);		// P8 10
		BlackGPIO BH_R_FF1(GPIO_69, input, FastMode);		// P8 09

		BlackGPIO BH_L_Reset(GPIO_45, output, FastMode);	// P8 11
		BlackGPIO BH_L_Dir(GPIO_44, output, FastMode);		// P8 12
		BlackGPIO BH_L_FF2(GPIO_46, input, FastMode);		// P8 16
		BlackGPIO BH_L_FF1(GPIO_47, input, FastMode);		// P8 15

		BlackGPIO i_magnet(GPIO_26, input, FastMode);		// P8 14
		BlackGPIO i_accel(GPIO_27, input, FastMode);		// P8 17
		BlackGPIO i_temp(GPIO_65, input, FastMode);			// P8 18
		BlackGPIO i_gyro(GPIO_61, input, FastMode);			// P8 26

		BlackGPIO LED_Vision(GPIO_48, output, FastMode);	// P9 15
		BlackGPIO LED_Bundle(GPIO_49, output, FastMode);	// P9 23
		BlackGPIO LED_Power(GPIO_20, output, FastMode);		// P9 41		// Oder GPIO_116

		BlackGPIO SW_Vision(GPIO_30, input, FastMode);		// P8 07
		BlackGPIO SW_Bundle(GPIO_31, input, FastMode);		// P8 07

		BlackGPIO OF_NPD(GPIO_117, output, FastMode);		// P8 07
		// BlackGPIO OF_RST(GPIO_115, output, FastMode);	// P8 07
		// BlackGPIO OF_NCS(GPIO_112, output, FastMode);	// P8 07


	// ADC
		BlackADC adc_light(AIN1);

	// I2C
		BlackI2C myI2C(I2C_2, 0x22);		// I2C_2 hinzufuegen und aktivieren
		myI2C.open(ReadWrite | NonBlock);	// Gyro: 0x69, Accel: 0x53, Magnet: 0x1E, Thermo: 0x77

	// SPI
		BlackSPI mySpi(SPI0_0, 8, SpiDefault, 200000);


	// PWM
		BlackPWM BH_L_PWM(P8_19);
		BlackPWM BH_R_PWM(P8_13);
		BlackPWM Servo_PWM(P9_14);
			Servo_PWM.setPeriodTime(20000);		// 20ms Periodendauer
			Servo_PWM.setSpaceRatioTime(1000);		// Werte zwischen 1ms und 2ms



	Servo_PWM.setRunState(run);



	bool lightbarrier_old = false;
	uint16_t adc_test[] = { 30001, 30001, 30001,
						  29999, 29999, 29999,
						  30001, 29999, 30001,
						  29999, 30001, 29999};

	uint16_t count = 0;
	// Frequency set with loop_rate()
	while(ros::ok()) {

		if ( adc_test[count%12] /*adc_light.getNumericValue()*/ > LIGHTBARRIER_THRESHOLD) {
			// something in lightbarrier
			if (lightbarrier_old != true) {			// something NEW
				msl_actuator_msgs::HaveBallInfo msg;

				lightbarrier_old = true;
				msg.haveBall = true;

				//hbiPub.publish(msg);
				ROS_INFO("HaveBall: True");
			}
		} else {
			// nothing in lightbarrier
			if (lightbarrier_old != false) {		// nothing NEW
				msl_actuator_msgs::HaveBallInfo msg;

				lightbarrier_old = false;
				msg.haveBall = false;

				//hbiPub.publish(msg);
				ROS_INFO("HaveBall: False");
			}
		}

		count++;


		//std_msgs::String msg;
		//std::stringstream ss;
		//ss << "hello world " << count;
		//msg.data = ss.str();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
