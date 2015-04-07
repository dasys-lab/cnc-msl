/*
 * main.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Lukas Will
 */


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

// Threads
#include <thread>
#include <mutex>
#include <condition_variable>

//eigene
#include "config.h"
#include "ballhandle.h"

// fuer Tests
//#include <thread>         // std::this_thread::sleep_for
//#include <chrono>         // std::chrono::seconds
//#include <stdio.h>		// File Open
//#include <unistd.h>		// File Open

using namespace BlackLib;


std::mutex m;
std::condition_variable cv;

BallHandle		BH_right(P8_13, GPIO_67, GPIO_66, GPIO_69, GPIO_68);
BallHandle		BH_left(P8_19, GPIO_44, GPIO_45, GPIO_47, GPIO_46);
BlackPWM		ShovelSelect(P9_14);
uint16_t		LightBarrier;

timeval			time_now;
timeval			last_ping;
timeval			ShovelSelect_lastSet;




void pubyy(const msl_actuator_msgs::HaveBallInfo msg) {
	// BallHandling
	if (msg.haveBall == true) {
		ROS_INFO("Sub ausgeloest: HB-True");
	} else {
		ROS_INFO("Sub ausgeloest: HB-False");
	}

	// Nachricht verarbeiten
}

void handleBallHandleControl(const msl_actuator_msgs::BallHandleCmd msg) {
	// BallHandling
	/*if (msg.enabled) {
		BH_right.setBallHandling(msg.rightMotor);
		BH_left.setBallHandling(msg.leftMotor);
	} else {
		BH_right.setBallHandling(0);
		BH_left.setBallHandling(0);
	}*/
}

void handleShovelSelectControl(const msl_actuator_msgs::ShovelSelectCmd msg) {
	// Schussauswahl (ggf Wert fuer Servoposition mit uebergeben lassen)
	if (msg.passing) {
		ShovelSelect.setSpaceRatioTime(ShovelSelect_PASSING);
	} else {
		ShovelSelect.setSpaceRatioTime(ShovelSelect_NORMAL);
	}
	ShovelSelect.setRunState(run);
}

void handleMotionLight(const msl_actuator_msgs::MotionLight msg) {
	// LED vom Maussensor ansteuern
}





int main(int argc, char** argv) {
	std::cout << "Test" << std::endl;



	// ROS
		ros::init(argc, argv, "ActuatorController");

		ros::NodeHandle node;
		ros::Rate loop_rate(1);		// 20 Hz

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

/*
	// Initialisierungen

	// BallHandle Init
		// SET PWM

	// ShovelSelect Init
		ShovelSelect.setPeriodTime(20000);			// in us - 20ms Periodendauer
		ShovelSelect.setSpaceRatioTime(1000);		// in us - Werte zwischen 1ms und 2ms



	// PINS


		BlackGPIO i_magnet(GPIO_26, input, FastMode);		// P8 14
		BlackGPIO i_accel(GPIO_27, input, FastMode);		// P8 17
		BlackGPIO i_temp(GPIO_65, input, FastMode);			// P8 18
		BlackGPIO i_gyro(GPIO_61, input, FastMode);			// P8 26

		BlackGPIO LED_Vision(GPIO_48, output, FastMode);	// P9 15
		BlackGPIO LED_Bundle(GPIO_49, output, FastMode);	// P9 23
		BlackGPIO LED_Power(GPIO_20, output, FastMode);		// P9 41		// Oder GPIO_116

		BlackGPIO SW_Vision(GPIO_30, input, FastMode);		// P8 07
		BlackGPIO SW_Bundle(GPIO_31, input, FastMode);		// P8 07


*/

	// ADC
		BlackADC lightbarrier(AIN1);

	// I2C
		BlackI2C myI2C(I2C_2, 0x22);		// I2C_2 hinzufuegen und aktivieren
		myI2C.open(ReadWrite | NonBlock);	// Gyro: 0x69, Accel: 0x53, Magnet: 0x1E, Thermo: 0x77

	// SPI
		BlackSPI mySpi(SPI0_0, 8, SpiDefault, 200000);



	bool lightbarrier_old = false;
	bool vision_old = false;
	bool bundle_old = false;


	uint16_t count = 0;

	BlackGPIO test66(GPIO_66, output, FastMode);

	// Frequency set with loop_rate()
	while(ros::ok()) {
		gettimeofday(&time_now, NULL);



		if ((lightbarrier.getNumericValue() > LIGHTBARRIER_THRESHOLD) /*&& ( alle 10 ms )*/) {
			// something in lightbarrier NEW
			msl_actuator_msgs::HaveBallInfo msg;

			lightbarrier_old = true;
			msg.haveBall = true;

			// hbiPub.publish(msg);
			// ROS_INFO("HaveBall: True");
		} else if ((lightbarrier.getNumericValue() < LIGHTBARRIER_THRESHOLD) && (lightbarrier_old != false)) {
			// something out of lightbarrier NEW
			msl_actuator_msgs::HaveBallInfo msg;

			lightbarrier_old = false;
			msg.haveBall = false;

			// hbiPub.publish(msg);
			// ROS_INFO("HaveBall: False");
		} else {
			// nothing NEW

		}
/*
		if ((SW_Vision.getNumericValue() == 1) && (vision_old != true)) {
			// Vision pressed

		} else if ((SW_Vision.getNumericValue() == 0) && (vision_old != false)) {
			// Vision released
		} else {
			// nothing NEW
		}

		if ((SW_Bundle.getNumericValue() == 1) && (bundle_old != true)) {
			// Bundle pressed

		} else if ((SW_Vision.getNumericValue() == 0) && (bundle_old != false)) {
			// Bundle released
		} else {
			// nothing NEW
		}
*/
		// MotionBurst

		// IMU

		timeval vorher, nachher;
		uint16_t value;

		bool set, get;

		set = count%2;

		gettimeofday(&vorher, NULL);
		bool working = test66.setValue(static_cast<digitalValue>(set));
		gettimeofday(&nachher, NULL);

		long int diffus = TIMEDIFFUS(nachher, vorher);
		long int diffms = TIMEDIFFMS(nachher, vorher);

		std::cout << "SET: " << set << " - " << working << std::endl;



		gettimeofday(&vorher, NULL);
		get = static_cast<bool>(test66.getNumericValue());
		gettimeofday(&nachher, NULL);

		diffus = TIMEDIFFUS(nachher, vorher);
		diffms = TIMEDIFFMS(nachher, vorher);

		std::cout << "GET: " << get << " - Zeit: " << diffms << " - " << diffus << std::endl;


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
