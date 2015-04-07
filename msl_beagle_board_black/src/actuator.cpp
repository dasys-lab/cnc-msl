/*
 * main.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Lukas Will
 */


#include "actuator.h"

// fuer Tests
//#include <thread>         // std::this_thread::sleep_for
//#include <chrono>         // std::chrono::seconds
//#include <stdio.h>		// File Open
//#include <unistd.h>		// File Open

using namespace BlackLib;


std::mutex mtx_light;
std::condition_variable cv;




void handleBallHandleControl(const msl_actuator_msgs::BallHandleCmd msg) {
	gettimeofday(&last_ping, NULL);

	if (msg.enabled) {
		BH_right.setBallHandling(msg.rightMotor);
		BH_left.setBallHandling(msg.leftMotor);
	} else {
		BH_right.setBallHandling(0);
		BH_left.setBallHandling(0);
	}
}

void handleShovelSelectControl(const msl_actuator_msgs::ShovelSelectCmd msg) {
	gettimeofday(&last_ping, NULL);
	shovel.last_ping = last_ping;

	// Schussauswahl (ggf Wert fuer Servoposition mit uebergeben lassen)
	if (msg.passing) {
		shovel.value = ShovelSelect_PASSING;
	} else {
		shovel.value = ShovelSelect_NORMAL;
	}
	shovel.enabled;
}

void handleMotionLight(const msl_actuator_msgs::MotionLight msg) {
	gettimeofday(&last_ping, NULL);

	// LED vom Maussensor ansteuern
}


void controlBHLeft() {
	BH_left.controlBallHandling();
}

void controlBHRight() {
	BH_right.controlBallHandling();
}

void contolShovelSelect() {
	if ((TIMEDIFFMS(time_now, shovel.last_ping) > ShovelSelect_TIMEOUT) && shovel.enabled) {
		shovel.enabled = false;
		ShovelSelect.setRunState(stop);
	}

	if (shovel.enabled) {
		if (ShovelSelect.getRunValue() == "0") {
			ShovelSelect.setRunState(run);
		}
		ShovelSelect.setSpaceRatioTime(shovel.value, microsecond);
	}
}

void getLightbarrier() {
	msl_actuator_msgs::HaveBallInfo msg;
	uint16_t value = ADC_Light.getNumericValue();

	if (lightbarrier > LIGHTBARRIER_THRESHOLD) {		// something in lightbarrier NEW
		msg.haveBall = true;
		hbiPub.publish(msg);
		// ROS_INFO("HaveBall: True");
	} else {											// something out of lightbarrier NEW
		msg.haveBall = false;
		hbiPub.publish(msg);
		// ROS_INFO("HaveBall: False");
	}
}

void getSwitches() {
	msl_actuator_msgs::VisionRelocTrigger msg;
	std_msgs::Empty msg_empty;
	uint8_t bundle, power, vision;

	bundle = SW_Bundle.getNumericValue();
	vision = SW_Vision.getNumericValue();
	power = SW_Power.getNumericValue();

	msg.receiverID = 123;	// ???
	msg.usePose = false;

	if (bundle == 1) {
		bsPub.publish(msg);
		brtPub.publish(msg_empty);
	}

	if (vision == 1) {
		vrtPub.publish(msg);
	}

	if (power == 1) {

	}
}




int main(int argc, char** argv) {
	std::cout << "Test" << std::endl;


	// ROS Init
	ros::init(argc, argv, "ActuatorController");
	ros::Rate loop_rate(1);		// in Hz

	// Shovel Init
	ShovelSelect.setPeriodTime(20000);			// in us - 20ms Periodendauer
	ShovelSelect.setSpaceRatioTime(1000);		// in us - Werte zwischen 1ms und 2ms
	shovel.enabled = false;

	// I2C
	myI2C.open(ReadWrite | NonBlock);			// Gyro: 0x69, Accel: 0x53, Magnet: 0x1E, Thermo: 0x77

	uint16_t count = 0;

	BlackGPIO test66(GPIO_66, output, FastMode);

	// Frequency set with loop_rate()
	while(ros::ok()) {
		gettimeofday(&time_now, NULL);

		std::thread th_controlBHRight(controlBHRight);
		std::thread th_controlBHLeft(controlBHLeft);
		std::thread th_controlShovel(contolShovelSelect);
		std::thread th_lightbarrier(getLightbarrier);
		std::thread th_switches(getSwitches);

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
