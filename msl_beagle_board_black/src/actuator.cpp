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

std::mutex					mtx;
//std::condition_variable		cv, cv2;

timeval		ls, le, rs, re, ss, se, lis, lim, lie, sws, swe;

uint8_t		th_count;

bool		th_activ = true;




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
	std::unique_lock<std::mutex> l_bhl(threw[0].mtx);
	while(th_activ) {
		threw[0].cv.wait(l_bhl, [&] { return !th_activ || threw[0].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		gettimeofday(&ls, NULL);
		BH_left.controlBallHandling();
		gettimeofday(&le, NULL);

		threw[0].notify = false;
	}
}

void controlBHRight() {
	std::unique_lock<std::mutex> l_bhr(threw[1].mtx);
	while(th_activ) {
		threw[1].cv.wait(l_bhr, [&] { return !th_activ || threw[1].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		gettimeofday(&rs, NULL);
		BH_right.controlBallHandling();
		gettimeofday(&re, NULL);

		threw[1].notify = false;
	}
}

void contolShovelSelect() {
	std::unique_lock<std::mutex> l_shovel(threw[2].mtx);
	while(th_activ) {
		threw[2].cv.wait(l_shovel, [&] { return !th_activ || threw[2].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		gettimeofday(&ss, NULL);
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
		gettimeofday(&se, NULL);

		threw[2].notify = false;
	}
}

void getLightbarrier(ros::Publisher *hbiPub) {
	std::unique_lock<std::mutex> l_light(threw[3].mtx);
	while(th_activ) {
		threw[3].cv.wait(l_light, [&] { return !th_activ || threw[3].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		gettimeofday(&lis, NULL);
		msl_actuator_msgs::HaveBallInfo msg;
		uint16_t value = ADC_Light.getNumericValue();
		gettimeofday(&lim, NULL);


		if (value > LIGHTBARRIER_THRESHOLD) {
			msg.haveBall = true;
			//ROS_INFO("HaveBall: True");
		} else {
			msg.haveBall = false;
			//ROS_INFO("HaveBall: False");
		}
		hbiPub->publish(msg);
		gettimeofday(&lie, NULL);

		threw[3].notify = false;
	}
}

void getSwitches(ros::Publisher *bsPub, ros::Publisher *brtPub, ros::Publisher *vrtPub) {
	std::unique_lock<std::mutex> l_switches(threw[4].mtx);
	while(th_activ) {
		threw[4].cv.wait(l_switches, [&] { return !th_activ || threw[4].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		gettimeofday(&sws, NULL);
		msl_actuator_msgs::VisionRelocTrigger msg;
		std_msgs::Empty msg_empty;
		uint8_t bundle, power, vision;

		bundle = SW_Bundle.getNumericValue();
		vision = SW_Vision.getNumericValue();
		power = SW_Power.getNumericValue();

		msg.receiverID = 123;	// ???
		msg.usePose = false;

		if (bundle == 1) {
			bsPub->publish(msg);
			brtPub->publish(msg_empty);
		}

		if (vision == 1) {
			vrtPub->publish(msg);
		}

		if (power == 1) {

		}
		gettimeofday(&swe, NULL);

		threw[4].notify = false;
	}
}

void exit_program(int sig) {
	ex = true;
	std::cout << "Programm wird beendet." << std::endl;
	th_activ = false;
	for (int i=0; i<5; i++)
		threw[i].cv.notify_all();
}




int main(int argc, char** argv) {
	std::cout << "Test" << std::endl;


	// ROS Init
	ros::init(argc, argv, "ActuatorController");
	ros::NodeHandle node;
	ros::Rate loop_rate(1);		// in Hz

	ros::Subscriber sscSub = node.subscribe<msl_actuator_msgs::ShovelSelectCmd>("ShovelSelectControl", 25, handleShovelSelectControl);
	ros::Subscriber mlcSub = node.subscribe<msl_actuator_msgs::MotionLight>("CNActuator/MotionLight", 25, handleMotionLight);
	ros::Subscriber bhcSub = node.subscribe<msl_actuator_msgs::BallHandleCmd>("BallHandleControl", 25, handleBallHandleControl);

	ros::Publisher bsPub = node.advertise<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/BundleStatus", 10);
	ros::Publisher brtPub = node.advertise<std_msgs::Empty>("CNActuator/BundleRestartTrigger", 10);
	ros::Publisher vrtPub = node.advertise<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/VisionRelocTrigger", 10);
	ros::Publisher mbcPub = node.advertise<msl_actuator_msgs::MotionBurst>("CNActuator/MotionBurst", 10);
	ros::Publisher hbiPub = node.advertise<msl_actuator_msgs::HaveBallInfo>("HaveBallInfo", 10);
	// ros::Publisher imuPub = node.advertise<YYeigene msg bauenYY>("IMU", 10);

	std::thread th_controlBHRight(controlBHRight);
	std::thread th_controlBHLeft(controlBHLeft);
	std::thread th_controlShovel(contolShovelSelect);
	std::thread th_lightbarrier(getLightbarrier, &hbiPub);
	std::thread th_switches(getSwitches, &bsPub, &brtPub, &vrtPub);

	// Shovel Init
	ShovelSelect.setPeriodTime(20000);			// in us - 20ms Periodendauer
	ShovelSelect.setSpaceRatioTime(1000);		// in us - Werte zwischen 1ms und 2ms
	shovel.enabled = false;

	// I2C
	bool i2c = myI2C.open(ReadWrite | NonBlock);			// Gyro: 0x69, Accel: 0x53, Magnet: 0x1E, Thermo: 0x77
	bool spi = mySpi.open(ReadWrite);

	std::cout << "SPI: " << spi << std::endl;
	std::cout << "I2C: " << i2c << std::endl;

	uint16_t count = 0;

	BlackGPIO test66(GPIO_66, output, FastMode);

	(void) signal(SIGINT, exit_program);
	// Frequency set with loop_rate()
	while(ros::ok() && !ex) {
		gettimeofday(&time_now, NULL);
		timeval vorher, mitte, nachher;

		gettimeofday(&vorher, NULL);

		// Thread Notify

		for (int i=0; i<5; i++) {
			threw[i].notify = true;
			threw[i].cv.notify_all();
		}

		gettimeofday(&mitte, NULL);

		// auf beenden aller Threads warten
		while (!th_activ || (!threw[0].notify && !threw[1].notify && !threw[2].notify && !threw[3].notify && !threw[4].notify)) {
			usleep(1000);
		}

		gettimeofday(&nachher, NULL);

		long int diffus = TIMEDIFFUS(nachher, vorher);
		long int diffms = TIMEDIFFMS(nachher, vorher);

		std::cout << "Zeit -gesamt: " << diffms << " - " << diffus << std::endl;

		diffus = TIMEDIFFUS(mitte, vorher);
		diffms = TIMEDIFFMS(mitte, vorher);
		std::cout << "Zeit -mitte: " << diffms << " - " << diffus << std::endl;

		diffus = TIMEDIFFUS(nachher, mitte);
		diffms = TIMEDIFFMS(nachher, mitte);
		std::cout << "Zeit -mitend: " << diffms << " - " << diffus << std::endl;

		diffus = TIMEDIFFUS(le, ls);
		std::cout << "Le : " << diffus << std::endl;

		diffus = TIMEDIFFUS(re, rs);
		std::cout << "Ri : " << diffus << std::endl;

		diffus = TIMEDIFFUS(se, ss);
		std::cout << "Sh : " << diffus << std::endl;

		diffus = TIMEDIFFUS(lie, lim);
		diffms = TIMEDIFFUS(lim, lis);
		std::cout << "Li : " << diffms << " - " << diffus << std::endl;

		diffus = TIMEDIFFUS(swe, sws);
		diffms = TIMEDIFFMS(swe, sws);
		std::cout << "Sw : " << diffus << std::endl;



		// MotionBurst

		// IMU

		ros::spinOnce();
		loop_rate.sleep();
	}

	std::cout << "Programm beendet." << std::endl;

	return 0;
}
