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
std::condition_variable		cv, cv_main;

timeval		ls, le, rs, re, ss, se, lis, lie, sws, swe;

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
	while(th_activ) {
		std::unique_lock<std::mutex> lck(c_bhl.mtx);
		cv.wait(lck, [&] { return !th_activ || c_bhl.notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		gettimeofday(&ls, NULL);
		BH_left.controlBallHandling();
		gettimeofday(&le, NULL);

		c_bhl.notify = false;
		cv_main.notify_all();
	}
}

void controlBHRight() {
	while(th_activ) {
		std::unique_lock<std::mutex> lck(c_bhr.mtx);
		cv.wait(lck, [&] { return !th_activ || c_bhr.notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		gettimeofday(&rs, NULL);
		BH_right.controlBallHandling();
		gettimeofday(&re, NULL);

		c_bhr.notify = false;
		cv_main.notify_all();
	}
}

void contolShovelSelect() {
	std::cout << "Hi in Shovel" << std::endl;
	while(th_activ) {
		std::unique_lock<std::mutex> lck(c_shovel.mtx);
		cv.wait(lck, [&] { return !th_activ || c_shovel.notify; }); // protection against spurious wake-ups
		std::cout << "While Shovel Start" << std::endl;
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

		c_shovel.notify = false;
		cv_main.notify_all();
		std::cout << "While Shovel Ende" << std::endl;
	}
}

void getLightbarrier(ros::Publisher *hbiPub) {
	while(th_activ) {
		std::unique_lock<std::mutex> lck(c_light.mtx);
		cv.wait(lck, [&] { return !th_activ || c_light.notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		gettimeofday(&lis, NULL);
		msl_actuator_msgs::HaveBallInfo msg;
		uint16_t value = ADC_Light.getNumericValue();

		if (value > LIGHTBARRIER_THRESHOLD) {
			msg.haveBall = true;
			ROS_INFO("HaveBall: True");
		} else {
			msg.haveBall = false;
			ROS_INFO("HaveBall: False");
		}
		hbiPub->publish(msg);
		gettimeofday(&lie, NULL);

		c_light.notify = false;
		cv_main.notify_all();
	}
}

void getSwitches(ros::Publisher *bsPub, ros::Publisher *brtPub, ros::Publisher *vrtPub) {
	while(th_activ) {
		std::unique_lock<std::mutex> lck(c_switches.mtx);
		cv.wait(lck, [&] { return !th_activ || c_switches.notify; }); // protection against spurious wake-ups
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

		c_switches.notify = false;
		cv_main.notify_all();
	}
}

void exit_program(int sig) {
	ex = true;
	std::cout << "Programm wird beendet." << std::endl;
	th_activ = false;
	cv.notify_all();
	cv_main.notify_all();
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
	// ros::Publisher mbcPub = node.advertise<msl_actuator_msgs::MotionBurst>("CNActuator/MotionBurst", 10);
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

		gettimeofday(&vorher, NULL);

		// Thread Notify

		c_bhl.notify = true;
		c_bhr.notify = true;
		c_shovel.notify = true;
		c_light.notify = true;
		c_switches.notify = true;

		cv.notify_all();

		gettimeofday(&mitte, NULL);

		// auf beenden aller Threads warten
		{
			std::unique_lock<std::mutex> lck(mtx);
			cv_main.wait(lck, [&]
			{ return !th_activ || (!c_bhl.notify && !c_bhr.notify && !c_shovel.notify && !c_light.notify && !c_switches.notify); });
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
		diffms = TIMEDIFFMS(le, ls);
		std::cout << "Le : " << diffms << " - " << diffus << std::endl;

		diffus = TIMEDIFFUS(re, rs);
		diffms = TIMEDIFFMS(re, rs);
		std::cout << "Ri : " << diffms << " - " << diffus << std::endl;

		diffus = TIMEDIFFUS(se, ss);
		diffms = TIMEDIFFMS(se, ss);
		std::cout << "Sh : " << diffms << " - " << diffus << std::endl;

		diffus = TIMEDIFFUS(lie, lis);
		diffms = TIMEDIFFMS(lie, lis);
		std::cout << "Li : " << diffms << " - " << diffus << std::endl;

		diffus = TIMEDIFFUS(swe, sws);
		diffms = TIMEDIFFMS(swe, sws);
		std::cout << "Sw : " << diffms << " - " << diffus << std::endl;

		std::cout << ls.tv_sec << " " << ls.tv_usec << std::endl;
		std::cout << rs.tv_sec << " " << rs.tv_usec << std::endl;
		std::cout << ss.tv_sec << " " << ss.tv_usec << std::endl;
		std::cout << lis.tv_sec << " " << lis.tv_usec << std::endl;
		std::cout << sws.tv_sec << " " << sws.tv_usec << std::endl;



		// MotionBurst

		// IMU

/*		timeval vorher, nachher;
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


		count++;*/


		//std_msgs::String msg;
		//std::stringstream ss;
		//ss << "hello world " << count;
		//msg.data = ss.str();

		ros::spinOnce();
		loop_rate.sleep();
	}

	th_controlBHLeft.join();
	th_controlBHRight.join();
	th_controlShovel.join();
	th_lightbarrier.join();
	th_switches.join();

	std::cout << "Programm beendet." << std::endl;

	return 0;
}
